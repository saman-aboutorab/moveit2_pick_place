import numpy as np
import cv2
from PIL import Image as PILImage
from scipy.spatial.transform import Rotation

import torch
from transformers import OwlViTProcessor, OwlViTForObjectDetection

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped

# cv_bridge is deliberately NOT used here — its C extension is compiled against
# NumPy 1.x and crashes at runtime when NumPy 2.x (installed by transformers/torch)
# is active. ROS Image messages are converted manually using numpy frombuffer instead.


class VlmDetector(Node):
    """
    Detects objects using OWL-ViT (open-vocabulary) + depth centroid for 3D world-frame position.

    Pipeline:
      1. OWL-ViT inference on RGB frame using text prompt → bounding box center pixel (u, v)
      2. Sample depth d at (u, v) from depth image
      3. Back-project: (u, v, d) → 3D camera-optical-frame point
      4. Transform camera-optical → world frame using known camera extrinsics
      5. Publish PoseStamped on /detected_pose (same topic as pose_estimator.py / yolo_detector.py)

    For Gazebo simulation: plain geometric shapes may fall below OWL-ViT confidence on first
    runs (model not fine-tuned on sim geometry). use_color_fallback (default True) enables
    HSV blue-object masking to guarantee a detection in simulation.
    For real objects: set use_color_fallback:=false — OWL-ViT generalises well to real scenes.
    """

    # Camera pose in world frame (matches pick_place.sdf)
    # pose: x=0.6 y=0.0 z=1.0 roll=0 pitch=π/2 yaw=0 (straight down)
    CAM_POSITION = np.array([0.6, 0.0, 1.0])
    CAM_RPY = np.array([0.0, 1.5708, 0.0])

    def __init__(self):
        super().__init__('vlm_detector')

        # --- ROS parameters ---
        self.declare_parameter('target_object', 'blue box')
        self.declare_parameter('model', 'google/owlvit-base-patch32')
        self.declare_parameter('conf_threshold', 0.05)   # low threshold suits Gazebo geometry
        self.declare_parameter('use_color_fallback', True)

        self.target_object = self.get_parameter('target_object').value
        model_name = self.get_parameter('model').value
        self.conf_threshold = self.get_parameter('conf_threshold').value
        self.use_color_fallback = self.get_parameter('use_color_fallback').value

        self.camera_matrix = None
        self.depth_image = None

        # Precompute world → camera-optical transform (same as yolo_detector.py)
        self.T_world_to_optical = self._build_camera_transform()

        # Load OWL-ViT (downloads weights on first run, then cached in ~/.cache/huggingface)
        self.get_logger().info(f'Loading OWL-ViT model: {model_name} ...')
        self.processor = OwlViTProcessor.from_pretrained(model_name)
        self.owl_model = OwlViTForObjectDetection.from_pretrained(model_name)
        self.owl_model.eval()
        self.get_logger().info(
            f'OWL-ViT loaded. Target object: "{self.target_object}" '
            f'(conf_threshold={self.conf_threshold})'
        )

        # --- Subscribers ---
        self.create_subscription(CameraInfo, '/rgbd_camera/camera_info',
                                 self._camera_info_cb, 10)
        self.create_subscription(Image, '/rgbd_camera/image',
                                 self._image_cb, 10)
        self.create_subscription(Image, '/rgbd_camera/depth_image',
                                 self._depth_cb, 10)

        # --- Publisher ---
        self.pose_pub = self.create_publisher(PoseStamped, '/detected_pose', 10)

        self.get_logger().info('VLM detector ready — waiting for camera data...')

    # ------------------------------------------------------------------ #
    #  Camera transform (identical to yolo_detector.py)                   #
    # ------------------------------------------------------------------ #

    def _build_camera_transform(self):
        """Build 4x4 homogeneous transform: world frame → camera optical frame."""
        R_world_to_link = Rotation.from_euler('xyz', self.CAM_RPY).as_matrix()
        T_world_to_link = np.eye(4)
        T_world_to_link[:3, :3] = R_world_to_link
        T_world_to_link[:3, 3] = self.CAM_POSITION

        # Gazebo camera link axes → OpenCV optical frame axes
        # Link:    +X forward, +Y left,  +Z up
        # Optical: +X right,   +Y down,  +Z forward (into scene)
        R_link_to_optical = np.array([
            [ 0,  0, 1],
            [-1,  0, 0],
            [ 0, -1, 0],
        ], dtype=np.float64)
        T_link_to_optical = np.eye(4)
        T_link_to_optical[:3, :3] = R_link_to_optical

        return T_world_to_link @ T_link_to_optical

    # ------------------------------------------------------------------ #
    #  Manual ROS Image conversion (no cv_bridge)                         #
    # ------------------------------------------------------------------ #

    def _ros_image_to_rgb(self, msg):
        """Convert sensor_msgs/Image to uint8 RGB numpy array (H x W x 3)."""
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        if msg.encoding == 'bgr8':
            return arr[:, :, ::-1].copy()
        # rgb8 — Gazebo RGBD camera default
        return arr.copy()

    def _ros_depth_to_float32(self, msg):
        """Convert sensor_msgs/Image (32FC1) to float32 numpy array (H x W), metres."""
        return np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width).copy()

    # ------------------------------------------------------------------ #
    #  ROS callbacks                                                       #
    # ------------------------------------------------------------------ #

    def _camera_info_cb(self, msg):
        """Store intrinsics once (fx, fy, cx, cy from K matrix)."""
        if self.camera_matrix is not None:
            return
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.get_logger().info('Camera intrinsics received.')

    def _depth_cb(self, msg):
        """Cache latest depth image (float32, metres)."""
        self.depth_image = self._ros_depth_to_float32(msg)

    def _image_cb(self, msg):
        """Main callback: detect object via OWL-ViT, project to world, publish pose."""
        if self.camera_matrix is None or self.depth_image is None:
            return

        rgb = self._ros_image_to_rgb(msg)   # H x W x 3, uint8 RGB
        result = self._detect_object(rgb)
        if result is None:
            return

        u_f, v_f = result

        # --- Sample depth at bounding box centre ---
        h, w = self.depth_image.shape[:2]
        u = int(np.clip(u_f, 0, w - 1))
        v = int(np.clip(v_f, 0, h - 1))
        depth = float(self.depth_image[v, u])

        if not np.isfinite(depth) or depth <= 0.0:
            self.get_logger().warn('Invalid depth at detection centre, skipping.')
            return

        # --- Back-project pixel + depth → camera optical frame ---
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]

        x_cam = (u - cx) * depth / fx
        y_cam = (v - cy) * depth / fy
        z_cam = depth

        # --- Transform camera optical frame → world frame ---
        p_cam_h = np.array([x_cam, y_cam, z_cam, 1.0])
        p_world = self.T_world_to_optical @ p_cam_h

        # --- Publish PoseStamped ---
        pose_msg = PoseStamped()
        pose_msg.header.stamp = msg.header.stamp
        pose_msg.header.frame_id = 'world'
        pose_msg.pose.position.x = p_world[0]
        pose_msg.pose.position.y = p_world[1]
        pose_msg.pose.position.z = p_world[2]
        pose_msg.pose.orientation.w = 1.0

        self.pose_pub.publish(pose_msg)
        self.get_logger().info(
            f'"{self.target_object}" at world: '
            f'x={p_world[0]:.3f} y={p_world[1]:.3f} z={p_world[2]:.3f}  '
            f'(depth={depth:.3f}m)',
            throttle_duration_sec=2.0,
        )

    # ------------------------------------------------------------------ #
    #  Detection logic                                                     #
    # ------------------------------------------------------------------ #

    def _detect_object(self, rgb):
        """
        Returns (cx, cy) float pixel of detected object centre, or None.

        rgb: H x W x 3 uint8 numpy array in RGB order.

        Priority:
          1. OWL-ViT text-grounded detection using target_object parameter
          2. HSV colour masking for blue objects (Gazebo sim fallback)
        """
        # -- 1. OWL-ViT open-vocabulary inference --
        pil_image = PILImage.fromarray(rgb)

        inputs = self.processor(
            text=[[self.target_object]],
            images=pil_image,
            return_tensors='pt',
        )

        with torch.no_grad():
            outputs = self.owl_model(**inputs)

        img_h, img_w = rgb.shape[:2]
        target_sizes = torch.tensor([[img_h, img_w]])
        results = self.processor.image_processor.post_process_object_detection(
            outputs=outputs,
            threshold=self.conf_threshold,
            target_sizes=target_sizes,
        )[0]

        if len(results['scores']) > 0:
            best_idx = int(results['scores'].argmax())
            score = float(results['scores'][best_idx])
            box = results['boxes'][best_idx].tolist()   # [x1, y1, x2, y2]
            cx = (box[0] + box[2]) / 2.0
            cy = (box[1] + box[3]) / 2.0
            self.get_logger().info(
                f'OWL-ViT: "{self.target_object}" conf={score:.3f} '
                f'center=({cx:.0f},{cy:.0f})',
                throttle_duration_sec=2.0,
            )
            return cx, cy

        if not self.use_color_fallback:
            return None

        # -- 2. HSV colour masking fallback (for Gazebo blue box) --
        # cv2 expects BGR; flip channels from the RGB input
        bgr = rgb[:, :, ::-1].copy()
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        # Blue hue in OpenCV HSV: ~100–130 (scale 0–180)
        mask = cv2.inRange(hsv, np.array([100, 100, 80]), np.array([130, 255, 255]))

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < 100:   # ignore noise
            return None

        M = cv2.moments(largest)
        if M['m00'] == 0:
            return None

        cx = M['m10'] / M['m00']
        cy = M['m01'] / M['m00']
        self.get_logger().info(
            f'Color fallback: blue object at pixel ({cx:.0f}, {cy:.0f})',
            throttle_duration_sec=5.0,
        )
        return cx, cy


def main(args=None):
    rclpy.init(args=args)
    node = VlmDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
