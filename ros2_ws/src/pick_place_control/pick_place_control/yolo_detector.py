import numpy as np
import cv2
from scipy.spatial.transform import Rotation
from ultralytics import YOLO

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge


class YoloDetector(Node):
    """
    Detects objects using YOLOv8 (RGB) + depth centroid for 3D world-frame position.

    Pipeline:
      1. YOLOv8 inference on RGB frame → bounding box center pixel (u, v)
      2. Sample depth d at (u, v) from depth image
      3. Back-project: (u, v, d) → 3D camera-optical-frame point
      4. Transform camera-optical → world frame using known camera extrinsics
      5. Publish PoseStamped on /detected_pose (same topic as pose_estimator.py)

    For Gazebo simulation: plain geometric shapes won't match COCO classes, so
    use_color_fallback (default True) enables HSV red-object masking as a fallback.
    For real objects: set use_color_fallback:=false and tune target_class.
    """

    # Camera pose in world frame (matches pick_place.sdf)
    # pose: x=0.6 y=0.0 z=1.0 roll=0 pitch=π/2 yaw=0 (straight down)
    CAM_POSITION = np.array([0.6, 0.0, 1.0])
    CAM_RPY = np.array([0.0, 1.5708, 0.0])

    def __init__(self):
        super().__init__('yolo_detector')

        # --- ROS parameters ---
        self.declare_parameter('model', 'yolov8n.pt')
        self.declare_parameter('target_class', 'bottle')
        self.declare_parameter('conf_threshold', 0.3)
        self.declare_parameter('use_color_fallback', True)

        model_path = self.get_parameter('model').value
        self.target_class = self.get_parameter('target_class').value
        self.conf_threshold = self.get_parameter('conf_threshold').value
        self.use_color_fallback = self.get_parameter('use_color_fallback').value

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.depth_image = None

        # Precompute world → camera-optical transform (inverted later for world coords)
        self.T_world_to_optical = self._build_camera_transform()

        # Load YOLO model (downloads weights on first run)
        self.yolo = YOLO(model_path)
        self.get_logger().info(f'YOLOv8 loaded: {model_path}, target class: {self.target_class}')

        # --- Subscribers ---
        self.create_subscription(CameraInfo, '/rgbd_camera/camera_info',
                                 self._camera_info_cb, 10)
        self.create_subscription(Image, '/rgbd_camera/image',
                                 self._image_cb, 10)
        self.create_subscription(Image, '/rgbd_camera/depth_image',
                                 self._depth_cb, 10)

        # --- Publisher ---
        self.pose_pub = self.create_publisher(PoseStamped, '/detected_pose', 10)

        self.get_logger().info('YOLO detector ready — waiting for camera data...')

    # ------------------------------------------------------------------ #
    #  Camera transform (identical logic to pose_estimator.py)            #
    # ------------------------------------------------------------------ #

    def _build_camera_transform(self):
        """Build 4x4 homogeneous transform: world frame → camera optical frame."""
        # Camera link pose in world (from SDF)
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
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def _image_cb(self, msg):
        """Main callback: detect object, project to world, publish pose."""
        if self.camera_matrix is None or self.depth_image is None:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        result = self._detect_object(frame)
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
        # T_world_to_optical maps optical→world (same convention as pose_estimator.py)
        p_cam_h = np.array([x_cam, y_cam, z_cam, 1.0])
        p_world = self.T_world_to_optical @ p_cam_h

        # --- Publish PoseStamped ---
        # Orientation is identity (gripper always approaches straight down).
        # pick_place_node uses position only and applies its own DOWN_QUAT.
        pose_msg = PoseStamped()
        pose_msg.header.stamp = msg.header.stamp
        pose_msg.header.frame_id = 'world'
        pose_msg.pose.position.x = p_world[0]
        pose_msg.pose.position.y = p_world[1]
        pose_msg.pose.position.z = p_world[2]
        pose_msg.pose.orientation.w = 1.0

        self.pose_pub.publish(pose_msg)
        self.get_logger().info(
            f'Object detected at world: '
            f'x={p_world[0]:.3f} y={p_world[1]:.3f} z={p_world[2]:.3f}  '
            f'(depth={depth:.3f}m)',
            throttle_duration_sec=2.0,
        )

    # ------------------------------------------------------------------ #
    #  Detection logic                                                     #
    # ------------------------------------------------------------------ #

    def _detect_object(self, frame):
        """
        Returns (cx, cy) float pixel of detected object centre, or None.

        Priority:
          1. YOLOv8 detection matching target_class
          2. HSV color masking for red objects (Gazebo sim fallback)
        """
        # -- 1. YOLO inference --
        results = self.yolo(frame, verbose=False, conf=self.conf_threshold)
        for r in results:
            for box in r.boxes:
                class_name = self.yolo.names[int(box.cls)]
                if class_name == self.target_class:
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    cx = (x1 + x2) / 2.0
                    cy = (y1 + y2) / 2.0
                    self.get_logger().info(
                        f'YOLO detected "{class_name}" conf={float(box.conf):.2f} '
                        f'center=({cx:.0f},{cy:.0f})',
                        throttle_duration_sec=2.0,
                    )
                    return cx, cy

        if not self.use_color_fallback:
            return None

        # -- 2. HSV color masking fallback (for Gazebo red box) --
        # Red wraps around in HSV, so we need two ranges.
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_lo = cv2.inRange(hsv, np.array([0,   100, 100]), np.array([10,  255, 255]))
        mask_hi = cv2.inRange(hsv, np.array([160, 100, 100]), np.array([180, 255, 255]))
        mask = cv2.bitwise_or(mask_lo, mask_hi)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < 100:  # ignore noise
            return None

        M = cv2.moments(largest)
        if M['m00'] == 0:
            return None

        cx = M['m10'] / M['m00']
        cy = M['m01'] / M['m00']
        self.get_logger().info(
            f'Color fallback: red object at pixel ({cx:.0f}, {cy:.0f})',
            throttle_duration_sec=5.0,
        )
        return cx, cy


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
