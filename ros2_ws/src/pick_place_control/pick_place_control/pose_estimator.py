import numpy as np
import cv2
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge


class PoseEstimator(Node):
    """Detects AprilTag 36h11 ID 0 and publishes its 6D pose in world frame."""

    TAG_SIZE = 0.03  # Physical tag size in meters (matches texture on 0.04m box)

    # Camera pose in world frame (from pick_place.sdf)
    # pose: x=0.6 y=0.0 z=1.0 roll=0 pitch=π/2 yaw=0 (straight down)
    CAM_POSITION = np.array([0.6, 0.0, 1.0])
    CAM_RPY = np.array([0.0, 1.5708, 0.0])

    def __init__(self):
        super().__init__('pose_estimator')

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None

        # Precompute world-to-camera-optical transform
        self.T_world_to_optical = self._build_camera_transform()

        # AprilTag detector (OpenCV ArUco with tag36h11 dictionary)
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(dictionary, params)

        # 3D corners of the tag in tag frame (used by solvePnP)
        half = self.TAG_SIZE / 2.0
        self.object_points = np.array([
            [-half,  half, 0.0],  # top-left
            [ half,  half, 0.0],  # top-right
            [ half, -half, 0.0],  # bottom-right
            [-half, -half, 0.0],  # bottom-left
        ], dtype=np.float64)

        # Subscribers
        self.create_subscription(CameraInfo, '/rgbd_camera/camera_info',
                                 self.camera_info_cb, 10)
        self.create_subscription(Image, '/rgbd_camera/image',
                                 self.image_cb, 10)

        # Publisher
        self.pose_pub = self.create_publisher(PoseStamped, '/detected_pose', 10)

        self.get_logger().info('Pose estimator ready — waiting for camera data...')

    def _build_camera_transform(self):
        """Build 4x4 transform: world frame → camera optical frame."""
        # Step 1: Camera link pose in world (from SDF)
        R_world_to_link = Rotation.from_euler('xyz', self.CAM_RPY).as_matrix()
        T_world_to_link = np.eye(4)
        T_world_to_link[:3, :3] = R_world_to_link
        T_world_to_link[:3, 3] = self.CAM_POSITION

        # Step 2: Gazebo camera link → OpenCV optical frame
        # Link: +X forward, +Y left, +Z up
        # Optical: +X right, +Y down, +Z forward
        # Columns = optical axes in link coordinates:
        #   opt_X (right) = -link_Y, opt_Y (down) = -link_Z, opt_Z (fwd) = +link_X
        R_link_to_optical = np.array([
            [ 0,  0, 1],
            [-1,  0, 0],
            [ 0, -1, 0],
        ], dtype=np.float64)
        T_link_to_optical = np.eye(4)
        T_link_to_optical[:3, :3] = R_link_to_optical

        return T_world_to_link @ T_link_to_optical

    def camera_info_cb(self, msg):
        """Store camera intrinsics (only need first message)."""
        if self.camera_matrix is not None:
            return
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d, dtype=np.float64)
        self.get_logger().info('Camera intrinsics received.')

    def image_cb(self, msg):
        """Detect AprilTag and publish its world-frame pose."""
        if self.camera_matrix is None:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is None:
            return

        # Find tag ID 0
        for i, tag_id in enumerate(ids.flatten()):
            if tag_id != 0:
                continue

            # solvePnP: get tag pose in camera optical frame
            image_points = corners[i][0].astype(np.float64)
            success, rvec, tvec = cv2.solvePnP(
                self.object_points, image_points,
                self.camera_matrix, self.dist_coeffs,
            )
            if not success:
                continue

            # Build 4x4 transform: optical frame → tag
            R_optical_to_tag, _ = cv2.Rodrigues(rvec)
            T_optical_to_tag = np.eye(4)
            T_optical_to_tag[:3, :3] = R_optical_to_tag
            T_optical_to_tag[:3, 3] = tvec.flatten()

            # Chain: world → optical → tag = world → tag
            T_world_to_tag = self.T_world_to_optical @ T_optical_to_tag

            # Convert to PoseStamped
            pose_msg = PoseStamped()
            pose_msg.header.stamp = msg.header.stamp
            pose_msg.header.frame_id = 'world'

            pose_msg.pose.position.x = T_world_to_tag[0, 3]
            pose_msg.pose.position.y = T_world_to_tag[1, 3]
            pose_msg.pose.position.z = T_world_to_tag[2, 3]

            quat = Rotation.from_matrix(T_world_to_tag[:3, :3]).as_quat()
            pose_msg.pose.orientation.x = quat[0]
            pose_msg.pose.orientation.y = quat[1]
            pose_msg.pose.orientation.z = quat[2]
            pose_msg.pose.orientation.w = quat[3]

            self.pose_pub.publish(pose_msg)
            self.get_logger().info(
                f'Tag 0 detected at world: '
                f'x={T_world_to_tag[0,3]:.3f} '
                f'y={T_world_to_tag[1,3]:.3f} '
                f'z={T_world_to_tag[2,3]:.3f}',
                throttle_duration_sec=2.0,
            )
            break


def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
