import os
import yaml
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from pymoveit2 import MoveIt2
from pymoveit2.robots import panda
from ament_index_python.packages import get_package_share_directory
from moveit_msgs.msg import CollisionObject, PlanningScene as PlanningSceneMsg
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, PoseStamped

# ── Fixed poses ──────────────────────────────────────────────────────────────

HOME_JOINT_POSITIONS = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]

# Offsets from detected tag position (tag sits on box top)
HAND_LENGTH = 0.12        # panda_link8 → fingertips when pointing down
PRE_GRASP_HEIGHT = 0.15   # hover above grasp point

# Orientation: end effector pointing straight down
DOWN_QUAT_XYZW = [1.0, 0.0, 0.0, 0.0]

# Place: fixed location on the table (not vision-based)
PLACE_POSITION = [0.5, 0.3, 0.54]
RETREAT_POSITION = [0.5, 0.3, 0.69]

# Gripper joint positions
GRIPPER_OPEN = [0.04, 0.04]
GRIPPER_CLOSED = [0.0, 0.0]
GRIPPER_JOINT_NAMES = ["panda_finger_joint1", "panda_finger_joint2"]

OBJECT_ID = "red_box"


class PickPlaceNode(Node):
    def __init__(self):
        super().__init__("pick_place_node")
        self.get_logger().info("Initializing Pick-and-Place node...")

        callback_group = ReentrantCallbackGroup()

        self.detected_pose = None

        # ── Arm interface ──
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=panda.joint_names(),
            base_link_name=panda.base_link_name(),
            end_effector_name="panda_link8",
            group_name="panda_arm",
            callback_group=callback_group,
        )
        self.moveit2.max_velocity = 0.5
        self.moveit2.max_acceleration = 0.5
        self.moveit2.num_planning_attempts = 10
        self.moveit2.allowed_planning_time = 5.0

        # ── Gripper interface ──
        self.gripper = MoveIt2(
            node=self,
            joint_names=GRIPPER_JOINT_NAMES,
            base_link_name=panda.base_link_name(),
            end_effector_name="panda_hand",
            group_name="hand",
            callback_group=callback_group,
        )

        # ── Planning scene publisher ──
        self.scene_pub = self.create_publisher(PlanningSceneMsg, '/planning_scene', 10)

        # ── Subscribe to detected pose from pose_estimator ──
        self.create_subscription(
            PoseStamped, '/detected_pose', self._detected_pose_cb, 10,
        )

        # Poll for first detection before starting sequence
        self._wait_timer = self.create_timer(
            1.0, self._wait_for_detection, callback_group=callback_group,
        )
        self.get_logger().info("Waiting for object detection on /detected_pose...")

    def _detected_pose_cb(self, msg):
        self.detected_pose = msg

    def _wait_for_detection(self):
        if self.detected_pose is None:
            return
        self._wait_timer.cancel()
        pos = self.detected_pose.pose.position
        self.get_logger().info(
            f"Object detected at x={pos.x:.3f} y={pos.y:.3f} z={pos.z:.3f} — starting sequence"
        )
        self.run_pick_and_place()

    def run_pick_and_place(self):
        self.get_logger().info("=== Starting Pick-and-Place Sequence ===")

        # Compute pick poses from detected tag position
        tag = self.detected_pose.pose.position
        grasp_z = tag.z + HAND_LENGTH       # link8 height so fingertips reach tag z
        pre_grasp_z = grasp_z + PRE_GRASP_HEIGHT

        grasp_pos = [tag.x, tag.y, grasp_z]
        pre_grasp_pos = [tag.x, tag.y, pre_grasp_z]
        lift_pos = [tag.x, tag.y, pre_grasp_z]

        self.get_logger().info(
            f"  Grasp: {grasp_pos}  Pre-grasp/Lift: {pre_grasp_pos}"
        )

        # Step 1: Move to home (BEFORE loading collision objects)
        self._move_to_home()

        # Step 2: Load collision objects into planning scene
        self._load_planning_scene()

        # Step 3: Open gripper
        self._open_gripper()

        # Step 4: Move to pre-grasp
        self._move_to_pose("PRE-GRASP", pre_grasp_pos, DOWN_QUAT_XYZW)

        # Step 5: Descend to grasp pose
        self._move_to_pose("GRASP", grasp_pos, DOWN_QUAT_XYZW, cartesian=True)

        # Step 6: Close gripper
        self._close_gripper()

        # Step 7: Attach object in planning scene
        self._attach_object()

        # Step 8: Lift
        self._move_to_pose("LIFT", lift_pos, DOWN_QUAT_XYZW, cartesian=True)

        # Step 9: Move to place pose
        self._move_to_pose("PLACE", PLACE_POSITION, DOWN_QUAT_XYZW)

        # Step 10: Open gripper
        self._open_gripper()

        # Step 11: Detach object
        self._detach_object()

        # Step 12: Retreat up to clear the object
        self._move_to_pose("RETREAT", RETREAT_POSITION, DOWN_QUAT_XYZW, cartesian=True)

        # Step 13: Return to home
        self._move_to_home()

        self.get_logger().info("=== Pick-and-Place Sequence Complete! ===")

    # ── Helper methods ───────────────────────────────────────────────────────

    def _move_to_home(self):
        self.get_logger().info("[HOME] Moving to home position...")
        self.moveit2.move_to_configuration(HOME_JOINT_POSITIONS)
        self.moveit2.wait_until_executed()
        self.get_logger().info("[HOME] Done.")

    def _move_to_pose(self, label, position, quat_xyzw, cartesian=False):
        self.get_logger().info(f"[{label}] Moving to position {position}...")
        self.moveit2.move_to_pose(
            position=position,
            quat_xyzw=quat_xyzw,
            cartesian=cartesian,
        )
        self.moveit2.wait_until_executed()
        self.get_logger().info(f"[{label}] Done.")

    def _open_gripper(self):
        self.get_logger().info("[GRIPPER] Opening...")
        self.gripper.move_to_configuration(GRIPPER_OPEN, joint_names=GRIPPER_JOINT_NAMES)
        self.gripper.wait_until_executed()
        self.get_logger().info("[GRIPPER] Open.")

    def _close_gripper(self):
        self.get_logger().info("[GRIPPER] Closing...")
        self.gripper.move_to_configuration(GRIPPER_CLOSED, joint_names=GRIPPER_JOINT_NAMES)
        self.gripper.wait_until_executed()
        self.get_logger().info("[GRIPPER] Closed.")

    def _attach_object(self):
        self.get_logger().info(f"[SCENE] Attaching '{OBJECT_ID}' to end effector...")
        self.moveit2.attach_collision_object(
            id=OBJECT_ID,
            link_name="panda_link8",
            touch_links=[
                "panda_hand",
                "panda_leftfinger",
                "panda_rightfinger",
            ],
        )
        self.get_logger().info(f"[SCENE] '{OBJECT_ID}' attached.")

    def _detach_object(self):
        self.get_logger().info(f"[SCENE] Detaching '{OBJECT_ID}'...")
        self.moveit2.detach_collision_object(id=OBJECT_ID)
        self.get_logger().info(f"[SCENE] '{OBJECT_ID}' detached.")

    def _load_planning_scene(self):
        self.get_logger().info("[SCENE] Loading collision objects...")
        config_path = os.path.join(
            get_package_share_directory('pick_place_control'),
            'config',
            'scene_objects.yaml',
        )
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        planning_scene_msg = PlanningSceneMsg()
        planning_scene_msg.is_diff = True

        for obj in config.get('scene_objects', []):
            collision_object = CollisionObject()
            collision_object.header.frame_id = 'world'
            collision_object.id = obj['id']
            collision_object.operation = CollisionObject.ADD

            primitive = SolidPrimitive()
            if obj['type'] == 'box':
                primitive.type = SolidPrimitive.BOX
                primitive.dimensions = obj['dimensions']
            elif obj['type'] == 'cylinder':
                primitive.type = SolidPrimitive.CYLINDER
                primitive.dimensions = obj['dimensions']

            pose = Pose()
            pose.position.x = obj['position'][0]
            pose.position.y = obj['position'][1]
            pose.position.z = obj['position'][2]
            pose.orientation.x = obj['orientation'][0]
            pose.orientation.y = obj['orientation'][1]
            pose.orientation.z = obj['orientation'][2]
            pose.orientation.w = obj['orientation'][3]

            collision_object.primitives.append(primitive)
            collision_object.primitive_poses.append(pose)
            planning_scene_msg.world.collision_objects.append(collision_object)
            self.get_logger().info(f"[SCENE] Added: {obj['id']}")

        self.scene_pub.publish(planning_scene_msg)
        self.get_logger().info("[SCENE] Planning scene published!")


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceNode()

    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor.spin()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
