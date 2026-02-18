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
from geometry_msgs.msg import Pose

# ── Predefined poses (matching scene_objects.yaml) ──────────────────────────

# "ready" pose from SRDF — a safe home position
HOME_JOINT_POSITIONS = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]

# Pre-grasp: 15cm above the red_box
# red_box is at z=0.42, panda_link8 needs ~12cm above fingertips
# so link8 at z=0.42+0.12+0.15 = 0.69
PRE_GRASP_POSITION = [0.6, 0.0, 0.69]
PRE_GRASP_QUAT_XYZW = [1.0, 0.0, 0.0, 0.0]  # pointing straight down

# Grasp: fingertips at red_box level (z=0.42), link8 ~12cm above
GRASP_POSITION = [0.6, 0.0, 0.54]
GRASP_QUAT_XYZW = [1.0, 0.0, 0.0, 0.0]

# Lift: raise 15cm after grasping
LIFT_POSITION = [0.6, 0.0, 0.69]
LIFT_QUAT_XYZW = [1.0, 0.0, 0.0, 0.0]

# Place: move to a different location on the table
PLACE_POSITION = [0.5, 0.3, 0.54]
PLACE_QUAT_XYZW = [1.0, 0.0, 0.0, 0.0]

# Retreat: move up after placing to clear the object
RETREAT_POSITION = [0.5, 0.3, 0.69]
RETREAT_QUAT_XYZW = [1.0, 0.0, 0.0, 0.0]

# Gripper joint positions
GRIPPER_OPEN = [0.04, 0.04]
GRIPPER_CLOSED = [0.0, 0.0]
GRIPPER_JOINT_NAMES = ["panda_finger_joint1", "panda_finger_joint2"]

# Object ID in the planning scene (must match scene_objects.yaml)
OBJECT_ID = "red_box"


class PickPlaceNode(Node):
    def __init__(self):
        super().__init__("pick_place_node")
        self.get_logger().info("Initializing Pick-and-Place node...")

        callback_group = ReentrantCallbackGroup()

        # ── Arm interface ──
        # Use panda_link8 (not panda_hand_tcp which doesn't exist in our URDF)
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
        # Use a separate MoveIt2 instance for the "hand" planning group
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

        # Start the sequence after delay (let move_group fully initialize)
        self._startup_timer = self.create_timer(
            5.0, self.run_pick_and_place, callback_group=callback_group
        )

    def run_pick_and_place(self):
        self._startup_timer.cancel()

        self.get_logger().info("=== Starting Pick-and-Place Sequence ===")

        # Step 1: Move to home (BEFORE loading collision objects)
        self._move_to_home()

        # Step 2: Load collision objects into planning scene
        self._load_planning_scene()

        # Step 3: Open gripper
        self._open_gripper()

        # Step 4: Move to pre-grasp
        self._move_to_pose("PRE-GRASP", PRE_GRASP_POSITION, PRE_GRASP_QUAT_XYZW)

        # Step 5: Descend to grasp pose
        self._move_to_pose("GRASP", GRASP_POSITION, GRASP_QUAT_XYZW, cartesian=True)

        # Step 6: Close gripper
        self._close_gripper()

        # Step 7: Attach object in planning scene
        self._attach_object()

        # Step 8: Lift
        self._move_to_pose("LIFT", LIFT_POSITION, LIFT_QUAT_XYZW, cartesian=True)

        # Step 9: Move to place pose
        self._move_to_pose("PLACE", PLACE_POSITION, PLACE_QUAT_XYZW)

        # Step 10: Open gripper
        self._open_gripper()

        # Step 11: Detach object
        self._detach_object()

        # Step 12: Retreat up to clear the object
        self._move_to_pose("RETREAT", RETREAT_POSITION, RETREAT_QUAT_XYZW, cartesian=True)

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
