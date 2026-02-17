import os
import yaml
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose


class PlanningSceneLoader(Node):
    def __init__(self):
        super().__init__('planning_scene_loader')

        # Publisher for the planning scene (MoveIt2 listens on this topic)
        self.scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 10)

        # Load objects from YAML config
        config_path = os.path.join(
            get_package_share_directory('pick_place_control'),
            'config',
            'scene_objects.yaml',
        )
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        self.objects = config.get('scene_objects', [])
        self.get_logger().info(f'Loaded {len(self.objects)} scene objects from config')

        # Wait for move_group to be ready, then publish
        self.timer = self.create_timer(3.0, self.publish_scene)

    def publish_scene(self):
        # Only publish once
        self.timer.cancel()

        planning_scene_msg = PlanningScene()
        planning_scene_msg.is_diff = True  # Add to existing scene, don't replace

        for obj in self.objects:
            collision_object = CollisionObject()
            collision_object.header.frame_id = 'world'
            collision_object.id = obj['id']
            collision_object.operation = CollisionObject.ADD

            # Build the shape primitive
            primitive = SolidPrimitive()
            if obj['type'] == 'box':
                primitive.type = SolidPrimitive.BOX
                primitive.dimensions = obj['dimensions']  # [x, y, z]
            elif obj['type'] == 'cylinder':
                primitive.type = SolidPrimitive.CYLINDER
                primitive.dimensions = obj['dimensions']  # [height, radius]

            # Build the pose
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
            self.get_logger().info(f'Added collision object: {obj["id"]}')

        self.scene_pub.publish(planning_scene_msg)
        self.get_logger().info('Planning scene published!')


def main(args=None):
    rclpy.init(args=args)
    node = PlanningSceneLoader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
