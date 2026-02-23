from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EqualsSubstitution, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    detector_arg = DeclareLaunchArgument(
        'detector',
        default_value='apriltag',
        description='Detector backend to use: apriltag | yolo | vlm',
    )
    detector = LaunchConfiguration('detector')

    # Include the full move_group launch (controllers, TF, RViz, move_group)
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('arm_moveit_config'),
                'launch',
                'move_group.launch.py',
            ])
        ),
        launch_arguments={'use_gazebo': 'true'}.items(),
    )

    # Phase 2A — AprilTag + solvePnP detector
    pose_estimator = Node(
        package='pick_place_control',
        executable='pose_estimator',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(EqualsSubstitution(detector, 'apriltag')),
    )

    # Phase 2B — YOLOv8 + depth centroid detector
    yolo_detector = Node(
        package='pick_place_control',
        executable='yolo_detector',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(EqualsSubstitution(detector, 'yolo')),
    )

    # Phase 2C — OWL-ViT open-vocabulary detector
    vlm_detector = Node(
        package='pick_place_control',
        executable='vlm_detector',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'target_object': 'blue box'},
        ],
        condition=IfCondition(EqualsSubstitution(detector, 'vlm')),
    )

    # Pick-and-place orchestrator — delayed to let move_group fully initialize
    pick_place_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='pick_place_control',
                executable='pick_place_node',
                output='screen',
                parameters=[{'use_sim_time': True}],
            )
        ],
    )

    return LaunchDescription([
        detector_arg,
        move_group_launch,
        pose_estimator,
        yolo_detector,
        vlm_detector,
        pick_place_node,
    ])
