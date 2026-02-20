from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    # Include the full move_group launch (controllers, TF, RViz, move_group)
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("arm_moveit_config"),
                "launch",
                "move_group.launch.py",
            ])
        ),
        launch_arguments={'use_gazebo': 'true'}.items(),
    )

    # Pose estimator — detects AprilTag and publishes /detected_pose
    pose_estimator = Node(
        package="pick_place_control",
        executable="pose_estimator",
        output="screen",
        parameters=[{'use_sim_time': True}],
    )


    # Pick-and-place node — delayed to let move_group fully initialize
    pick_place_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="pick_place_control",
                executable="pick_place_node",
                output="screen",
                parameters=[{'use_sim_time': True}],
            )

        ],
    )

    return LaunchDescription([
        move_group_launch,
        pose_estimator,
        pick_place_node,
    ])

