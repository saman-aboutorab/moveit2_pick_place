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
        )
    )

    # Pick-and-place node — delayed to let move_group fully initialize
    pick_place_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="pick_place_control",
                executable="pick_place_node",
                output="screen",
            )
        ],
    )

    return LaunchDescription([
        move_group_launch,
        pick_place_node,
    ])
