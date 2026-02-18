import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    ros2_control_hardware_type = DeclareLaunchArgument(
        "ros2_control_hardware_type",
        default_value="mock_components",
        description="ROS 2 control hardware interface type",
    )

    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(
            file_path=os.path.join(
                get_package_share_directory("arm_bringup"),
                "urdf",
                "panda.urdf.xacro",
            ),
            mappings={
                "ros2_control_hardware_type": LaunchConfiguration(
                    "ros2_control_hardware_type"
                )
            },
        )
        .robot_description_semantic(
            file_path=os.path.join(
                get_package_share_directory("arm_moveit_config"),
                "config",
                "panda.srdf",
            )
        )
        .robot_description_kinematics(
            file_path=os.path.join(
                get_package_share_directory("arm_moveit_config"),
                "config",
                "kinematics.yaml",
            )
        )
        .joint_limits(
            file_path=os.path.join(
                get_package_share_directory("arm_moveit_config"),
                "config",
                "joint_limits.yaml",
            )
        )
        .trajectory_execution(
            file_path=os.path.join(
                get_package_share_directory("arm_moveit_config"),
                "config",
                "moveit_controllers.yaml",
            )
        )
        .planning_pipelines(pipelines=["ompl"])
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )

    # move_group node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # RViz
    rviz_config = PathJoinSubstitution([
        FindPackageShare("moveit_resources_panda_moveit_config"),
        "launch",
        "moveit.rviz",
    ])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    # Static TF: world -> panda_link0
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="log",
        arguments=["0", "0", "0", "0", "0", "0", "world", "panda_link0"],
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # ros2_control node (for mock_components mode)
    ros2_controllers_path = os.path.join(
        get_package_share_directory("arm_bringup"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="screen",
    )

    # Controller spawners
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["panda_arm_controller", "-c", "/controller_manager"],
    )
    hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["hand_trajectory_controller", "-c", "/controller_manager"],
    )

    return LaunchDescription([
        ros2_control_hardware_type,
        static_tf,
        robot_state_publisher,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        hand_controller_spawner,
        move_group_node,
        rviz_node,
    ])
