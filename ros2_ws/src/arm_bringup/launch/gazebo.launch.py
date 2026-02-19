import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command


def generate_launch_description():
    pkg_arm_bringup = FindPackageShare('arm_bringup')
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim')

    # World file
    world_file = PathJoinSubstitution([
        pkg_arm_bringup, 'worlds', 'pick_place.sdf'
    ])

    # Start Gazebo with our world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={'gz_args': ['-r ', world_file]}.items(),
    )

    # Process xacro to URDF
    urdf_path = PathJoinSubstitution([
        pkg_arm_bringup, 'urdf', 'panda.urdf.xacro'
    ])

    robot_description = Command([
        'xacro ', urdf_path,
        ' ros2_control_hardware_type:=gz_ros2_control'
    ])

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )

    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'panda',
            '-z', '0.0',  # spawn on top of table height
        ],
        output='screen',
    )

    # Spawn controllers
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['panda_arm_controller'],
        output='screen',
    )

    hand_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['panda_hand_controller'],
        output='screen',
    )

    # Bridge Gazebo camera topics to ROS2
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/rgbd_camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/rgbd_camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
        ],
        output='screen',
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        joint_state_broadcaster,
        arm_controller,
        hand_controller,
        gz_bridge,
    ])

