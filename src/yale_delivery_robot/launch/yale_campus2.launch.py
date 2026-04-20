#!/usr/bin/env python3
"""
Yale Campus Launch
- Stock Turtlebot3 Waffle (with built-in depth camera)
- Mirrors the resource path setup from turtlebot3_world.launch.py so meshes load
- Teleop works with TwistStamped (Jazzy default)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_dir = get_package_share_directory('yale_delivery_robot')
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')
    tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')

    # This is the key — same path the stock turtlebot3 launch sets
    # so Gazebo can find waffle_base.stl, lds.stl, r200.dae etc.
    tb3_models_path = os.path.join(tb3_gazebo_dir, 'models')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='74.382000')
    y_pose = LaunchConfiguration('y_pose', default='5.192810')
    z_pose = LaunchConfiguration('z_pose', default='0.3')

    world_file = os.path.join(pkg_dir, 'worlds', 'yale_campus.world')
    gz_args = LaunchConfiguration('gz_args', default=f'-r {world_file}')

    # ===== ENV VARS =====
    set_tb3_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle')

    # Add TB3 models dir to GZ resource path — this is what makes meshes load
    existing_gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    set_gz_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        tb3_models_path + (':' + existing_gz_path if existing_gz_path else '')
    )

    # ===== GAZEBO HARMONIC =====
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': gz_args}.items(),
    )

    # ===== STOCK TURTLEBOT3 ROBOT STATE PUBLISHER =====
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_dir, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # ===== SPAWN STOCK TURTLEBOT3 WAFFLE =====
    spawn_turtlebot3 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'waffle',
            '-topic', 'robot_description',
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
        ],
        output='screen'
    )

    # ===== ROS-GZ BRIDGE =====
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/cmd_vel@geometry_msgs/msg/TwistStamped]gz.msgs.Twist',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/depth/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('x_pose', default_value='74.382000'),
        DeclareLaunchArgument('y_pose', default_value='5.192810'),
        DeclareLaunchArgument('z_pose', default_value='0.3'),
        DeclareLaunchArgument('gz_args', default_value=f'-r {world_file}'),
        set_tb3_model,
        set_gz_resource_path,
        gazebo,
        robot_state_publisher,
        spawn_turtlebot3,
        bridge,
    ])