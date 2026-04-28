# Adapted from turtlebot3_navigation2 by Darby Lim (Apache 2.0)
# Changes for Yale Delivery Robot:
#   - Points to yale_delivery_robot package instead of turtlebot3_navigation2
#   - Uses our nav2_params.yaml (copied from apt install + edited)
#   - use_sim_time defaults to true (always running in Gazebo)
#   - Added twist_mux for pedestrian safety override
#   - Uses turtlebot3 RViz config (still works fine for our robot)
#   - Removed ROS_DISTRO branching (we are always on Jazzy)
#   - Removed TURTLEBOT3_MODEL dependency (not needed for params path)

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── Package directories ───────────────────────────────────────────────────
    pkg_dir      = get_package_share_directory('yale_delivery_robot')
    nav2_bringup = get_package_share_directory('nav2_bringup')
    tb3_nav_dir  = get_package_share_directory('turtlebot3_navigation2')

    # ── Launch arguments ─────────────────────────────────────────────────────
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Default map points to our maps folder
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(pkg_dir, 'maps', 'map.yaml')
    )

    # Default params point to our edited copy of the nav2 params
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    )

    # RViz config — reuse turtlebot3's config, works fine for waffle
    rviz_config = os.path.join(
        tb3_nav_dir, 'rviz', 'tb3_navigation2.rviz'
    )

    # ── Nav2 bringup ─────────────────────────────────────────────────────────
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map':         map_dir,
            'use_sim_time': use_sim_time,
            'params_file': param_dir,
        }.items(),
    )

    # ── Twist mux ─────────────────────────────────────────────────────────────
    # Sits between Nav2 and the robot.
    # Nav2 publishes to /cmd_vel_nav (priority 10)
    # Pedestrian safety node publishes to /cmd_vel_safety (priority 100)
    # twist_mux outputs the winner to /cmd_vel
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[os.path.join(pkg_dir, 'config', 'twist_mux.yaml')],
        remappings=[('/cmd_vel_out', '/cmd_vel')]
    )

    # ── RViz ─────────────────────────────────────────────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(pkg_dir, 'maps', 'map.yaml'),
            description='Full path to map yaml file'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_dir, 'config', 'nav2_params.yaml'),
            description='Full path to nav2 params file'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation clock'
        ),
        nav2,
        twist_mux,
        rviz,
    ])