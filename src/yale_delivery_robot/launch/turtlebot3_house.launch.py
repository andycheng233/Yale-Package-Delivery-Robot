#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    yale_pkg_share = get_package_share_directory('yale_delivery_robot')
    yale_launch_dir = os.path.join(yale_pkg_share, 'launch')

    tb3_pkg_share = get_package_share_directory('turtlebot3_gazebo')
    tb3_launch_dir = os.path.join(tb3_pkg_share, 'launch')

    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-2.0')
    y_pose = LaunchConfiguration('y_pose', default='0.7')

    world = os.path.join(
        yale_pkg_share,
        'worlds',
        'turtlebot3_house.world'
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -s -v2 ', world],
            'on_exit_shutdown': 'true'
        }.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': '-g -v2 ',
            'on_exit_shutdown': 'true'
        }.items()
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(yale_launch_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(yale_launch_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    # Put your package models FIRST so Gazebo finds your copied turtlebot3_waffle
    set_env_vars_resources_user = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(yale_pkg_share, 'models')
    )

    # Keep stock TurtleBot3 models available as fallback
    set_env_vars_resources_tb3 = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(tb3_pkg_share, 'models')
    )

    ld = LaunchDescription()
    ld.add_action(set_env_vars_resources_user)
    ld.add_action(set_env_vars_resources_tb3)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)

    return ld