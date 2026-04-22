# -*- coding: utf-8 -*-
"""Launch the perception node plus a synthetic sensor feed and RViz.

Usage:
    ros2 launch autonomy_pipeline autonomy_pipeline.launch.py
    ros2 launch autonomy_pipeline autonomy_pipeline.launch.py use_rviz:=false
    ros2 launch autonomy_pipeline autonomy_pipeline.launch.py use_fake:=false

Set `use_fake:=false` when feeding real sensors or a rosbag. Set
`use_rviz:=false` for headless runs (e.g. on a Jetson over SSH).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('autonomy_pipeline')
    default_params = os.path.join(pkg_share, 'config', 'params.yaml')
    default_rviz = os.path.join(pkg_share, 'rviz', 'autonomy.rviz')

    params_arg = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='Path to the parameter YAML.')
    rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Whether to start RViz2.')
    fake_arg = DeclareLaunchArgument(
        'use_fake', default_value='true',
        description='Whether to start the synthetic sensor publisher.')
    log_arg = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='rclpy log level: debug|info|warn|error')

    params_file = LaunchConfiguration('params_file')
    log_level = LaunchConfiguration('log_level')

    fake = Node(
        package='autonomy_pipeline',
        executable='fake_sensor_publisher',
        name='fake_sensor_publisher',
        parameters=[params_file],
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        condition=IfCondition(LaunchConfiguration('use_fake')),
    )

    grid = Node(
        package='autonomy_pipeline',
        executable='obstacle_grid_node',
        name='obstacle_grid_node',
        parameters=[params_file],
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        params_arg, rviz_arg, fake_arg, log_arg,
        fake, grid, rviz,
    ])
