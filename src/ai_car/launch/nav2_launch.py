#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node

pkg_ai_car       = get_package_share_directory('ai_car')
pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

def generate_launch_description():
    urdf_path = os.path.join(
        os.path.expanduser('~'),
        'ros2_class_ws', 'src', 'ai_car', 'urdf', 'ai_car.urdf'
    )
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': False,
            }]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_nav2_bringup, 'launch', 'localization_launch.py'])),
            launch_arguments={
                'map': os.path.join(os.path.expanduser('~'), 'map.yaml'),
                'params_file': os.path.join(pkg_ai_car, 'config', 'localization.yaml'),
                'use_sim_time': 'False',
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_nav2_bringup, 'launch', 'navigation_launch.py'])),
            launch_arguments={
                'params_file': os.path.join(pkg_ai_car, 'config', 'nav2.yaml'),
                'use_sim_time': 'False',
                'use_composition': 'False',
            }.items()
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_ai_car, 'rviz', 'nav2.rviz')]
        ),
    ])
