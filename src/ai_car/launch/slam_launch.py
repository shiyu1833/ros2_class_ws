#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('ai_car')

    urdf_path = os.path.join(
        os.path.expanduser('~'),
        'ros2_class_ws', 'src', 'ai_car', 'urdf', 'ai_car.urdf'
    )
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    slam_configure = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'configure'],
        output='screen'
    )

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
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[os.path.join(pkg, 'config', 'slam.yaml')],
        ),
        TimerAction(
            period=3.0,
            actions=[slam_configure]
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=slam_configure,
                on_exit=[ExecuteProcess(
                    cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'activate'],
                    output='screen'
                )]
            )
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg, 'rviz', 'slam.rviz')]
        ),
    ])