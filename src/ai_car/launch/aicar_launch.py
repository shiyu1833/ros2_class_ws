#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ai_car',
            executable='aicar_base',
        ),
        Node(
            package='ai_car',
            executable='aicar_imu',
        ),
        Node(
            package='ai_car',
            executable='aicar_odom',
        ),
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar',
            output='screen',
            parameters=[{
                'serial_port': '/dev/RPLIDAR',
                'serial_baudrate': 115200,
                'frame_id': 'base_scan',
                'angle_compensate': True,
                'scan_mode': 'Standard',
            }]
        ),
        Node(
            package='camera_ros',
            executable='camera_node',
            name='camera',
            output='screen',
            parameters=[{
                'width': 320,
                'height': 240,
            }]
        ),
    ])