"""
forward_stop_test.launch.py — Lanza solo el nodo de prueba.

Asume que hardware.launch.py ya está corriendo (IR scanner + motor driver).

Uso:
  ros2 launch qupa_experiment forward_stop_test.launch.py
  ros2 launch qupa_experiment forward_stop_test.launch.py namespace:=qupa_3B
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    ns = LaunchConfiguration('namespace')

    test_node = Node(
        package='qupa_experiment',
        executable='forward_stop_test',
        name='forward_stop_test',
        namespace=ns,
        output='screen',
        parameters=[{
            'forward_speed_mps': 0.04,
            'stop_distance_m':   0.20,
        }],
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'namespace', default_value='qupa_AE',
            description='Namespace del robot',
        ),

        test_node,

    ])
