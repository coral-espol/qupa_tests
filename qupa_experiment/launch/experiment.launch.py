"""
experiment.launch.py — Lanza solo el nodo de comportamiento del experimento.

Asume que hardware.launch.py ya está corriendo:
  ir_scanner, motor_driver, floor_sensor, led_node.

Uso:
  ros2 launch qupa_experiment experiment.launch.py
  ros2 launch qupa_experiment experiment.launch.py namespace:=qupa_3B
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    exp_pkg = get_package_share_directory('qupa_experiment')
    exp_cfg = os.path.join(exp_pkg, 'config', 'experiment.yaml')

    # Load YAML as plain dict to bypass ROS 2's namespace-aware param matching
    # (passing the file path silently drops params when a namespace is applied).
    with open(exp_cfg, 'r') as f:
        cfg = yaml.safe_load(f)
    shared_params = cfg.get('experiment_node', {}).get('ros__parameters', {})

    ns = LaunchConfiguration('namespace')

    experiment_node = Node(
        package='qupa_experiment',
        executable='experiment',
        name='experiment_node',
        namespace=ns,
        output='screen',
        parameters=[
            shared_params,
            {'v_max_mps':        0.10},
            {'w_max_rps':        2.50},
            {'obstacle_stop_cm': 15.0},
            {'sensor_max_cm':    20.0},
        ],
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'namespace', default_value='qupa_C0',
            description='Namespace del robot',
        ),

        experiment_node,

    ])
