"""
experiment_classic.launch.py — Lanza el nodo CLÁSICO sin cámara/social learning.

Versión diagnóstica para descartar que el camera_node sea la causa de las
caídas de SSH durante el experimento. Asume que hardware.launch.py ya está
corriendo (ir_scanner, motor_driver, floor_sensor, led_node). NO requiere
que el camera_node esté lanzado — este nodo no se suscribe a la cámara.

Uso:
  ros2 launch qupa_experiment experiment_classic.launch.py
  ros2 launch qupa_experiment experiment_classic.launch.py namespace:=qupa_3B
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    exp_pkg = get_package_share_directory('qupa_experiment')
    exp_cfg = os.path.join(exp_pkg, 'config', 'experiment.yaml')

    ns = LaunchConfiguration('namespace')

    experiment_node = Node(
        package='qupa_experiment',
        executable='experiment_classic',
        name='experiment_node_classic',
        namespace=ns,
        output='screen',
        parameters=[
            exp_cfg,
            {'v_max_mps':        0.08},
            {'w_max_rps':        2.50},
            {'obstacle_stop_cm': 15.0},
            {'sensor_max_cm':    40.0},
        ],
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'namespace', default_value='qupa_AE',
            description='Namespace del robot',
        ),

        experiment_node,

    ])
