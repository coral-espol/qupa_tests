"""
forward_stop_test.launch.py — Hardware mínimo + nodo de prueba.

Lanza solo los nodos necesarios para la prueba:
  IR scanner  (sensores de proximidad)
  Motor driver (cmd_vel → PWM)
  Nodo de prueba (avanza y para con sensor frontal)

Uso:
  ros2 launch qupa_experiment forward_stop_test.launch.py
  ros2 launch qupa_experiment forward_stop_test.launch.py namespace:=qupa_3B
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    hw_pkg  = get_package_share_directory('qupa_hardware')
    exp_pkg = get_package_share_directory('qupa_experiment')

    ir_cfg    = os.path.join(hw_pkg,  'config', 'ir_scanner.yaml')
    motor_cfg = os.path.join(hw_pkg,  'config', 'motor.yaml')

    ns = LaunchConfiguration('namespace')

    ir_node = Node(
        package='qupa_hardware',
        executable='ir_scanner',
        name='ir_scanner',
        namespace=ns,
        output='screen',
        parameters=[ir_cfg],
    )

    motor_node = Node(
        package='qupa_hardware',
        executable='motor_driver',
        name='motor_node',
        namespace=ns,
        output='screen',
        parameters=[motor_cfg],
    )

    test_node = Node(
        package='qupa_experiment',
        executable='forward_stop_test',
        name='forward_stop_test',
        namespace=ns,
        output='screen',
        parameters=[{
            'forward_speed_mps': 0.04,   # ajustar si va muy rápido/lento
            'stop_distance_m':   0.20,   # para a 20 cm del obstáculo
        }],
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'namespace', default_value='qupa_3A',
            description='Namespace del robot',
        ),

        ir_node,                                           # t = 0 s
        TimerAction(period=3.0, actions=[motor_node]),    # t = 3 s
        TimerAction(period=6.0, actions=[test_node]),     # t = 6 s

    ])
