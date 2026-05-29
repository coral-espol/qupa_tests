"""
timer.launch.py — Lanza el nodo coordinador del experimento.

Publica /experiment/running (Bool, TRANSIENT_LOCAL) para que todos los
robots sepan cuándo el experimento está activo y deben registrar datos.

Uso:
  ros2 launch qupa_experiment timer.launch.py
  ros2 launch qupa_experiment timer.launch.py duration_s:=300.0

Para iniciar el experimento una vez que los robots estén corriendo:
  ros2 service call /experiment/start std_srvs/srv/Trigger

Para detener antes de tiempo:
  ros2 service call /experiment/stop std_srvs/srv/Trigger
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    exp_pkg   = get_package_share_directory('qupa_experiment')
    timer_cfg = os.path.join(exp_pkg, 'config', 'timer.yaml')

    with open(timer_cfg, 'r') as f:
        cfg = yaml.safe_load(f)
    timer_params = cfg.get('experiment_timer_node', {}).get('ros__parameters', {})

    duration = LaunchConfiguration('duration_s')

    timer_node = Node(
        package='qupa_experiment',
        executable='experiment_timer',
        name='experiment_timer_node',
        output='screen',
        parameters=[
            timer_params,
            {'duration_s': duration},
        ],
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'duration_s',
            default_value=str(timer_params.get('duration_s', 600.0)),
            description='Duración total del experimento en segundos',
        ),

        timer_node,

    ])
