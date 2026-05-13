"""
experiment_multi.launch.py — Lanza el experimento para varios robots a la vez.

Cada namespace en `namespaces` lanza una instancia del mismo nodo
`experiment_node`. Asume que `hardware.launch.py` ya está corriendo en cada
robot físico (los nodos se suscriben a `scan`, `floor/color`, etc.,
relativos al namespace).

Uso:
  # 2 robots (default)
  ros2 launch qupa_experiment experiment_multi.launch.py

  # Lista explícita separada por coma
  ros2 launch qupa_experiment experiment_multi.launch.py \\
      namespaces:=qupa_3A,qupa_3B,qupa_F4
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _spawn_nodes(context, *args, **kwargs):
    raw     = LaunchConfiguration('namespaces').perform(context)
    ns_list = [n.strip() for n in raw.split(',') if n.strip()]

    if not ns_list:
        raise RuntimeError(
            "Lista de 'namespaces' vacía. Ejemplo: namespaces:=qupa_3A,qupa_3B"
        )

    exp_pkg = get_package_share_directory('qupa_experiment')
    exp_cfg = os.path.join(exp_pkg, 'config', 'experiment.yaml')

    nodes = []
    for ns in ns_list:
        nodes.append(Node(
            package='qupa_experiment',
            executable='experiment',
            name='experiment_node',
            namespace=ns,
            output='screen',
            parameters=[
                exp_cfg,
                {'v_max_mps':        0.10},
                {'w_max_rps':        1.50},
                {'obstacle_stop_cm': 15.0},
                {'sensor_max_cm':    20.0},
            ],
        ))
    return nodes


def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument(
            'namespaces',
            default_value='qupa_3A,qupa_3B',
            description='Lista de namespaces separados por coma (e.g. qupa_3A,qupa_3B)',
        ),

        OpaqueFunction(function=_spawn_nodes),

    ])
