"""
color_behavior.launch.py — Lanza el comportamiento reactivo por color en
uno o varios robots.

Cada namespace de `namespaces` obtiene una instancia de `color_behavior_node`
(suscrita a camera/detections y scan, publica cmd_vel, cliente del servicio
de LEDs `set`), todo relativo a su namespace.

Comportamiento (ver color_behavior_node.py):
  · Random walk / exploración por defecto.
  · Detecta VERDE  → frena de inmediato mientras lo vea.
  · Detecta AZUL   → se agrega hacia el color; a 15 cm frena y enciende LED azul.

Uso (con la simulación de 4 robots ya corriendo, high_res:=true):
  ros2 launch qupa_experiment color_behavior.launch.py

  # Lista explícita:
  ros2 launch qupa_experiment color_behavior.launch.py namespaces:=qupa,qupa_2

Requiere que la simulación corra con cámara fisheye para detectar color:
  ros2 launch qupa_simulation simulation.launch.py high_res:=true
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
            "Lista de 'namespaces' vacía. Ej: namespaces:=qupa,qupa_2"
        )

    exp_pkg = get_package_share_directory('qupa_experiment')
    cfg     = os.path.join(exp_pkg, 'config', 'color_behavior.yaml')

    nodes = []
    for ns in ns_list:
        nodes.append(Node(
            package='qupa_experiment',
            executable='color_behavior',
            name='color_behavior_node',
            namespace=ns,
            output='screen',
            parameters=[cfg, {'use_sim_time': True}],
        ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespaces',
            default_value='qupa_1,qupa_2,qupa_3,qupa_4,qupa_5,qupa_6,qupa_7,qupa_8',
            description='Namespaces separados por coma (default: los 4 robots '
                        'de simulation.launch.py).',
        ),
        OpaqueFunction(function=_spawn_nodes),
    ])
