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
import yaml

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

    exp_pkg  = get_package_share_directory('qupa_experiment')
    exp_cfg  = os.path.join(exp_pkg, 'config', 'experiment.yaml')
    log_dir  = LaunchConfiguration('data_log_dir').perform(context)

    # Load YAML as plain dict. Passing the file path directly relies on
    # ROS 2 matching `experiment_node:` against the node FQN, which fails
    # silently under a namespace — that left cluster_threshold_px and
    # other params at their declared defaults.
    with open(exp_cfg, 'r') as f:
        cfg = yaml.safe_load(f)
    shared_params = cfg.get('experiment_node', {}).get('ros__parameters', {})

    nodes = []
    for ns in ns_list:
        log_path = os.path.join(log_dir, f'{ns}.csv') if log_dir else ''
        nodes.append(Node(
            package='qupa_experiment',
            executable='experiment',
            name='experiment_node',
            namespace=ns,
            output='screen',
            parameters=[
                shared_params,
                {'v_max_mps':        0.0},
                {'w_max_rps':        0.0},
                {'obstacle_stop_cm': 15.0},
                {'sensor_max_cm':    20.0},
                {'data_log_path':    log_path},
            ],
        ))
    return nodes


def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument(
            'namespaces',
            default_value='qupa_7E,qupa_27',
            description='Lista de namespaces separados por coma (e.g. qupa_3A,qupa_2C)',
        ),

        DeclareLaunchArgument(
            'data_log_dir',
            default_value=os.path.join(os.path.expanduser('~'), 'qupa_ws', 'exp', 'data'),
            description='Directorio de salida para los CSV por robot. '
                        'Vacío = sin logging.',
        ),

        OpaqueFunction(function=_spawn_nodes),

    ])
