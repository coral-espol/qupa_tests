"""
experiment.launch.py — Launches the full QUPA experiment stack.

Starts the hardware drivers (IR scanner, floor sensor, motors, LEDs)
and the experiment behaviour node.  Camera is intentionally omitted —
floor color is provided by the TCS34725 sensor, not vision.

Usage:
  ros2 launch qupa_experiment experiment.launch.py
  ros2 launch qupa_experiment experiment.launch.py namespace:=qupa_3B
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

    # Config files
    ir_cfg    = os.path.join(hw_pkg,  'config', 'ir_scanner.yaml')
    motor_cfg = os.path.join(hw_pkg,  'config', 'motor.yaml')
    floor_cfg = os.path.join(hw_pkg,  'config', 'floor_sensor.yaml')
    leds_cfg  = os.path.join(hw_pkg,  'config', 'leds.yaml')
    exp_cfg   = os.path.join(exp_pkg, 'config', 'experiment.yaml')

    ns = LaunchConfiguration('namespace')

    # ── Hardware nodes (same staggered order as hardware.launch.py) ───────────

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

    floor_node = Node(
        package='qupa_hardware',
        executable='floor_sensor',
        name='floor_sensor_node',
        namespace=ns,
        output='screen',
        parameters=[floor_cfg],
    )

    led_node = Node(
        package='qupa_hardware',
        executable='led',
        name='led_node',
        namespace=ns,
        output='screen',
        parameters=[leds_cfg],
    )

    # ── Experiment brain ──────────────────────────────────────────────────────
    # Launched last (t = 12 s) so all sensors are ready before the loop starts.

    experiment_node = Node(
        package='qupa_experiment',
        executable='experiment',
        name='experiment_node',
        namespace=ns,
        output='screen',
        parameters=[
            exp_cfg,
            # Kinematic limits come from motor.yaml so both nodes stay in sync
            {'v_max_mps':        0.08},
            {'w_max_rps':        2.50},
            {'obstacle_stop_cm': 15.0},
            {'sensor_max_cm':    40.0},
        ],
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'namespace', default_value='qupa_3A',
            description='Robot namespace — change per robot (e.g. qupa_3B)',
        ),

        ir_node,                                              # t =  0 s
        TimerAction(period=3.0,  actions=[motor_node]),      # t =  3 s
        TimerAction(period=6.0,  actions=[floor_node]),      # t =  6 s
        TimerAction(period=9.0,  actions=[led_node]),        # t =  9 s
        TimerAction(period=12.0, actions=[experiment_node]), # t = 12 s

    ])
