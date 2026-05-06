#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
forward_stop_test — mueve el robot hacia adelante con evasión de obstáculos
por campo vectorial, igual que experiment_node pero sin lógica de tareas.

Subscribes:  scan     sensor_msgs/LaserScan
Publishes:   cmd_vel  geometry_msgs/Twist
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# Slot 0 = frente en este robot (board rotado -90° respecto al estándar).
# Ángulos en frame del robot: 0°=frente, +izquierda, -derecha.
#
#  slot  ángulo_robot  sensor
#    6     -90°  (derecha)         ✓
#    7     -45°  (frente-derecha)  ✓
#    0       0°  (frente)          ✓
#    1     +45°  (frente-izquierda)✓
#    2     +90°  (izquierda)       ✓
SENSOR_SLOTS: list[tuple[int, float]] = [
    (6, math.radians(-90.0)),
    (7, math.radians(-45.0)),
    (0, math.radians(  0.0)),
    (1, math.radians( 45.0)),
    (2, math.radians( 90.0)),
]


class ForwardStopTest(Node):

    def __init__(self):
        super().__init__('forward_stop_test')

        self.declare_parameter('forward_speed_mps', 0.04)
        self.declare_parameter('obstacle_stop_cm',  15.0)
        self.declare_parameter('sensor_max_cm',     40.0)
        self.declare_parameter('prox_threshold',     0.4)
        self.declare_parameter('prox_gain',          3.0)
        self.declare_parameter('torque_deadzone',    0.15)

        self._fwd_speed   = self.get_parameter('forward_speed_mps').value
        self._min_dist_cm = self.get_parameter('obstacle_stop_cm').value
        self._max_dist_cm = self.get_parameter('sensor_max_cm').value
        self._prox_thresh = self.get_parameter('prox_threshold').value
        self._prox_gain   = self.get_parameter('prox_gain').value
        self._torque_dz   = self.get_parameter('torque_deadzone').value

        self._ranges: list[float] = [float('inf')] * 8

        self._pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(LaserScan, 'scan', self._scan_cb, 10)
        self.create_timer(0.2, self._step)   # 5 Hz

        self.get_logger().info(
            f'forward_stop_test listo | '
            f'v={self._fwd_speed} m/s | '
            f'stop={self._min_dist_cm:.0f} cm | '
            f'umbral_prox={self._prox_thresh}'
        )

    def _scan_cb(self, msg: LaserScan):
        self._ranges = list(msg.ranges)

    def _normalize(self, dist_m: float) -> float:
        """Distancia en metros → proximidad [0..1]."""
        min_m = self._min_dist_cm / 100.0
        max_m = self._max_dist_cm / 100.0
        if not math.isfinite(dist_m) or dist_m >= max_m:
            return 0.0
        if dist_m <= min_m:
            return 1.0
        return 1.0 - (dist_m - min_m) / (max_m - min_m)

    def _get_cmd(self) -> tuple[float, float]:
        max_prox = 0.0
        torque   = 0.0

        for slot, angle in SENSOR_SLOTS:
            dist_m = self._ranges[slot] if slot < len(self._ranges) else float('inf')
            v = self._normalize(dist_m)
            if v > max_prox:
                max_prox = v
            torque += v * math.sin(angle)

        if max_prox <= self._prox_thresh:
            return self._fwd_speed, 0.0

        # Obstáculo detectado — frena y gira
        linear_x = self._fwd_speed * 0.2

        if abs(torque) < self._torque_dz:
            # Obstáculo directo al frente sin lado claro — sesgo fijo izquierda
            angular_z = 0.4
        else:
            angular_z = max(min(-self._prox_gain * torque, 2.5), -2.5)

        self.get_logger().info(
            f'EVADIENDO — max_prox={max_prox:.2f} torque={torque:.2f} w={angular_z:.2f}',
            throttle_duration_sec=0.5
        )
        return linear_x, angular_z

    def _step(self):
        v, w = self._get_cmd()
        cmd = Twist()
        cmd.linear.x  = round(v, 3)
        cmd.angular.z = round(w, 3)
        self._pub.publish(cmd)

    def destroy_node(self):
        self._pub.publish(Twist())
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ForwardStopTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
