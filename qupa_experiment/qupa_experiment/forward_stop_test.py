#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
forward_stop_test — mueve el robot hacia adelante y para si el sensor
frontal (ir_n) detecta un obstáculo más cerca que stop_distance_m.

Subscribes:  ir_n   sensor_msgs/LaserScan
Publishes:   cmd_vel  geometry_msgs/Twist
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class ForwardStopTest(Node):

    def __init__(self):
        super().__init__('forward_stop_test')

        self.declare_parameter('forward_speed_mps', 0.04)   # m/s  (~50% v_max)
        self.declare_parameter('stop_distance_m',   0.20)   # m    (20 cm)

        self._speed    = self.get_parameter('forward_speed_mps').value
        self._stop_m   = self.get_parameter('stop_distance_m').value

        # None = todavía no recibimos ninguna lectura del sensor
        self._front_dist_m: float | None = None

        self._pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(LaserScan, 'ir_n', self._ir_cb, 10)

        # Publica a 10 Hz
        self.create_timer(0.1, self._step)

        self.get_logger().info(
            f'forward_stop_test listo | '
            f'velocidad={self._speed} m/s | '
            f'parar a ≤{self._stop_m*100:.0f} cm'
        )

    def _ir_cb(self, msg: LaserScan):
        if msg.ranges:
            d = msg.ranges[0]
            self._front_dist_m = d if math.isfinite(d) else None

    def _step(self):
        cmd = Twist()

        if self._front_dist_m is None:
            # Sin lectura todavía — espera quieto
            self.get_logger().info('Esperando lectura del sensor ir_n...', once=True)
        elif self._front_dist_m <= self._stop_m:
            # Obstáculo cercano — para
            self.get_logger().info(
                f'STOP — obstáculo a {self._front_dist_m*100:.1f} cm', throttle_duration_sec=1.0
            )
        else:
            # Camino libre — avanza
            cmd.linear.x = self._speed
            self.get_logger().info(
                f'ADELANTE — frontal a {self._front_dist_m*100:.1f} cm', throttle_duration_sec=1.0
            )

        self._pub.publish(cmd)

    def destroy_node(self):
        # Para los motores al salir
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
