#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
led_test — Publish-based LED tester.

Standalone diagnostic node that cycles the robot LEDs through several
colours by publishing to leds/command (std_msgs/String with JSON payload).
Useful to confirm LED control works without launching the full experiment.

Topic:
  leds/command   std_msgs/String   {"mode": "set_all", "rgb": [r, g, b]}

Run:
  ros2 run qupa_experiment led_test
  ros2 run qupa_experiment led_test --ros-args -r __ns:=/qupa_AE
"""

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


COLORS = [
    ('RED',    (255,   0,   0)),
    ('GREEN',  (  0, 255,   0)),
    ('BLUE',   (  0,   0, 255)),
    ('AMBER',  (255, 165,   0)),
    ('WHITE',  (255, 255, 255)),
    ('OFF',    (  0,   0,   0)),
]


class LedTestNode(Node):

    def __init__(self):
        super().__init__('led_test')

        self.declare_parameter('topic',      'leds/command')
        self.declare_parameter('step_s',     1.0)
        self.declare_parameter('loop',       False)

        topic         = self.get_parameter('topic').value
        self._step_s  = self.get_parameter('step_s').value
        self._loop    = self.get_parameter('loop').value

        self._pub = self.create_publisher(String, topic, 10)
        self.get_logger().info(
            f'Publishing LED commands to "{topic}" '
            f'(step={self._step_s}s, loop={self._loop})'
        )

    def _publish(self, r: int, g: int, b: int):
        msg      = String()
        msg.data = json.dumps({'mode': 'set_all', 'rgb': [r, g, b]})
        self._pub.publish(msg)

    def run(self):
        # Give the publisher a moment to discover the subscriber before
        # firing the first message (otherwise it may be dropped).
        time.sleep(0.5)

        while rclpy.ok():
            for name, (r, g, b) in COLORS:
                self.get_logger().info(f'→ {name:6s} rgb=({r:3d}, {g:3d}, {b:3d})')
                self._publish(r, g, b)
                # Spin a bit so callbacks (none here, but good practice) can fire
                end = time.time() + self._step_s
                while time.time() < end and rclpy.ok():
                    rclpy.spin_once(self, timeout_sec=0.05)

            if not self._loop:
                break


def main(args=None):
    rclpy.init(args=args)
    node = LedTestNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        # Leave the LEDs off when exiting.
        try:
            node._publish(0, 0, 0)
            time.sleep(0.1)
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
