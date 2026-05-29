#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
experiment_timer_node — Coordina el inicio y fin del experimento multi-robot.

Expone dos servicios:
  experiment/start  (std_srvs/Trigger) — arranca el temporizador
  experiment/stop   (std_srvs/Trigger) — detiene el experimento antes de tiempo

Publica (TRANSIENT_LOCAL, para que nodos que se conecten tarde reciban el
estado actual de inmediato):
  /experiment/running  (std_msgs/Bool)

Parámetro:
  duration_s  float  Duración total del experimento en segundos (default 600.0)

Uso típico:
  ros2 launch qupa_experiment timer.launch.py
  ros2 service call /experiment/start std_srvs/srv/Trigger
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


class ExperimentTimerNode(Node):

    def __init__(self):
        super().__init__('experiment_timer_node')

        self.declare_parameter('duration_s', 600.0)
        self._duration_s = self.get_parameter('duration_s').value

        _latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._pub = self.create_publisher(Bool, '/experiment/running', _latched_qos)
        self._pub.publish(Bool(data=False))

        self._start_srv = self.create_service(
            Trigger, 'experiment/start', self._start_cb
        )
        self._stop_srv = self.create_service(
            Trigger, 'experiment/stop', self._stop_cb
        )

        self._countdown_timer = None

        self.get_logger().info(
            f'Experiment timer ready | duration={self._duration_s:.1f}s | '
            'call /experiment/start to begin'
        )

    def _start_cb(self, _request, response):
        if self._countdown_timer is not None:
            response.success = False
            response.message = 'Experiment already running'
            return response

        self._pub.publish(Bool(data=True))
        self._countdown_timer = self.create_timer(self._duration_s, self._on_expired)

        self.get_logger().info(
            f'Experiment STARTED — will run for {self._duration_s:.1f}s'
        )
        response.success = True
        response.message = f'Experiment started ({self._duration_s:.1f}s)'
        return response

    def _stop_cb(self, _request, response):
        self._stop()
        response.success = True
        response.message = 'Experiment stopped'
        return response

    def _on_expired(self):
        self.get_logger().info('Experiment timer EXPIRED.')
        self._stop()

    def _stop(self):
        if self._countdown_timer is not None:
            self._countdown_timer.cancel()
            self._countdown_timer = None
        self._pub.publish(Bool(data=False))
        self.get_logger().info('Experiment STOPPED')


def main(args=None):
    rclpy.init(args=args)
    node = ExperimentTimerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
