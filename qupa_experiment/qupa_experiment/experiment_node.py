#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
experiment_node — ROS2 port of the MQTT-based QupaExperiment brain.

Implements the Brutschy et al. (2012) self-organised task-allocation model
with vector-field obstacle avoidance using the robot's IR proximity sensors.

Subscribes:
  ir_n   / ir_nw / ir_ne / ir_w / ir_e   sensor_msgs/LaserScan  IR distances
  floor/color                              std_msgs/String        JSON color label

Publishes:
  cmd_vel       geometry_msgs/Twist   velocity commands
  leds/command  std_msgs/String       JSON LED commands
"""

import json
import math
import random

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


# ── Scan slot → robot-frame angle ────────────────────────────────────────────
# ir_scanner_node publica un único LaserScan ('scan') con 8 slots de 45°.
# El frame del scan tiene 0° apuntando al Este; 90° = Norte = frente del robot.
# Ángulo en frame del robot: positivo = izquierda (convención ROS).
#
#  slot  ángulo_scan  dirección   ángulo_robot
#    0       0°          E          -90°  (derecha)
#    1      45°          NE         -45°  (frente-derecha)
#    2      90°          N            0°  (frente)
#    3     135°          NW         +45°  (frente-izquierda, sin sensor → inf)
#    4     180°          W          +90°  (izquierda)
#
# Slots 5, 6, 7 = sur/traseros — no útiles para navegación hacia adelante.
SENSOR_SLOTS: list[tuple[int, float]] = [
    (0, math.radians(-90.0)),   # E  — derecha
    (1, math.radians(-45.0)),   # NE — frente-derecha
    (2, math.radians(0.0)),     # N  — frente
    (3, math.radians(45.0)),    # NW — frente-izquierda (siempre inf)
    (4, math.radians(90.0)),    # W  — izquierda
]


class States:
    EXPLORE = 'EXPLORE'
    EXECUTE = 'EXECUTE'
    EXIT    = 'EXIT_PATCH'


class QupaExperimentNode(Node):

    def __init__(self):
        super().__init__('experiment_node')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('loop_rate_hz',     10.0)
        self.declare_parameter('refractory_ticks', 20)
        self.declare_parameter('fwd_speed_ratio',  0.6)
        self.declare_parameter('prox_threshold',   0.4)
        self.declare_parameter('prox_gain',        3.0)
        self.declare_parameter('torque_deadzone',  0.15)

        self.declare_parameter('type_a_colors', ['MAGENTA'])
        self.declare_parameter('type_b_colors', ['YELLOW'])

        # Flat parameters for nested YAML keys (ROS2 param flattening)
        self.declare_parameter('task_timing.base_work_ticks', 100)
        self.declare_parameter('task_timing.min_work_ticks',   30)
        self.declare_parameter('task_timing.learning_step',    10)

        self.declare_parameter('specialization.m_max',  5)
        self.declare_parameter('specialization.gamma',  1.0)

        self.declare_parameter('forgetting.forget_interval_s', 30.0)

        self.declare_parameter('patrol.period_s', 4.0)
        self.declare_parameter('patrol.on_s',     0.5)

        # Motor kinematics — read from motor_node params via remapped topic,
        # or fall back to safe defaults matching motor.yaml.
        self.declare_parameter('v_max_mps',        0.08)
        self.declare_parameter('w_max_rps',        2.50)
        self.declare_parameter('obstacle_stop_cm', 15.0)
        self.declare_parameter('sensor_max_cm',    40.0)

        # ── Cache parameter values ────────────────────────────────────────────
        self._loop_hz       = self.get_parameter('loop_rate_hz').value
        self._loop_period   = 1.0 / self._loop_hz
        self._refract_ticks = self.get_parameter('refractory_ticks').value

        v_max               = self.get_parameter('v_max_mps').value
        self._fwd_speed     = v_max * self.get_parameter('fwd_speed_ratio').value
        self._w_max         = self.get_parameter('w_max_rps').value
        self._prox_thresh   = self.get_parameter('prox_threshold').value
        self._prox_gain     = self.get_parameter('prox_gain').value
        self._torque_dz     = self.get_parameter('torque_deadzone').value
        self._min_dist_cm   = self.get_parameter('obstacle_stop_cm').value
        self._max_dist_cm   = self.get_parameter('sensor_max_cm').value

        self._type_a_colors = list(self.get_parameter('type_a_colors').value)
        self._type_b_colors = list(self.get_parameter('type_b_colors').value)

        self._base_work   = self.get_parameter('task_timing.base_work_ticks').value
        self._min_work    = self.get_parameter('task_timing.min_work_ticks').value
        self._learn_step  = self.get_parameter('task_timing.learning_step').value

        self._m_max       = self.get_parameter('specialization.m_max').value
        self._gamma       = self.get_parameter('specialization.gamma').value

        self._forget_s    = self.get_parameter('forgetting.forget_interval_s').value

        patrol_period_s   = self.get_parameter('patrol.period_s').value
        patrol_on_s       = self.get_parameter('patrol.on_s').value
        self._patrol_blink_ticks = int(patrol_period_s * self._loop_hz)
        self._patrol_on_ticks    = int(patrol_on_s     * self._loop_hz)

        # ── Sensor state ──────────────────────────────────────────────────────
        # Distancias en metros por slot; inf = sin obstáculo / sin sensor
        self._ranges: list[float] = [float('inf')] * 8
        self._last_floor: dict = {}

        # ── Behaviour state ───────────────────────────────────────────────────
        self._state       = States.EXPLORE
        self._state_timer = 0

        self._ignore_ticks     = 0
        self._decision_made    = False
        self._last_seen_color  = 'NONE'

        # Specialisation counter — m > 0 → TYPE_A tendency, m < 0 → TYPE_B
        self._m                 = 0
        self._current_task_type = None
        self._current_job_time  = self._base_work
        self._forget_timer_s    = 0.0

        # LED helpers
        self._last_led_cmd     = None
        self._patrol_timer     = 0
        self._reject_led_timer = 0

        # ── Publishers ────────────────────────────────────────────────────────
        self._pub_cmd = self.create_publisher(Twist,  'cmd_vel',       10)
        self._pub_led = self.create_publisher(String, 'leds/command',  10)

        # ── Subscriptions ─────────────────────────────────────────────────────
        # Topic único publicado por ir_scanner_node
        self.create_subscription(LaserScan, 'scan', self._scan_cb, 10)
        self.create_subscription(String, 'floor/color', self._floor_cb, 10)

        # ── Main loop ─────────────────────────────────────────────────────────
        self._timer = self.create_timer(self._loop_period, self._step)

        self._set_leds(255, 165, 0)   # Orange on boot
        self.get_logger().info(
            f'Experiment node ready @ {self._loop_hz:.0f} Hz | '
            f'TYPE_A={self._type_a_colors} TYPE_B={self._type_b_colors}'
        )

    # =========================================================
    # Callbacks
    # =========================================================

    def _scan_cb(self, msg: LaserScan):
        self._ranges = list(msg.ranges)

    def _floor_cb(self, msg: String):
        try:
            self._last_floor = json.loads(msg.data)
        except Exception:
            pass

    # =========================================================
    # Navigation — vector-field obstacle avoidance
    # =========================================================

    def _normalize(self, dist_m: float) -> float:
        """Map distancia en metros a proximidad [0..1]."""
        min_m = self._min_dist_cm / 100.0
        max_m = self._max_dist_cm / 100.0
        if not math.isfinite(dist_m) or dist_m >= max_m:
            return 0.0
        if dist_m <= min_m:
            return 1.0
        return 1.0 - (dist_m - min_m) / (max_m - min_m)

    def _get_vector_move_cmd(self) -> tuple[float, float, bool]:
        """Return (linear_x, angular_z, is_avoiding)."""
        max_prox = 0.0
        torque   = 0.0

        for slot, angle in SENSOR_SLOTS:
            dist_m = self._ranges[slot] if slot < len(self._ranges) else float('inf')
            v = self._normalize(dist_m)
            if v > max_prox:
                max_prox = v
            torque += v * math.sin(angle)

        linear_x    = self._fwd_speed
        angular_z   = 0.0
        is_avoiding = False

        if max_prox > self._prox_thresh:
            is_avoiding = True
            linear_x    = self._fwd_speed * 0.2

            if abs(torque) < self._torque_dz:
                # Obstacle directly ahead — fixed left-bias to break symmetry
                angular_z = 0.4
            else:
                turn      = -self._prox_gain * torque
                angular_z = max(min(turn, self._w_max), -self._w_max)

        return linear_x, angular_z, is_avoiding

    # =========================================================
    # Helpers — floor & task
    # =========================================================

    def _get_floor_label(self) -> str:
        return self._last_floor.get('label', 'NONE').upper()

    def _get_service_time(self) -> int:
        specialization = abs(self._m)
        t = self._base_work - specialization * self._learn_step
        return int(max(t, self._min_work))

    def _prob_accept(self, task_type: str) -> float:
        """Sigmoid acceptance probability (Brutschy et al. 2012, Eq. 2)."""
        p_a = 1.0 / (1.0 + math.exp(-self._gamma * self._m))
        return p_a if task_type == 'TYPE_A' else 1.0 - p_a

    def _decide_task(self, task_type: str) -> bool:
        return random.random() < self._prob_accept(task_type)

    def _update_m_after_task(self, task_type: str):
        if task_type == 'TYPE_A':
            self._m = min(self._m + 1, self._m_max)
        else:
            self._m = max(self._m - 1, -self._m_max)
        self._forget_timer_s = 0.0

    def _apply_search_forgetting(self):
        self._forget_timer_s += self._loop_period
        if self._forget_timer_s >= self._forget_s:
            self._forget_timer_s = 0.0
            if self._m > 0:
                self._m -= 1
            elif self._m < 0:
                self._m += 1

    # =========================================================
    # Helpers — LEDs
    # =========================================================

    def _set_leds(self, r: int, g: int, b: int):
        new = (r, g, b)
        if self._last_led_cmd == new:
            return
        self._last_led_cmd = new
        msg      = String()
        msg.data = json.dumps({'mode': 'set_all', 'rgb': [r, g, b]})
        self._pub_led.publish(msg)

    def _update_patrol_leds(self):
        self._patrol_timer = (self._patrol_timer + 1) % self._patrol_blink_ticks
        if self._patrol_timer < self._patrol_on_ticks:
            self._set_leds(255, 165, 0)   # Orange
        else:
            self._set_leds(0, 0, 0)       # Off

    def _publish_velocity(self, v: float, w: float):
        msg             = Twist()
        msg.linear.x    = round(v, 3)
        msg.angular.z   = round(w, 3)
        self._pub_cmd.publish(msg)

    # =========================================================
    # Main behaviour loop
    # =========================================================

    def _step(self):
        v, w = 0.0, 0.0
        nav_v, nav_w, avoiding = self._get_vector_move_cmd()

        if self._ignore_ticks > 0:
            self._ignore_ticks -= 1

        current_color = self._get_floor_label()
        is_type_a     = current_color in self._type_a_colors
        is_type_b     = current_color in self._type_b_colors
        is_candidate  = is_type_a or is_type_b

        # ---- EXPLORE ----
        if self._state == States.EXPLORE:
            v, w = nav_v, nav_w

            if avoiding:
                self._set_leds(255, 100, 0)   # Amber while avoiding

            else:
                # Reset decision flag when floor returns to neutral
                if not is_candidate:
                    self._decision_made   = False
                    self._last_seen_color = 'NONE'

                if is_candidate and self._ignore_ticks == 0 and not self._decision_made:
                    task_type = 'TYPE_A' if is_type_a else 'TYPE_B'
                    accepted  = self._decide_task(task_type)

                    self._decision_made   = True
                    self._last_seen_color = current_color

                    if accepted:
                        self._current_task_type = task_type
                        self._current_job_time  = self._get_service_time()

                        self.get_logger().info(
                            f'[JOB] {task_type} ACCEPTED | '
                            f'T={self._current_job_time / self._loop_hz:.1f}s | '
                            f'm={self._m} | p={self._prob_accept(task_type):.2f}'
                        )

                        self._state       = States.EXECUTE
                        self._state_timer = 0
                        v, w = 0.0, 0.0

                    else:
                        self.get_logger().info(
                            f'[SKIP] {task_type} REJECTED | '
                            f'm={self._m} | p={self._prob_accept(task_type):.2f}'
                        )
                        self._reject_led_timer = self._refract_ticks
                        self._ignore_ticks     = self._refract_ticks

                # LED management during exploration
                if not is_candidate or self._ignore_ticks > 0:
                    if self._reject_led_timer > 0:
                        self._reject_led_timer -= 1
                        self._set_leds(255, 0, 0)      # Red → recent rejection
                    else:
                        self._apply_search_forgetting()
                        self._update_patrol_leds()     # Orange patrol blink

        # ---- EXECUTE ----
        elif self._state == States.EXECUTE:
            v, w = 0.0, 0.0
            self._state_timer += 1

            if self._current_task_type == 'TYPE_A':
                self._set_leds(0, 0, 255)     # Blue  — MAGENTA task
            else:
                self._set_leds(0, 255, 0)     # Green — YELLOW task

            if self._state_timer >= self._current_job_time:
                self._update_m_after_task(self._current_task_type)
                self._state = States.EXIT
                self.get_logger().info(
                    f'[DONE] {self._current_task_type} completed | '
                    f'm={self._m} | p_A={1.0/(1.0+math.exp(-self._gamma*self._m)):.2f} | '
                    f'T={self._current_job_time / self._loop_hz:.1f}s'
                )

        # ---- EXIT PATCH ----
        elif self._state == States.EXIT:
            v, w = nav_v, nav_w
            self._set_leds(0, 0, 0)

            if not is_candidate:
                self._state           = States.EXPLORE
                self._ignore_ticks    = self._refract_ticks
                self._decision_made   = False
                self._last_seen_color = 'NONE'
                self.get_logger().info('[FREE] Left patch — resuming exploration.')

        self._publish_velocity(v, w)

    def destroy_node(self):
        self._publish_velocity(0.0, 0.0)
        self._set_leds(0, 0, 0)
        super().destroy_node()


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = QupaExperimentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
