#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
experiment_node_classic — Brutschy baseline without camera or social learning.

Diagnostic twin of experiment_node.py used to isolate whether the camera /
social-learning code path is responsible for SSH drops on the robot.

Differences vs. the full experiment_node:
  - No camera subscription, ever.
  - No social-learning reward/penalty (Δ, F). After a task: m ±= 1.
  - Single integer specialisation counter m ∈ [-m_max, +m_max].
  - Scan subscription stays alive permanently (no destroy/recreate cycle).

Kept identical to the main node:
  - 5 Hz loop with rclpy.duration.Duration timers (no tick counters).
  - Vector-field obstacle avoidance + stuck-recovery escape rotation.
  - Patrol / reject / avoid LED feedback.
  - Time-based forgetting of |m| toward 0.

Subscribes:
  scan          sensor_msgs/LaserScan  IR distances (8 slots, 45° each)
  floor/color   std_msgs/String        JSON color label

Publishes:
  cmd_vel       geometry_msgs/Twist    velocity commands

Service clients:
  leds/set      qupa_msgs/LEDCommand   LED control
"""

import json
import math
import random

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from qupa_msgs.srv import LEDCommand


SENSOR_SLOTS: list[tuple[int, float]] = [
    (6, math.radians(-90.0)),   # derecha
    (7, math.radians(-45.0)),   # frente-derecha
    (0, math.radians(  0.0)),   # frente
    (1, math.radians( 45.0)),   # frente-izquierda
    (2, math.radians( 90.0)),   # izquierda
]


class States:
    EXPLORE = 'EXPLORE'
    EXECUTE = 'EXECUTE'
    EXIT    = 'EXIT_PATCH'


class QupaExperimentClassicNode(Node):

    def __init__(self):
        super().__init__('experiment_node_classic')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('loop_rate_hz',     5.0)
        self.declare_parameter('refractory_s',     2.0)
        self.declare_parameter('fwd_speed_ratio',  0.6)
        self.declare_parameter('prox_threshold',   0.4)
        self.declare_parameter('prox_gain',        3.0)
        self.declare_parameter('torque_deadzone',  0.15)

        self.declare_parameter('stuck_threshold_s', 1.5)
        self.declare_parameter('escape_turn_deg',   180.0)
        self.declare_parameter('escape_turn_w_rps', 2.0)

        self.declare_parameter('type_a_colors', ['MAGENTA'])
        self.declare_parameter('type_b_colors', ['YELLOW'])

        self.declare_parameter('task_timing.base_work_s',    10.0)
        self.declare_parameter('task_timing.min_work_s',      3.0)
        self.declare_parameter('task_timing.learning_step_s', 1.0)

        self.declare_parameter('specialization.m_max',  5)
        self.declare_parameter('specialization.gamma',  1.0)

        self.declare_parameter('forgetting.forget_interval_s', 30.0)

        self.declare_parameter('patrol.period_s', 4.0)
        self.declare_parameter('patrol.on_s',     0.5)

        self.declare_parameter('v_max_mps',        0.08)
        self.declare_parameter('w_max_rps',        2.50)
        self.declare_parameter('obstacle_stop_cm', 15.0)
        self.declare_parameter('sensor_max_cm',    40.0)

        # ── Cache parameter values ────────────────────────────────────────────
        loop_hz             = self.get_parameter('loop_rate_hz').value
        self._loop_period   = 1.0 / loop_hz

        self._refract_dur   = Duration(seconds=self.get_parameter('refractory_s').value)

        v_max               = self.get_parameter('v_max_mps').value
        self._fwd_speed     = v_max * self.get_parameter('fwd_speed_ratio').value
        self._w_max         = self.get_parameter('w_max_rps').value
        self._prox_thresh   = self.get_parameter('prox_threshold').value
        self._prox_gain     = self.get_parameter('prox_gain').value
        self._torque_dz     = self.get_parameter('torque_deadzone').value
        self._min_dist_cm   = self.get_parameter('obstacle_stop_cm').value
        self._max_dist_cm   = self.get_parameter('sensor_max_cm').value

        self._stuck_dur     = Duration(
            seconds=self.get_parameter('stuck_threshold_s').value
        )
        escape_rad          = math.radians(self.get_parameter('escape_turn_deg').value)
        escape_w            = self.get_parameter('escape_turn_w_rps').value
        self._escape_turn_w   = escape_w
        self._escape_turn_dur = Duration(seconds=escape_rad / abs(escape_w))

        self._type_a_colors = list(self.get_parameter('type_a_colors').value)
        self._type_b_colors = list(self.get_parameter('type_b_colors').value)

        self._base_work_s   = self.get_parameter('task_timing.base_work_s').value
        self._min_work_s    = self.get_parameter('task_timing.min_work_s').value
        self._learn_step_s  = self.get_parameter('task_timing.learning_step_s').value

        self._m_max         = self.get_parameter('specialization.m_max').value
        self._gamma         = self.get_parameter('specialization.gamma').value

        self._forget_dur    = Duration(
            seconds=self.get_parameter('forgetting.forget_interval_s').value
        )

        self._patrol_period_ns = int(self.get_parameter('patrol.period_s').value * 1e9)
        self._patrol_on_ns     = int(self.get_parameter('patrol.on_s').value     * 1e9)

        # ── Sensor state ──────────────────────────────────────────────────────
        self._ranges: list[float] = [float('inf')] * 8
        self._last_floor: dict = {}

        # ── Behaviour state ───────────────────────────────────────────────────
        now = self.get_clock().now()

        self._state                = States.EXPLORE
        self._execute_start        = now
        self._current_job_duration = Duration(seconds=self._base_work_s)
        self._current_task_type    = None

        self._ignore_until         = now
        self._reject_led_until     = now
        self._last_forget_check    = now
        self._escape_turn_until    = now
        self._stuck_since          = None

        self._decision_made        = False
        self._last_seen_color      = 'NONE'

        # Original Brutschy single counter: m > 0 → TYPE_A, m < 0 → TYPE_B.
        self._m = 0

        self._last_led_cmd = None

        # ── Publishers / clients ──────────────────────────────────────────────
        self._pub_cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        self._led_cli = self.create_client(LEDCommand, 'set')

        # ── Subscriptions ─────────────────────────────────────────────────────
        # Both stay alive for the lifetime of the node (no swapping).
        self.create_subscription(LaserScan, 'scan',        self._scan_cb,  10)
        self.create_subscription(String,    'floor/color', self._floor_cb, 10)

        # ── Main loop ─────────────────────────────────────────────────────────
        self._timer = self.create_timer(self._loop_period, self._step)

        self._set_leds(255, 165, 0)
        self.get_logger().info(
            f'Experiment CLASSIC node ready @ {loop_hz:.1f} Hz | '
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
        min_m = self._min_dist_cm / 100.0
        max_m = self._max_dist_cm / 100.0
        if not math.isfinite(dist_m) or dist_m >= max_m:
            return 0.0
        if dist_m <= min_m:
            return 1.0
        return 1.0 - (dist_m - min_m) / (max_m - min_m)

    def _get_vector_move_cmd(self, now) -> tuple[float, float, bool]:
        if now < self._escape_turn_until:
            return 0.0, self._escape_turn_w, True

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
                if self._stuck_since is None:
                    self._stuck_since = now
                elif now - self._stuck_since >= self._stuck_dur:
                    self._escape_turn_until = now + self._escape_turn_dur
                    self._stuck_since       = None
                    self.get_logger().warn(
                        '[AVOID] Stuck in deadzone — forcing escape rotation.'
                    )
                    return 0.0, self._escape_turn_w, True
                angular_z = 0.4
            else:
                self._stuck_since = None
                turn      = -self._prox_gain * torque
                angular_z = max(min(turn, self._w_max), -self._w_max)
        else:
            self._stuck_since = None

        return linear_x, angular_z, is_avoiding

    # =========================================================
    # Helpers — task model (original Brutschy, no social)
    # =========================================================

    def _get_floor_label(self) -> str:
        return self._last_floor.get('label', 'NONE').upper()

    def _get_service_time_s(self) -> float:
        specialization = abs(self._m)
        t = self._base_work_s - specialization * self._learn_step_s
        return max(t, self._min_work_s)

    def _prob_accept(self, task_type: str) -> float:
        p_a = 1.0 / (1.0 + math.exp(-self._gamma * self._m))
        return p_a if task_type == 'TYPE_A' else 1.0 - p_a

    def _decide_task(self, task_type: str) -> bool:
        return random.random() < self._prob_accept(task_type)

    def _update_m_after_task(self, task_type: str):
        if task_type == 'TYPE_A':
            self._m = min(self._m + 1, self._m_max)
        else:
            self._m = max(self._m - 1, -self._m_max)
        self._last_forget_check = self.get_clock().now()

    def _apply_search_forgetting(self, now):
        if now - self._last_forget_check >= self._forget_dur:
            self._last_forget_check = now
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
        req         = LEDCommand.Request()
        req.command = json.dumps({'mode': 'set_all', 'rgb': [r, g, b]})
        self._led_cli.call_async(req)

    def _update_patrol_leds(self, now):
        phase_ns = now.nanoseconds % self._patrol_period_ns
        if phase_ns < self._patrol_on_ns:
            self._set_leds(255, 165, 0)
        else:
            self._set_leds(0, 0, 0)

    def _publish_velocity(self, v: float, w: float):
        msg           = Twist()
        msg.linear.x  = round(v, 3)
        msg.angular.z = round(w, 3)
        self._pub_cmd.publish(msg)

    # =========================================================
    # Main behaviour loop
    # =========================================================

    def _step(self):
        now = self.get_clock().now()
        v, w = 0.0, 0.0
        nav_v, nav_w, avoiding = self._get_vector_move_cmd(now)

        ignoring = now < self._ignore_until

        current_color = self._get_floor_label()
        is_type_a     = current_color in self._type_a_colors
        is_type_b     = current_color in self._type_b_colors
        is_candidate  = is_type_a or is_type_b

        # ---- EXPLORE ----
        if self._state == States.EXPLORE:
            v, w = nav_v, nav_w

            if avoiding:
                self._set_leds(255, 100, 0)

            else:
                if not is_candidate:
                    self._decision_made   = False
                    self._last_seen_color = 'NONE'

                if is_candidate and not ignoring and not self._decision_made:
                    task_type = 'TYPE_A' if is_type_a else 'TYPE_B'
                    accepted  = self._decide_task(task_type)

                    self._decision_made   = True
                    self._last_seen_color = current_color

                    if accepted:
                        service_s = self._get_service_time_s()
                        self._current_task_type    = task_type
                        self._current_job_duration = Duration(seconds=service_s)
                        self._execute_start        = now

                        self.get_logger().info(
                            f'[JOB] {task_type} ACCEPTED | T={service_s:.1f}s | '
                            f'm={self._m} | p={self._prob_accept(task_type):.2f}'
                        )

                        self._state = States.EXECUTE
                        v, w = 0.0, 0.0

                    else:
                        self.get_logger().info(
                            f'[SKIP] {task_type} REJECTED | '
                            f'm={self._m} | p={self._prob_accept(task_type):.2f}'
                        )
                        self._reject_led_until = now + self._refract_dur
                        self._ignore_until     = now + self._refract_dur

                if not is_candidate or ignoring:
                    if now < self._reject_led_until:
                        self._set_leds(255, 0, 0)
                    else:
                        self._apply_search_forgetting(now)
                        self._update_patrol_leds(now)

        # ---- EXECUTE ----
        elif self._state == States.EXECUTE:
            v, w = 0.0, 0.0

            if self._current_task_type == 'TYPE_A':
                self._set_leds(0, 0, 255)
            else:
                self._set_leds(0, 255, 0)

            if now - self._execute_start >= self._current_job_duration:
                self._update_m_after_task(self._current_task_type)
                self._state = States.EXIT
                self.get_logger().info(
                    f'[DONE] {self._current_task_type} completed | '
                    f'm={self._m} | p_A={1.0/(1.0+math.exp(-self._gamma*self._m)):.2f} | '
                    f'T={self._current_job_duration.nanoseconds * 1e-9:.1f}s'
                )

        # ---- EXIT PATCH ----
        elif self._state == States.EXIT:
            v, w = nav_v, nav_w
            self._set_leds(0, 0, 0)

            if not is_candidate:
                self._state           = States.EXPLORE
                self._ignore_until    = now + self._refract_dur
                self._decision_made   = False
                self._last_seen_color = 'NONE'
                self.get_logger().info('[FREE] Left patch — resuming exploration.')

        self._publish_velocity(v, w)

    def destroy_node(self):
        self._publish_velocity(0.0, 0.0)
        self._set_leds(0, 0, 0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = QupaExperimentClassicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
