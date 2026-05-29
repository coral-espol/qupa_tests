#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
experiment_node — ROS2 port of the MQTT-based QupaExperiment brain.

Implements the Brutschy et al. (2012) self-organised task-allocation model
with vector-field obstacle avoidance using the robot's IR proximity sensors.

All durations are expressed in seconds and tracked via the ROS clock
(`self.get_clock().now()`), so behaviour is independent of `loop_rate_hz`
and compatible with `use_sim_time` for bag replay / simulation.

Subscribes:
  scan          sensor_msgs/LaserScan  IR distances (8 slots, 45° each)
  floor/color   std_msgs/String        JSON color label

Publishes:
  cmd_vel       geometry_msgs/Twist    velocity commands

Service clients:
  leds/set      qupa_msgs/LEDCommand   LED control
"""

import csv
import json
import math
import os
import random

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool
from qupa_msgs.msg import DetectionArray
from qupa_msgs.srv import LEDCommand


# ── Scan slot → robot-frame angle ────────────────────────────────────────────
# ir_scanner_node publica un único LaserScan ('scan') con 8 slots de 45°.
# Convención confirmada en el robot físico: slot 0 = frente.
# Ángulo en frame del robot: positivo = izquierda (convención ROS).
#
#  slot  ángulo_robot  sensor
#    6     -90°  (derecha)          ✓
#    7     -45°  (frente-derecha)   ✓
#    0       0°  (frente)           ✓
#    1     +45°  (frente-izquierda) ✓
#    2     +90°  (izquierda)        ✓
#    3     +135° (sin sensor → inf)
#    4     +180° (trasero)          ✓  — no usado en avoidance
#    5     +225° (sin sensor → inf)
SENSOR_SLOTS: list[tuple[int, float]] = [
    (6, math.radians(-90.0)),   # derecha
    (7, math.radians(-45.0)),   # frente-derecha
    (0, math.radians(  0.0)),   # frente
    (1, math.radians( 45.0)),   # frente-izquierda
    (2, math.radians( 90.0)),   # izquierda
]


class States:
    WAITING = 'WAITING'
    EXPLORE = 'EXPLORE'
    EXECUTE = 'EXECUTE'
    EXIT    = 'EXIT_PATCH'


class QupaExperimentNode(Node):

    def __init__(self):
        super().__init__('experiment_node')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('loop_rate_hz',     5.0)
        self.declare_parameter('refractory_s',     2.0)
        self.declare_parameter('fwd_speed_ratio',  1.0)
        self.declare_parameter('prox_threshold',   0.05)
        self.declare_parameter('prox_gain',        2.0)
        self.declare_parameter('torque_deadzone',  0.6)

        # Stuck recovery — forced in-place rotation when deadzone persists.
        self.declare_parameter('stuck_threshold_s', 1.5)
        self.declare_parameter('escape_turn_deg',   180.0)
        self.declare_parameter('escape_turn_w_rps', 2.0)

        self.declare_parameter('type_a_colors', ['MAGENTA'])
        self.declare_parameter('type_b_colors', ['YELLOW'])

        # Task timing — all in seconds; m affects duration linearly.
        self.declare_parameter('task_timing.base_work_s',    60.0)
        self.declare_parameter('task_timing.min_work_s',      8.0)
        self.declare_parameter('task_timing.learning_step_s', 1.0)

        self.declare_parameter('specialization.m_max',  12)
        self.declare_parameter('specialization.gamma',  1.0)
        self.declare_parameter('specialization.k',      1.15)

        # Social-learning reward/penalty modulation (see Δ and F formulas).
        # n_same = neighbours running the same task as the one just completed.
        #   Δ = 1 + α · min(n_same, delta_cap_n)   → reward to current task
        #   F = 1 + β · min(n_same, forget_cap_n)  → penalty to opposite task
        self.declare_parameter('social.alpha',             0.9)
        self.declare_parameter('social.beta',              1.5)
        self.declare_parameter('social.delta_cap_n',       3)
        self.declare_parameter('social.forget_cap_n',      2)
        self.declare_parameter('social.forget_saturation', 3.7)
        # 'iterative' = max simultáneo durante EXECUTE  (Lua: SOCIAL_BOOL=true)
        # 'snapshot'  = conteo al inicio del EXECUTE     (Lua: SOCIAL_BOOL=false)
        self.declare_parameter('social.count_mode',           'iterative')
        # Pixel-proximity threshold for clustering same-color blobs as a
        # single robot (mirrors Lua CLUSTER_THRESHOLD_CM = 15 cm).
        self.declare_parameter('social.cluster_threshold_px', 40.0)
        # Greedy bypasses the sigmoid and always accepts a candidate patch.
        self.declare_parameter('greedy_mode',         False)

        # CSV output — empty string disables logging.
        self.declare_parameter('data_log_path', '')

        self.declare_parameter('forgetting.forget_interval_s', 30.0)

        self.declare_parameter('patrol.period_s', 4.0)
        self.declare_parameter('patrol.on_s',     0.5)

        # Camera topic for social-learning observation during EXECUTE.
        # Expected payload: std_msgs/String with JSON {"BLUE": N, "GREEN": M}
        # where BLUE counts robots running a TYPE_A task and GREEN TYPE_B.
        self.declare_parameter('camera_topic', 'camera/detections')

        # Motor kinematics
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
        self._escape_turn_w   = escape_w                                   # rad/s, +left
        self._escape_turn_dur = Duration(seconds=escape_rad / abs(escape_w))

        self._type_a_colors = list(self.get_parameter('type_a_colors').value)
        self._type_b_colors = list(self.get_parameter('type_b_colors').value)

        self._base_work_s   = self.get_parameter('task_timing.base_work_s').value
        self._min_work_s    = self.get_parameter('task_timing.min_work_s').value
        self._learn_step_s  = self.get_parameter('task_timing.learning_step_s').value

        self._m_max         = self.get_parameter('specialization.m_max').value
        self._gamma         = self.get_parameter('specialization.gamma').value
        self._k             = self.get_parameter('specialization.k').value
        self._c             = self._m_max / 2.0   # sigmoid midpoint (Lua: c = N_MAX/2)

        self._alpha         = self.get_parameter('social.alpha').value
        self._beta          = self.get_parameter('social.beta').value
        self._delta_cap_n   = int(self.get_parameter('social.delta_cap_n').value)
        self._forget_cap_n  = int(self.get_parameter('social.forget_cap_n').value)
        self._forget_sat    = self.get_parameter('social.forget_saturation').value
        self._count_mode    = self.get_parameter('social.count_mode').value
        self._cluster_thr   = self.get_parameter('social.cluster_threshold_px').value
        self._greedy_mode   = bool(self.get_parameter('greedy_mode').value)

        self._forget_dur    = Duration(
            seconds=self.get_parameter('forgetting.forget_interval_s').value
        )

        # Patrol blink — phase computed directly from wall clock nanoseconds.
        self._patrol_period_ns = int(self.get_parameter('patrol.period_s').value * 1e9)
        self._patrol_on_ns     = int(self.get_parameter('patrol.on_s').value     * 1e9)

        self._camera_topic = self.get_parameter('camera_topic').value

        # ── Sensor state ──────────────────────────────────────────────────────
        self._ranges: list[float] = [float('inf')] * 8
        self._last_floor: dict = {}

        # ── Behaviour state ───────────────────────────────────────────────────
        now = self.get_clock().now()

        self._state                = States.WAITING
        self._execute_start        = now
        self._current_job_duration = Duration(seconds=self._base_work_s)
        self._current_task_type    = None

        # Timestamps in the past → "expired" (not active).
        self._ignore_until         = now
        self._reject_led_until     = now
        self._last_forget_check    = now
        self._escape_turn_until    = now
        self._stuck_since          = None  # None → currently not stuck

        self._decision_made        = False
        self._last_seen_color      = 'NONE'

        # Specialisation counters per task type (used for service-time sigmoid).
        # Each n ∈ [0, m_max].
        self._n = {'TYPE_A': 0.0, 'TYPE_B': 0.0}

        # Signed specialisation m ∈ [-m_max, +m_max] used by the *acceptance*
        # sigmoid. Mirrors the Lua: m is an independent state updated by ±delta
        # after each task, NOT derived from n[A] − n[B]. The two may diverge
        # because n is bounded ≥ 0 and decays independently.
        self._m = 0.0

        # ── Experiment timer state ────────────────────────────────────────────
        # Set to True when /experiment/running publishes True.
        # CSV rows are only written while the experiment is active.
        self._experiment_running    = False
        self._experiment_start_time = None
        self._search_start_time     = now   # reset each time EXPLORE resumes

        # ── Data logging ──────────────────────────────────────────────────────
        self._csv_file   = None
        self._csv_writer = None

        log_path = self.get_parameter('data_log_path').value
        if log_path:
            os.makedirs(os.path.dirname(log_path), exist_ok=True)
            self._csv_file   = open(log_path, 'w', newline='')
            self._csv_writer = csv.writer(self._csv_file)
            self._csv_writer.writerow(
                ['tick', 'greedy', 'robot', 'm', 'p_x',
                 'planned_wticks', 'task', 'search_ticks', 'x', 'y', 'seed']
            )
            self._csv_file.flush()
            self.get_logger().info(f'Data logging → {log_path}')

        # LED state cache — suppresses redundant calls but retries if the
        # service response is never received (WiFi packet loss).
        self._last_led_cmd       = None
        self._last_led_send_time = now

        # Social-learning observation buffers.
        #   _neighbor_counts     → latest sample from the camera
        #   _max_neighbor_counts → peak simultaneous count during EXECUTE
        #   _snapshot_counts     → counts captured at the moment EXECUTE begins
        # Which one is fed into Δ/F is chosen by `social.count_mode`:
        #   'iterative' → max (Lua SOCIAL_BOOL=true)
        #   'snapshot'  → snapshot at start of EXECUTE (Lua SOCIAL_BOOL=false)
        self._neighbor_counts     = {'TYPE_A': 0, 'TYPE_B': 0}
        self._max_neighbor_counts = {'TYPE_A': 0, 'TYPE_B': 0}
        self._snapshot_counts     = {'TYPE_A': 0, 'TYPE_B': 0}
        self._tracking_exec_max   = False

        # ── Publishers / clients ──────────────────────────────────────────────
        self._pub_cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        self._led_cli = self.create_client(LEDCommand, 'set')

        # ── Subscriptions ─────────────────────────────────────────────────────
        # scan: active during EXPLORE / EXIT_PATCH only — torn down in EXECUTE.
        # camera: always active so a fresh snapshot exists the moment we enter
        #   EXECUTE (mirrors the Lua, where the omnidirectional camera is
        #   queried directly at the patch-entry instant).
        # floor/color: always active (needed to detect patches anytime).
        # /experiment/running: TRANSIENT_LOCAL so late-joining nodes receive the
        #   current state immediately upon connection.
        self._scan_sub = None
        # Scan activates only when experiment starts (WAITING → EXPLORE).
        self.create_subscription(String,         'floor/color',      self._floor_cb,  10)
        self.create_subscription(DetectionArray, self._camera_topic, self._camera_cb, 10)

        _latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            Bool, '/experiment/running', self._experiment_state_cb, _latched_qos
        )

        # ── Main loop ─────────────────────────────────────────────────────────
        self._timer = self.create_timer(self._loop_period, self._step)

        self._set_leds(255, 255, 255)   # White = waiting for experiment start
        self.get_logger().info(
            f'Experiment node ready @ {loop_hz:.1f} Hz | '
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

    def _experiment_state_cb(self, msg: Bool):
        if msg.data and not self._experiment_running:
            now = self.get_clock().now()
            self._experiment_start_time = now
            self._search_start_time     = now
            self._state = States.EXPLORE
            self._activate_scan()
            self.get_logger().info('Experiment STARTED — beginning exploration.')
        elif not msg.data and self._experiment_running:
            if self._state == States.EXECUTE:
                self._end_exec_observation()
            self._deactivate_scan()
            self._state = States.WAITING
            self.get_logger().info('Experiment ENDED — shutting down.')
            self._shutdown_timer = self.create_timer(0.5, self._do_shutdown)
        self._experiment_running = msg.data

    def _do_shutdown(self):
        self._shutdown_timer.cancel()
        rclpy.try_shutdown()

    def _camera_cb(self, msg: DetectionArray):
        try:
            # Cluster same-color blobs that are within `cluster_threshold_px`
            # of an existing cluster centroid (in image-pixel space). Each
            # surviving cluster represents one neighbour robot. Mirrors the
            # Lua's CLUSTER_THRESHOLD_CM grouping by radial distance.
            #
            # Done in two passes:
            #   1) Sequential merge as detections arrive (cheap).
            #   2) Iterative pass that collapses any two final clusters still
            #      within threshold — fixes order-dependent splits caused by
            #      a noisy blob breaking the merge chain.
            clusters = {'TYPE_A': [], 'TYPE_B': []}
            thr2     = self._cluster_thr * self._cluster_thr
            for det in msg.targets:
                if   det.color == 'BLUE':  key = 'TYPE_A'
                elif det.color == 'GREEN': key = 'TYPE_B'
                else: continue

                merged = False
                for i, (ccx, ccy) in enumerate(clusters[key]):
                    dx, dy = det.cx - ccx, det.cy - ccy
                    if dx * dx + dy * dy < thr2:
                        clusters[key][i] = ((ccx + det.cx) / 2.0,
                                            (ccy + det.cy) / 2.0)
                        merged = True
                        break
                if not merged:
                    clusters[key].append((det.cx, det.cy))

            # Second pass: keep merging cluster pairs until no two are within
            # threshold (handles cases where the order of incoming detections
            # broke the chain in pass 1).
            for key in ('TYPE_A', 'TYPE_B'):
                changed = True
                while changed:
                    changed = False
                    n = len(clusters[key])
                    for i in range(n):
                        for j in range(i + 1, n):
                            ax, ay = clusters[key][i]
                            bx, by = clusters[key][j]
                            dx, dy = ax - bx, ay - by
                            if dx * dx + dy * dy < thr2:
                                clusters[key][i] = ((ax + bx) / 2.0,
                                                    (ay + by) / 2.0)
                                clusters[key].pop(j)
                                changed = True
                                break
                        if changed:
                            break

            new_counts = {'TYPE_A': len(clusters['TYPE_A']),
                          'TYPE_B': len(clusters['TYPE_B'])}

            # While tracking (i.e. during EXECUTE), log every change in the
            # same-task count so the social-learning input can be visualised
            # tick-by-tick. The peak (which feeds Δ) is also shown.
            if self._tracking_exec_max and self._current_task_type is not None:
                task = self._current_task_type
                old  = self._neighbor_counts[task]
                new  = new_counts[task]
                if new != old:
                    peak     = max(self._max_neighbor_counts[task], new)
                    centers  = ', '.join(
                        f'({cx:.0f},{cy:.0f})' for cx, cy in clusters[task]
                    ) or '-'
                    self.get_logger().info(
                        f'[SOCIAL] {task} same-task neighbours: '
                        f'{old} → {new} (peak={peak}) | centers: {centers}'
                    )

            self._neighbor_counts = new_counts

            if self._tracking_exec_max:
                for k in ('TYPE_A', 'TYPE_B'):
                    if self._neighbor_counts[k] > self._max_neighbor_counts[k]:
                        self._max_neighbor_counts[k] = self._neighbor_counts[k]
        except Exception:
            pass

    # =========================================================
    # Subscription lifecycle (per-state activation)
    # =========================================================

    def _activate_scan(self):
        if self._scan_sub is None:
            self._scan_sub = self.create_subscription(
                LaserScan, 'scan', self._scan_cb, 10
            )

    def _deactivate_scan(self):
        if self._scan_sub is not None:
            self.destroy_subscription(self._scan_sub)
            self._scan_sub = None
            # Clear stale ranges so the next nav cycle after re-activation
            # doesn't fire avoidance on data from minutes ago.
            self._ranges = [float('inf')] * 8

    def _begin_exec_observation(self):
        """Called at the moment we enter EXECUTE. Captures the snapshot first,
        then resets both the latest-sample and peak buffers to zero so this
        task's observation starts fresh (Lua: max_social_seen = 0 at task
        start). The first incoming camera message will then log 0 → N."""
        self._snapshot_counts     = dict(self._neighbor_counts)
        self._neighbor_counts     = {'TYPE_A': 0, 'TYPE_B': 0}
        self._max_neighbor_counts = {'TYPE_A': 0, 'TYPE_B': 0}
        self._tracking_exec_max   = True

    def _end_exec_observation(self):
        self._tracking_exec_max = False

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

    def _get_vector_move_cmd(self, now) -> tuple[float, float, bool]:
        """Return (linear_x, angular_z, is_avoiding)."""
        # Forced in-place escape rotation overrides everything until it ends.
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
                # Front blocked but laterals give no usable turn direction.
                # Start (or continue) the stuck timer; if it elapses, escape.
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
    # Helpers — floor & task
    # =========================================================

    def _get_floor_label(self) -> str:
        return self._last_floor.get('label', 'NONE').upper()

    def _get_service_time_s(self, task_type: str) -> float:
        # Lua's short-circuit: n=0 → full base work, no descuento.
        specialization = self._n[task_type]
        if specialization <= 0:
            return self._base_work_s
        t = self._base_work_s - (self._base_work_s / (self._k * (1 + math.exp(-specialization + self._c))))
        return max(t, self._min_work_s)

    def _prob_accept(self, task_type: str) -> float:
        """Sigmoid acceptance probability (Brutschy et al. 2012, Eq. 2)."""
        p_a = 1.0 / (1.0 + math.exp(-self._gamma * self._m))
        return p_a if task_type == 'TYPE_A' else 1.0 - p_a

    def _decide_task(self, task_type: str) -> bool:
        if self._greedy_mode:
            return True
        return random.random() < self._prob_accept(task_type)

    def _social_delta(self, n_same: int) -> float:
        # Mirrors Lua: reward saturates at 1 + α·cap (= 3.7 with α=0.9, cap=3).
        # The applied delta is 1.0 (individual increment) + reward (social).
        n      = min(n_same, self._delta_cap_n)
        reward = 1.0 + self._alpha * n
        return reward

    def _social_forget(self, n_same: int) -> float:
        # Mirrors Lua: piecewise — linear under cap, hardcoded saturation above.
        # The Lua's BETA_SOCIAL var is unused; the formula uses 1.5 directly,
        # but the saturation is a separate constant (3.7), so we expose both.
        if n_same < self._forget_cap_n:
            return 1.0 + self._beta * n_same
        return self._forget_sat

    def _update_specialization_after_task(
        self, task_type: str, n_same_neighbors: int,
    ) -> tuple[float, float]:
        delta    = self._social_delta(n_same_neighbors)
        forget   = self._social_forget(n_same_neighbors)
        opposite = 'TYPE_B' if task_type == 'TYPE_A' else 'TYPE_A'

        # Per-task counters (service-time sigmoid input).
        self._n[task_type] = min(self._n[task_type] + delta,  self._m_max)
        self._n[opposite]  = max(self._n[opposite]  - forget, 0.0)

        # Signed memory m (acceptance sigmoid input) — independent state,
        # updated by ±delta only (Lua convention).
        if task_type == 'TYPE_A':
            self._m = max(min(self._m + delta,  self._m_max), -self._m_max)
        else:
            self._m = max(min(self._m - delta,  self._m_max), -self._m_max)

        self._last_forget_check = self.get_clock().now()
        return delta, forget

    def _apply_search_forgetting(self, now):
        if now - self._last_forget_check >= self._forget_dur:
            self._last_forget_check = now
            self._n['TYPE_A'] = max(self._n['TYPE_A'] - 1.0, 0.0)
            self._n['TYPE_B'] = max(self._n['TYPE_B'] - 1.0, 0.0)
            # m decays toward 0 by 1 (Lua's distance-based decay analogue).
            if   self._m > 0: self._m = max(self._m - 1.0, 0.0)
            elif self._m < 0: self._m = min(self._m + 1.0, 0.0)

    # =========================================================
    # Helpers — LEDs
    # =========================================================

    def _set_leds(self, r: int, g: int, b: int):
        new   = (r, g, b)
        now   = self.get_clock().now()
        age_s = (now - self._last_led_send_time).nanoseconds * 1e-9
        if self._last_led_cmd == new and age_s < 2.0:
            return
        self._last_led_cmd       = new
        self._last_led_send_time = now
        req         = LEDCommand.Request()
        req.command = json.dumps({'mode': 'set_all', 'rgb': [r, g, b]})
        fut = self._led_cli.call_async(req)
        fut.add_done_callback(lambda f: self._led_done_cb(f))

    def _led_done_cb(self, future):
        try:
            result = future.result()
            if not result.success:
                self.get_logger().warn(f'LED service returned failure: {result.message}')
                self._last_led_cmd = None   # force retry next tick
        except Exception as e:
            self.get_logger().warn(f'LED service call failed: {e}')
            self._last_led_cmd = None       # force retry next tick

    def _update_patrol_leds(self, now):
        phase_ns = now.nanoseconds % self._patrol_period_ns
        if phase_ns < self._patrol_on_ns:
            self._set_leds(0, 0, 0)   # Orange
        else:
            self._set_leds(0, 0, 0)       # Off

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

        # ---- WAITING — parado hasta recibir la señal del timer ----
        if self._state == States.WAITING:
            self._publish_velocity(0.0, 0.0)
            self._set_leds(255, 255, 255)   # White = waiting
            return

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
                self._set_leds(0, 0, 0)   # Amber while avoiding

            else:
                # Reset decision flag when floor returns to neutral,
                # or when the refractory timer expires while still on the patch
                # (needed when the robot is stationary — it never leaves the patch).
                if not is_candidate:
                    self._decision_made   = False
                    self._last_seen_color = 'NONE'
                elif not ignoring and self._decision_made:
                    self._decision_made = False

                if is_candidate and not ignoring and not self._decision_made:
                    task_type = 'TYPE_A' if is_type_a else 'TYPE_B'
                    accepted  = self._decide_task(task_type)

                    self._decision_made   = True
                    self._last_seen_color = current_color

                    if accepted:
                        service_s = self._get_service_time_s(task_type)
                        self._current_task_type    = task_type
                        self._current_job_duration = Duration(seconds=service_s)
                        self._execute_start        = now

                        self.get_logger().info(
                            f'[JOB] {task_type} ACCEPTED | T={service_s:.1f}s | '
                            f'm={self._m:.1f} | '
                            f'n_a={self._n["TYPE_A"]:.1f} n_b={self._n["TYPE_B"]:.1f} | '
                            f'p={self._prob_accept(task_type):.2f}'
                        )

                        if (self._csv_writer is not None
                                and self._experiment_running
                                and self._experiment_start_time is not None):
                            elapsed_s = (now - self._experiment_start_time).nanoseconds * 1e-9
                            search_s  = (now - self._search_start_time).nanoseconds * 1e-9
                            self._csv_writer.writerow([
                                f'{elapsed_s:.3f}',
                                str(self._greedy_mode).lower(),
                                self.get_namespace().strip('/') or self.get_name(),
                                f'{self._m:.6f}',
                                f'{self._prob_accept(task_type):.6f}',
                                f'{service_s:.3f}',
                                task_type,
                                f'{search_s:.3f}',
                                '0',
                                '0',
                                '0',
                            ])
                            self._csv_file.flush()

                        # Robot will be still during EXECUTE → release scan
                        # (no need to avoid). Camera stays subscribed always.
                        self._deactivate_scan()
                        self._begin_exec_observation()

                        self._state = States.EXECUTE
                        v, w = 0.0, 0.0

                    else:
                        self.get_logger().info(
                            f'[SKIP] {task_type} REJECTED | '
                            f'm={self._m:.1f} | '
                            f'n_a={self._n["TYPE_A"]:.1f} n_b={self._n["TYPE_B"]:.1f} | '
                            f'p={self._prob_accept(task_type):.2f}'
                        )
                        self._reject_led_until = now + self._refract_dur
                        self._ignore_until     = now + self._refract_dur

                # LED management during exploration
                if not is_candidate or ignoring:
                    if now < self._reject_led_until:
                        self._set_leds(255, 0, 0)      # Red → recent rejection
                    else:
                        self._apply_search_forgetting(now)
                        self._update_patrol_leds(now)  # Orange patrol blink

        # ---- EXECUTE ----
        # Robot stays put while the task runs. Scan is off (no need to avoid),
        # camera is on so we can observe neighbour LEDs for social learning.
        elif self._state == States.EXECUTE:
            v, w = 0.0, 0.0

            if self._current_task_type == 'TYPE_A':
                self._set_leds(0, 0, 255)     # Blue  — MAGENTA task
            else:
                self._set_leds(0, 255, 0)     # Green — YELLOW task

            if now - self._execute_start >= self._current_job_duration:
                self._end_exec_observation()

                peak_a = self._max_neighbor_counts['TYPE_A']
                peak_b = self._max_neighbor_counts['TYPE_B']

                # Pick the neighbour count fed into Δ/F per `social.count_mode`.
                if self._count_mode == 'snapshot':
                    n_same = self._snapshot_counts[self._current_task_type]
                else:  # 'iterative' (default)
                    n_same = self._max_neighbor_counts[self._current_task_type]

                delta, forget = self._update_specialization_after_task(
                    self._current_task_type, n_same
                )

                # Camera stays subscribed; just bring scan back for EXIT_PATCH.
                self._activate_scan()

                # _ignore_until doubles as the EXIT timeout: if the robot is
                # stationary it will never physically leave the patch, so this
                # timer lets EXIT transition back to EXPLORE after refractory_s.
                self._ignore_until = now + self._refract_dur
                self._state = States.EXIT
                p_a = 1.0 / (1.0 + math.exp(-self._gamma * self._m))
                self.get_logger().info(
                    f'[DONE] {self._current_task_type} completed | '
                    f'peak A={peak_a} B={peak_b} (n_same={n_same}, mode={self._count_mode}) | '
                    f'Δ={delta:.2f} F={forget:.2f} | '
                    f'm={self._m:.1f} | '
                    f'n_a={self._n["TYPE_A"]:.1f} n_b={self._n["TYPE_B"]:.1f} | '
                    f'p_A={p_a:.2f} | '
                    f'T={self._current_job_duration.nanoseconds * 1e-9:.1f}s'
                )

        # ---- EXIT PATCH ----
        elif self._state == States.EXIT:
            v, w = nav_v, nav_w
            self._set_leds(0, 0, 0)

            if not is_candidate or now >= self._ignore_until:
                self._state             = States.EXPLORE
                self._search_start_time = now
                self._ignore_until      = now + self._refract_dur
                self._decision_made     = False
                self._last_seen_color   = 'NONE'
                self.get_logger().info('[FREE] Left patch — resuming exploration.')

        self._publish_velocity(v, w)

    def destroy_node(self):
        self._publish_velocity(0.0, 0.0)
        req = LEDCommand.Request()
        req.command = json.dumps({'mode': 'clear'})
        if self._led_cli.service_is_ready():
            future = self._led_cli.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        self._deactivate_scan()
        if self._csv_file is not None:
            self._csv_file.close()
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
