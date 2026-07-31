#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
color_behavior_node — Comportamiento reactivo por color para el experimento QUPA.

Máquina de estados sencilla, una instancia por robot (namespaceada):

  EXPLORE   Random walk / exploración con evasión de obstáculos (usa /scan).
  STOP_GREEN Si la cámara detecta VERDE → frena en el acto (v=w=0) mientras
             el verde siga visible. No toca los LEDs.
  AGGREGATE  Si la cámara detecta AZUL → se orienta hacia el blob y avanza
             (agregación hacia el color).
  ARRIVED    Al llegar a <arrival_distance_m> del elemento (medido con /scan
             hacia el frente) → frena y enciende sus LEDs en AZUL. Se queda
             como "baliza" azul mientras siga viendo azul cerca.

Prioridad por tick:  VERDE  >  AZUL  >  EXPLORE
(Como el panel de la arena es de un solo color a la vez, el conflicto solo
 aparece si un robot ve un panel verde y a la vez el LED azul de otro robot;
 en ese caso el "freno por verde" gana, a modo de parada de seguridad.)

Suscribe (relativos al namespace):
  camera/detections   qupa_msgs/DetectionArray   (color, angle_deg, area, ...)
  scan                sensor_msgs/LaserScan      distancias (8 rayos, 45°)

Publica:
  cmd_vel             geometry_msgs/Twist

Cliente de servicio:
  set                 qupa_msgs/LEDCommand       control de LEDs (JSON)

Notas de sintonía:
  · steer_sign: el mapeo del ángulo de la fisheye al giro del robot depende del
    montaje del espejo. Si en AGGREGATE el robot gira ALEJÁNDOSE del azul en vez
    de acercarse, invierte este parámetro (1.0 <-> -1.0).
  · arrival_distance_m: distancia de llegada (default 0.15 m = 15 cm). Se mide
    con el sector frontal del /scan, cuyo mínimo físico es 0.08 m.
"""

import math
import random

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from qupa_msgs.msg import DetectionArray
from qupa_msgs.srv import LEDCommand

# Estados
EXPLORE     = 'EXPLORE'
STOP_RED    = 'STOP_RED'       # rojo  → paro
REPEL_GREEN = 'REPEL_GREEN'    # verde → repulsión (huye)
AGGREGATE   = 'AGGREGATE'      # azul  → agregación
ARRIVED     = 'ARRIVED'


class ColorBehaviorNode(Node):

    def __init__(self):
        super().__init__('color_behavior_node')

        # ── Parámetros ───────────────────────────────────────────────────────
        self.declare_parameter('loop_rate_hz',        10.0)
        # Velocidades
        self.declare_parameter('explore_speed_mps',   0.06)
        self.declare_parameter('aggregate_speed_mps', 0.06)
        self.declare_parameter('v_max_mps',           0.08)
        self.declare_parameter('w_max_rps',           2.0)
        # Colores (nombres tal como los publica la cámara)
        #   ROJO  → paro (freeze)       VERDE → repulsión (huye)
        #   AZUL  → agregación (+LED azul al llegar)
        self.declare_parameter('red_color',           'RED')
        self.declare_parameter('green_color',         'GREEN')
        self.declare_parameter('blue_color',          'BLUE')
        self.declare_parameter('blue_rgb',            [0, 0, 255])
        self.declare_parameter('repel_speed_mps',     0.06)
        # Detección
        self.declare_parameter('min_area',            120)      # px² mínimos
        self.declare_parameter('detection_timeout_s', 0.7)      # cámara ~3 Hz
        # Agregación / dirección
        #   steer_sign: +1 gira hacia el color; si va al lado opuesto, poner -1.
        self.declare_parameter('steer_sign',           1.0)
        self.declare_parameter('steer_gain',          0.03)     # rad/s por grado
        self.declare_parameter('heading_deadzone_deg', 8.0)     # gap ±8° al frente (no gira dentro)
        self.declare_parameter('face_deg',            25.0)     # |ángulo|<esto = "encarado" al objetivo
        self.declare_parameter('arrival_distance_m',  0.15)     # 15 cm (confirmación por lidar)
        self.declare_parameter('arrival_min_area',    800)      # px²: blob azul debe ser ≥ esto
        #   (confirmación por CÁMARA: sin esto, un obstáculo al frente encendería
        #    los LEDs sin haber llegado al azul de verdad)
        self.declare_parameter('front_half_deg',      50.0)     # arco frontal de seguridad (rayos 0,±45)
        # Exploración / evasión
        self.declare_parameter('obstacle_stop_m',     0.20)     # gira si algo < esto
        self.declare_parameter('avoid_turn_rps',      1.2)
        self.declare_parameter('wander_turn_rps',     0.6)
        self.declare_parameter('wander_min_s',        1.0)
        self.declare_parameter('wander_max_s',        3.0)

        p = self.get_parameter
        self._explore_v   = float(p('explore_speed_mps').value)
        self._aggr_v      = float(p('aggregate_speed_mps').value)
        self._v_max       = float(p('v_max_mps').value)
        self._w_max       = float(p('w_max_rps').value)
        self._red         = str(p('red_color').value).upper()
        self._green       = str(p('green_color').value).upper()
        self._blue        = str(p('blue_color').value).upper()
        self._blue_rgb    = [int(c) for c in p('blue_rgb').value]
        self._repel_v     = float(p('repel_speed_mps').value)
        self._min_area    = int(p('min_area').value)
        self._det_timeout = float(p('detection_timeout_s').value)
        self._steer_sign  = float(p('steer_sign').value)
        self._steer_gain  = float(p('steer_gain').value)
        self._deadzone    = float(p('heading_deadzone_deg').value)
        self._face_deg    = float(p('face_deg').value)
        self._arrival_m   = float(p('arrival_distance_m').value)
        self._arrival_area = int(p('arrival_min_area').value)
        self._front_half  = float(p('front_half_deg').value)
        self._obstacle_m  = float(p('obstacle_stop_m').value)
        self._avoid_w     = float(p('avoid_turn_rps').value)
        self._wander_w    = float(p('wander_turn_rps').value)
        self._wander_min  = float(p('wander_min_s').value)
        self._wander_max  = float(p('wander_max_s').value)

        # ── Estado interno ───────────────────────────────────────────────────
        self._last_det        = None    # DetectionArray más reciente
        self._last_det_time   = None    # rclpy.Time de recepción
        self._scan            = None    # LaserScan más reciente
        self._led_is_blue     = False   # estado LED actual (evita spam al servicio)
        self._state           = EXPLORE
        # Random walk
        self._wander_cmd_w    = 0.0
        self._wander_until    = self.get_clock().now()

        # ── I/O ──────────────────────────────────────────────────────────────
        self._pub_cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        self._led_cli = self.create_client(LEDCommand, 'set')
        self.create_subscription(DetectionArray, 'camera/detections',
                                 self._detections_cb, 10)
        self.create_subscription(LaserScan, 'scan', self._scan_cb, 10)

        self._timer = self.create_timer(1.0 / float(p('loop_rate_hz').value),
                                        self._control_tick)

        ns = self.get_namespace().strip('/') or '(root)'
        self.get_logger().info(
            f'color_behavior_node listo [ns={ns}] — rojo=paro, verde=repulsión, '
            f'azul=agregación (llegada {self._arrival_m*100:.0f} cm → LED azul)'
        )

    # ── Callbacks de sensores ───────────────────────────────────────────────

    def _detections_cb(self, msg: DetectionArray):
        self._last_det      = msg
        self._last_det_time = self.get_clock().now()

    def _scan_cb(self, msg: LaserScan):
        self._scan = msg

    # ── Helpers de detección ────────────────────────────────────────────────

    def _fresh_targets(self):
        """Detecciones vigentes (dentro del timeout) filtradas por área."""
        if self._last_det is None or self._last_det_time is None:
            return []
        age = (self.get_clock().now() - self._last_det_time).nanoseconds * 1e-9
        if age > self._det_timeout:
            return []
        return [t for t in self._last_det.targets if t.area >= self._min_area]

    # ── Helpers de scan ─────────────────────────────────────────────────────

    def _sector_min(self, center_deg: float, half_deg: float):
        """Rango mínimo válido en un sector angular (grados, robot frame).
        Devuelve None si no hay lecturas válidas (todo fuera de rango)."""
        s = self._scan
        if s is None or not s.ranges:
            return None
        best = None
        for i, r in enumerate(s.ranges):
            if not math.isfinite(r) or r < s.range_min or r > s.range_max:
                continue
            ang = math.degrees(s.angle_min + i * s.angle_increment)
            # Normaliza a (-180, 180]
            ang = (ang + 180.0) % 360.0 - 180.0
            diff = abs((ang - center_deg + 180.0) % 360.0 - 180.0)
            if diff <= half_deg:
                if best is None or r < best:
                    best = r
        return best

    def _forward_distance(self):
        return self._sector_min(0.0, self._front_half)

    # ── Comportamientos ─────────────────────────────────────────────────────

    def _avoid_dir(self):
        """Sentido de giro para esquivar: hacia el lado más despejado (+ = izq)."""
        left  = self._sector_min(90.0, 45.0)
        right = self._sector_min(-90.0, 45.0)
        lv = left  if left  is not None else float('inf')
        rv = right if right is not None else float('inf')
        return 1.0 if lv >= rv else -1.0

    def _explore(self):
        """Random walk con evasión reactiva. Devuelve (v, w)."""
        now   = self.get_clock().now()
        front = self._sector_min(0.0, self._front_half)   # arco frontal (rayos 0,±45)

        # Evasión: algo cerca al frente → girar hacia el lado más despejado.
        if front is not None and front < self._obstacle_m:
            self._wander_until = now                   # replanear al despejar
            return 0.0, self._avoid_dir() * self._avoid_w

        # Wander: cada cierto tiempo elige un nuevo giro aleatorio suave.
        if now >= self._wander_until:
            self._wander_cmd_w = random.uniform(-1.0, 1.0) * self._wander_w
            dur = random.uniform(self._wander_min, self._wander_max)
            self._wander_until = now + Duration(seconds=dur)

        return self._explore_v, self._wander_cmd_w

    def _steer_to(self, err_deg):
        """Velocidad angular hacia el objetivo con deadzone ±deadzone (gap frontal)."""
        if abs(err_deg) <= self._deadzone:
            return 0.0
        w = self._steer_sign * self._steer_gain * err_deg
        return max(-self._w_max, min(self._w_max, w))

    def _flee_green(self, green_target):
        """Repulsión: orienta el frente en sentido OPUESTO al verde y avanza,
        con evasión de obstáculos para no huir contra una pared. Devuelve (v, w)."""
        # Ángulo hacia donde queremos mirar = opuesto al verde (verde + 180°).
        away_err = ((float(green_target.angle_deg) + 180.0 + 180.0) % 360.0) - 180.0
        w = self._steer_to(away_err)

        front = self._sector_min(0.0, self._front_half)
        # Si hay pared/obstáculo al frente mientras huye → esquiva.
        if front is not None and front < self._obstacle_m:
            return 0.0, self._avoid_dir() * self._avoid_w

        face_scale = max(0.0, 1.0 - abs(away_err) / 90.0)   # avanza al encarar el "lejos"
        v = self._repel_v * face_scale
        return v, w

    def _go_to_blue(self, blue_target):
        """Agregación hacia el azul, con evasión y llegada robusta.
        Devuelve (v, w, arrived)."""
        err_deg = float(blue_target.angle_deg)          # 0 = frente
        w       = self._steer_to(err_deg)
        facing  = abs(err_deg) <= self._face_deg
        front   = self._sector_min(0.0, self._front_half)   # arco ancho: pilla la pared

        close_lidar = front is not None and front <= self._arrival_m   # hay algo a ≤15 cm
        close_cam   = blue_target.area >= self._arrival_area           # el azul se ve GRANDE

        # 1) Llegada REAL: encarado + lidar cerca + la cámara confirma que ese
        #    "algo" es el azul (blob grande). Solo aquí se enciende la baliza.
        if facing and close_lidar and close_cam:
            return 0.0, 0.0, True

        # 2) Algo pegado al frente que NO es el azul (blob chico) → es un obstáculo:
        #    esquiva hacia el lado despejado, SIN encender.
        if close_lidar and not close_cam:
            return 0.0, self._avoid_dir() * self._avoid_w, False

        # 3) Obstáculo al frente y el azul de lado → gira hacia el azul.
        if (not facing) and front is not None and front < self._obstacle_m:
            return 0.0, w, False

        # 3) Aproximación: avanza escalado por cuán encarado va; frena cerca de obstáculos.
        face_scale = max(0.0, 1.0 - abs(err_deg) / 90.0)
        v = self._aggr_v * face_scale
        if front is not None and front < self._obstacle_m:
            v *= 0.4
        return v, w, False

    # ── LEDs ────────────────────────────────────────────────────────────────

    def _set_leds_blue(self, on: bool):
        """Enciende (azul) o apaga los LEDs, solo si cambia el estado."""
        if on == self._led_is_blue:
            return
        if not self._led_cli.service_is_ready():
            # El servicio del led_sim_node aún no está; reintenta el próximo tick.
            return
        req = LEDCommand.Request()
        if on:
            req.command = ('{"mode": "set_all", "rgb": [%d, %d, %d]}'
                           % (self._blue_rgb[0], self._blue_rgb[1], self._blue_rgb[2]))
        else:
            req.command = '{"mode": "clear"}'
        self._led_cli.call_async(req)      # fire-and-forget
        self._led_is_blue = on

    # ── Lazo de control ─────────────────────────────────────────────────────

    def _control_tick(self):
        targets   = self._fresh_targets()
        red_seen  = any(t.color.upper() == self._red   for t in targets)
        greens    = [t for t in targets if t.color.upper() == self._green]
        blues     = [t for t in targets if t.color.upper() == self._blue]
        green_tgt = max(greens, key=lambda t: t.area) if greens else None
        blue_tgt  = max(blues,  key=lambda t: t.area) if blues  else None

        # Prioridad:  ROJO (paro)  >  VERDE (repulsión)  >  AZUL (agregación)
        if red_seen:
            self._state = STOP_RED
            v, w = 0.0, 0.0
            self._set_leds_blue(False)

        elif green_tgt is not None:
            self._state = REPEL_GREEN
            v, w = self._flee_green(green_tgt)
            self._set_leds_blue(False)

        elif blue_tgt is not None:
            v, w, arrived = self._go_to_blue(blue_tgt)
            self._state = ARRIVED if arrived else AGGREGATE
            self._set_leds_blue(arrived)           # baliza azul solo al llegar

        else:
            self._state = EXPLORE
            v, w = self._explore()
            self._set_leds_blue(False)

        # Saturación final por seguridad
        v = max(-self._v_max, min(self._v_max, v))
        w = max(-self._w_max, min(self._w_max, w))

        msg = Twist()
        msg.linear.x  = round(float(v), 3)
        msg.angular.z = round(float(w), 3)
        self._pub_cmd.publish(msg)

        self.get_logger().debug(
            f'{self._state} v={v:.3f} w={w:.3f} '
            f'red={red_seen} green={green_tgt is not None} blue={blue_tgt is not None}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = ColorBehaviorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Frena el robot al salir.
        try:
            node._pub_cmd.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
