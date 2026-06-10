"""
BUG2 (sectores LiDAR + M-line) — portado de karifm v3
======================================================
Codigo que probo funcionar en el Puzzlebot fisico. Adaptado a la
arquitectura MiniChallenge6:
  - Subscribe a /current_goal (PoseStamped) del point_generator
  - Publica /goal_reached (Empty) para handshake
  - Publica /goal_marker (Marker) para RViz
  - Mantiene cycling interno de waypoints del YAML si NO recibe goals
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       DurabilityPolicy, qos_profile_sensor_data)
from geometry_msgs.msg import Pose2D, PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty, ColorRGBA
from visualization_msgs.msg import Marker

# ArUco detection para calibracion inicial
try:
    from puzzlebot_msgs.msg import ArucoDetectionArray
    ARUCO_DETECTIONS_AVAILABLE = True
except ImportError:
    ARUCO_DETECTIONS_AVAILABLE = False


def norm_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


DEFAULT_WAYPOINTS_X = [2.00]
DEFAULT_WAYPOINTS_Y = [-0.28]


class Bug2(Node):

    STATE_GO_TO_GOAL  = "go_to_goal"
    STATE_FOLLOW_WALL = "follow_wall"
    STATE_CALIBRATING = "calibrating"
    STATE_STOP        = "stop"

    def __init__(self):
        super().__init__('bug2')

        # === Parametros ===
        self.declare_parameter('controller_update_rate',           25.0)
        self.declare_parameter('distance_tolerance',                0.12)
        self.declare_parameter('following_walls_distance',          0.25)
        self.declare_parameter('front_stop_distance',               0.30)
        self.declare_parameter('lookahead_distance',                0.30)
        self.declare_parameter('p2p_v_Kp',                         0.8)
        self.declare_parameter('p2p_w_Kp',                         1.2)
        self.declare_parameter('fw_w_Kp',                          1.0)
        self.declare_parameter('fw_e_Kp',                          3.5)
        self.declare_parameter('fw_linear_speed',                   0.12)
        self.declare_parameter('fw_outer_corner_angular_speed',     1.2)
        self.declare_parameter('fw_outer_corner_linear_speed',      0.16)
        self.declare_parameter('v_max',                             0.12)
        self.declare_parameter('w_max',                             1.2)
        self.declare_parameter('side_open_angle',                   0.5236)
        self.declare_parameter('front_open_angle',                  0.4)
        self.declare_parameter('controller_type',                  'BUG2')
        # lidar_yaw_offset: 0.0 si el LiDAR esta APUNTANDO al frente (caso
        # nuestro), 3.14159 si esta girado 180 deg (cable hacia atras).
        self.declare_parameter('lidar_yaw_offset',                  0.0)
        self.declare_parameter('max_w_accel',                       4.0)
        self.declare_parameter('bug2_line_tol',                     0.15)
        self.declare_parameter('min_wall_follow_distance',          0.40)
        self.declare_parameter('waypoints_x',                       DEFAULT_WAYPOINTS_X)
        self.declare_parameter('waypoints_y',                       DEFAULT_WAYPOINTS_Y)
        self.declare_parameter('loop',                              False)
        # Calibracion inicial
        self.declare_parameter('calibration_enable',                True)
        self.declare_parameter('calibration_speed',                 0.04)
        self.declare_parameter('calibration_max_time',              10.0)
        self.declare_parameter('calibration_min_detections',        15)

        gp = self.get_parameter
        self.update_rate                   = float(gp('controller_update_rate').value)
        self.distance_tolerance            = float(gp('distance_tolerance').value)
        self.d_wall                        = float(gp('following_walls_distance').value)
        self.front_stop_distance           = float(gp('front_stop_distance').value)
        self.lookahead_distance            = float(gp('lookahead_distance').value)
        self.p2p_v_Kp                      = float(gp('p2p_v_Kp').value)
        self.p2p_w_Kp                      = float(gp('p2p_w_Kp').value)
        self.fw_w_Kp                       = float(gp('fw_w_Kp').value)
        self.fw_e_Kp                       = float(gp('fw_e_Kp').value)
        self.fw_linear_speed               = float(gp('fw_linear_speed').value)
        self.fw_outer_corner_angular_speed = float(gp('fw_outer_corner_angular_speed').value)
        self.fw_outer_corner_linear_speed  = float(gp('fw_outer_corner_linear_speed').value)
        self.v_max                         = float(gp('v_max').value)
        self.w_max                         = float(gp('w_max').value)
        self.side_open_angle               = float(gp('side_open_angle').value)
        self.front_open_angle              = float(gp('front_open_angle').value)
        self.controller_type               = str(gp('controller_type').value)
        self.lidar_yaw_offset              = float(gp('lidar_yaw_offset').value)
        self.max_w_accel                   = float(gp('max_w_accel').value)
        self.bug2_line_tol                 = float(gp('bug2_line_tol').value)
        self.min_wall_follow_distance      = float(gp('min_wall_follow_distance').value)
        self.loop                          = bool(gp('loop').value)
        self.calibration_enable            = bool(gp('calibration_enable').value)
        self.calibration_speed             = float(gp('calibration_speed').value)
        self.calibration_max_time          = float(gp('calibration_max_time').value)
        self.calibration_min_detections    = int(gp('calibration_min_detections').value)

        # Empty waypoints OK: aceptamos goals por /current_goal
        try:
            wx = list(gp('waypoints_x').value or [])
            wy = list(gp('waypoints_y').value or [])
        except Exception:
            wx, wy = [], []
        self.waypoints = [(float(x), float(y)) for x, y in zip(wx, wy)]
        self.use_internal_waypoints = len(self.waypoints) > 0
        self.goal_index = 0

        # === Estado ===
        self.robot_pose           = Pose2D()
        self.goal_pose            = Pose2D()
        self.have_goal            = False
        self.scan_ready           = False
        self.prev_w               = 0.0
        self.collision_time       = self.get_clock().now()
        self.min_front            = float('inf')
        self.min_side             = float('inf')
        self.min_opp_side         = float('inf')
        self.min_left             = float('inf')
        self.min_right            = float('inf')
        self.min_back_side        = float('inf')
        self.min_back_side_out    = float('inf')
        self.closest_object_angle = 0.0
        self.lidar_min_range      = 0.15
        self.state                = self.STATE_GO_TO_GOAL
        self.fw_direction         = 'fwccw'
        self.d_gtg_at_hit         = float('inf')
        self.line_A = self.line_B = self.line_C = 0.0

        # Calibracion inicial: cuando recibe goal nuevo, avanza despacio
        # hasta detectar el primer ArUco (~1 segundo de correcciones EKF)
        self.aruco_detected = False
        self.calibration_start_time = None
        self.aruco_detect_count = 0     # contador de ticks viendo ArUco

        # === QoS ===
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # === Pubs / Subs ===
        self.cmd_vel_publisher      = self.create_publisher(Twist, 'cmd_vel', 10)
        # /goal_reached como Empty para compatibilidad con point_generator MC6
        self.goal_reached_publisher = self.create_publisher(Empty, 'goal_reached', reliable_qos)
        self.goal_marker_pub        = self.create_publisher(Marker, 'goal_marker', latched_qos)

        self.create_subscription(Odometry,  'odom', self.odom_callback, qos_profile_sensor_data)
        self.create_subscription(LaserScan, 'scan', self.lidar_callback, qos_profile_sensor_data)
        # current_goal (PoseStamped) desde point_generator
        self.create_subscription(
            PoseStamped, 'current_goal', self.current_goal_callback, latched_qos)
        # Setpoint (Pose2D) por compatibilidad con karifm-style
        self.create_subscription(
            Pose2D, 'setpoint', self.setpoint_callback, qos_profile_sensor_data)
        # ArUco detections para fase de CALIBRACION (avanza lento hasta detectar)
        if ARUCO_DETECTIONS_AVAILABLE:
            self.create_subscription(
                ArucoDetectionArray, '/aruco_detections',
                self._aruco_detection_callback, 10)

        if self.use_internal_waypoints:
            self._set_goal_from_list()

        self.get_logger().info(
            f'{self.controller_type} iniciado. {len(self.waypoints)} waypoints, '
            f'loop={self.loop}, lidar_offset={self.lidar_yaw_offset:.4f} rad'
        )

        self.create_timer(1.0 / self.update_rate, self.controller_callback)

    # === Waypoints ===
    def _set_goal_from_list(self):
        if not self.waypoints or self.goal_index >= len(self.waypoints):
            return
        gx, gy = self.waypoints[self.goal_index]
        self.goal_pose = Pose2D(x=gx, y=gy, theta=0.0)
        self.have_goal = True
        self.state = self.STATE_GO_TO_GOAL
        self._compute_start_line()
        self._publish_goal_marker()
        self.get_logger().info(f'-> WP{self.goal_index}: ({gx:.2f}, {gy:.2f})')

    def _advance_waypoint(self):
        if self.goal_index + 1 >= len(self.waypoints):
            if self.loop:
                self.goal_index = 0
            else:
                self.get_logger().info('Ruta completa.')
                self.state = self.STATE_STOP
                return
        else:
            self.goal_index += 1
        self._set_goal_from_list()

    def setpoint_callback(self, msg: Pose2D):
        self.goal_pose = msg
        self.have_goal = True
        self.use_internal_waypoints = False  # cancel internal cycling
        self._compute_start_line()
        self._start_calibration_or_goto()
        self._publish_goal_marker()
        self.get_logger().info(
            f'Setpoint recibido: ({msg.x:.2f}, {msg.y:.2f})')

    def current_goal_callback(self, msg: PoseStamped):
        # Anti-stale-latched
        msg_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        now_stamp = self.get_clock().now().nanoseconds * 1e-9
        if msg_stamp > 0 and (now_stamp - msg_stamp) > 5.0:
            self.get_logger().warn(
                f'Goal descartado por stale ({now_stamp - msg_stamp:.1f}s viejo)')
            return
        gx = float(msg.pose.position.x)
        gy = float(msg.pose.position.y)
        # Solo aceptar si cambia
        if self.have_goal and abs(gx - self.goal_pose.x) < 0.01 \
                and abs(gy - self.goal_pose.y) < 0.01:
            return
        self.goal_pose = Pose2D(x=gx, y=gy, theta=0.0)
        self.have_goal = True
        self.use_internal_waypoints = False
        self._compute_start_line()
        self._start_calibration_or_goto()
        self._publish_goal_marker()
        self.get_logger().info(
            f'/current_goal recibido: ({gx:.2f}, {gy:.2f})')

    def _start_calibration_or_goto(self):
        """Comienza fase CALIBRATING (avanza despacio buscando ArUco) si
        habilitado. Si no, va directo a GO_TO_GOAL."""
        if self.calibration_enable:
            self.aruco_detected = False
            self.aruco_detect_count = 0
            self.calibration_start_time = self.get_clock().now()
            self.state = self.STATE_CALIBRATING
            self.get_logger().info(
                f'CALIBRATING: avanzando {self.calibration_speed:.2f} m/s '
                f'hasta detectar ArUco (max {self.calibration_max_time:.0f}s)'
            )
        else:
            self.state = self.STATE_GO_TO_GOAL

    # === Callbacks sensores ===
    def _aruco_detection_callback(self, msg):
        """Cuenta detecciones de ArUco para la fase de CALIBRACION."""
        if len(msg.detections) > 0:
            self.aruco_detect_count += 1
            if not self.aruco_detected:
                self.aruco_detected = True

    def odom_callback(self, msg: Odometry):
        self.robot_pose.x = msg.pose.pose.position.x
        self.robot_pose.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.robot_pose.theta = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def lidar_callback(self, msg: LaserScan):
        self.scan_ready = True
        # rplidar A1 publica 0..2pi sin importar el campo angle_min/max
        msg.angle_min = 0.0
        msg.angle_max = 2.0 * math.pi

        ranges = np.array(msg.ranges)
        ranges = np.where(
            np.isfinite(ranges) & (ranges > msg.range_min),
            ranges, np.inf)

        self.closest_object_angle = norm_angle(
            msg.angle_min + int(np.argmin(ranges)) * msg.angle_increment
            - self.lidar_yaw_offset)
        self.lidar_min_range = msg.range_min

        eff_offset = msg.angle_min - self.lidar_yaw_offset
        inc        = msg.angle_increment
        rmin       = msg.range_min

        self.min_front = self._region_min(
            ranges, -msg.angle_min, eff_offset,
            self.front_open_angle, self.front_open_angle, inc, rmin)

        side_c = self._side_center(self.fw_direction, msg.angle_min)
        self.min_side = self._region_min(
            ranges, side_c, eff_offset,
            self.side_open_angle, self.side_open_angle, inc, rmin)
        self.min_back_side = self._region_min(
            ranges, side_c, eff_offset,
            self.side_open_angle * 0.5, self.side_open_angle, inc, rmin)
        self.min_back_side_out = self._region_min_outside(
            ranges, side_c, eff_offset,
            self.side_open_angle * 0.5, self.side_open_angle, inc, rmin)
        # Lado OPUESTO al wall-follow (para detectar si la rueda se acerca a
        # un muro que no estamos siguiendo). Si fwccw -> wall en der (+90deg),
        # opp_center = -90deg (izq); si fwcw, al reves.
        opp_dir = 'fwcw' if self.fw_direction == 'fwccw' else 'fwccw'
        opp_c = self._side_center(opp_dir, msg.angle_min)
        self.min_opp_side = self._region_min(
            ranges, opp_c, eff_offset,
            self.side_open_angle, self.side_open_angle, inc, rmin)

        # NUEVO: distancias absolutas left/right (independientes de fw_direction)
        # left  = +90 deg (CCW desde el frente)
        # right = -90 deg = 270 deg (CW desde el frente)
        left_c = math.pi / 2.0 - msg.angle_min
        right_c = 3.0 * math.pi / 2.0 - msg.angle_min
        # Usamos sector mas ancho para detectar mejor las paredes y huecos
        self.min_left = self._region_min(
            ranges, left_c, eff_offset, 0.8, 0.8, inc, rmin)
        self.min_right = self._region_min(
            ranges, right_c, eff_offset, 0.8, 0.8, inc, rmin)

    # === Maquina de estados ===
    def controller_callback(self):
        if not self.scan_ready or not self.have_goal:
            return

        d_gtg     = self._dist_to_goal()
        theta_gtg = self._angle_to_goal()
        twist     = Twist()

        # === CAPA DE SEGURIDAD: SIGUE-PARED HASTA CAMINO LIBRE ===
        # Si hay riesgo de chocar (pared cerca de costado o frente),
        # ignora la nav normal del bug2 y SIGUE LA PARED hasta encontrar
        # espacio libre. Solo entonces regresa control a la nav normal.
        SAFE_SIDE = 0.20      # umbral lateral peligroso (rueda ~8cm fuera)
        SAFE_FRONT = 0.25     # umbral frontal peligroso
        TARGET_DIST = 0.32    # distancia objetivo durante recovery
        CLEAR_THRESH = 0.35   # ambos lados >= esto + front clear = camino libre

        front_close = self.min_front < SAFE_FRONT
        left_close = self.min_left < SAFE_SIDE
        right_close = self.min_right < SAFE_SIDE

        # Sticky state: una vez en SAFETY, queda hasta liberacion clara
        if not hasattr(self, '_safety_active'):
            self._safety_active = False
            self._safety_wall = None  # 'left' o 'right'

        if front_close or left_close or right_close:
            self._safety_active = True
            # Determinar de que lado seguir
            if self._safety_wall is None:
                if left_close and not right_close:
                    self._safety_wall = 'left'
                elif right_close and not left_close:
                    self._safety_wall = 'right'
                else:
                    # ambos lados cerca (corredor) o solo frente
                    # elegir el lado MAS cercano para seguirlo
                    self._safety_wall = ('left' if self.min_left < self.min_right
                                         else 'right')

        # Si esta activo el modo seguridad, ejecutarlo
        if self._safety_active:
            # Condicion para liberar
            free = (self.min_front > CLEAR_THRESH + 0.1 and
                    self.min_left > CLEAR_THRESH and
                    self.min_right > CLEAR_THRESH)
            if free:
                self._safety_active = False
                self._safety_wall = None
                self.get_logger().info(
                    f'SAFETY -> nav normal (front={self.min_front:.2f} '
                    f'L={self.min_left:.2f} R={self.min_right:.2f})')
            else:
                v_safe, w_safe = self._safety_wall_follow(TARGET_DIST)
                twist.linear.x = v_safe
                twist.angular.z = w_safe
                self.cmd_vel_publisher.publish(twist)
                self.get_logger().warn(
                    f'SAFETY ({self._safety_wall}): front={self.min_front:.2f} '
                    f'L={self.min_left:.2f} R={self.min_right:.2f} '
                    f'v={v_safe:+.2f} w={w_safe:+.2f}',
                    throttle_duration_sec=0.5)
                return

        if self.state == self.STATE_STOP:
            self.cmd_vel_publisher.publish(Twist())
            return

        elif self.state == self.STATE_CALIBRATING:
            # Avanzar despacio hasta detectar ArUco N veces (~0.5s de
            # correcciones EKF) o hasta timeout. Si hay pared en frente,
            # parar y proceder a GO_TO_GOAL (eventualmente entrara wall-follow).
            elapsed = (self.get_clock().now() -
                       self.calibration_start_time).nanoseconds * 1e-9

            # Salida 1: tenemos suficientes detecciones de ArUco
            if self.aruco_detect_count >= self.calibration_min_detections:
                self.get_logger().info(
                    f'CALIBRATING -> GO_TO_GOAL '
                    f'(ArUco visto {self.aruco_detect_count}x en {elapsed:.1f}s)'
                )
                self._compute_start_line()  # M-line con pose corregida
                self.state = self.STATE_GO_TO_GOAL
                return

            # Salida 2: timeout
            if elapsed > self.calibration_max_time:
                self.get_logger().warn(
                    f'CALIBRATING -> GO_TO_GOAL (timeout sin ArUco; pose'
                    f' inicial sin corregir)'
                )
                self.state = self.STATE_GO_TO_GOAL
                return

            # Salida 3: pared al frente (no podemos avanzar mas)
            if self.min_front < self.front_stop_distance:
                self.get_logger().warn(
                    f'CALIBRATING -> GO_TO_GOAL (pared al frente, '
                    f'ArUco visto {self.aruco_detect_count}x)'
                )
                self.state = self.STATE_GO_TO_GOAL
                return

            # Avanzar despacio en linea recta
            twist.linear.x = self.calibration_speed
            twist.angular.z = 0.0
            self.cmd_vel_publisher.publish(twist)
            self.get_logger().info(
                f'[calibrating] aruco_count={self.aruco_detect_count} '
                f'elapsed={elapsed:.1f}s front={self.min_front:.2f}',
                throttle_duration_sec=1.0)
            return

        elif self.state == self.STATE_GO_TO_GOAL:
            if d_gtg < self.distance_tolerance:
                # === VERIFICACION FISICA antes de declarar alcanzado ===
                # El robot SOLO puede declarar goal alcanzado si fisicamente
                # esta en un area abierta (no contra pared). Esto evita
                # falsos "alcanzados" cuando la EKF derivo y el robot esta
                # contra un muro pensando que llego.
                front_clear = self.min_front > 0.25
                space_clear = (self.min_left > 0.15 and
                               self.min_right > 0.15)
                if front_clear and space_clear:
                    self.cmd_vel_publisher.publish(Twist())
                    self.goal_reached_publisher.publish(Empty())
                    self.get_logger().info(
                        f'WP{self.goal_index} alcanzado '
                        f'(front={self.min_front:.2f} L={self.min_left:.2f} '
                        f'R={self.min_right:.2f}).')
                    self._publish_goal_marker()
                    if self.use_internal_waypoints:
                        self._advance_waypoint()
                    else:
                        self.state = self.STATE_STOP
                    return
                else:
                    self.get_logger().warn(
                        f'EKF dice alcanzado (d={d_gtg:.2f}) pero no hay '
                        f'espacio fisico (front={self.min_front:.2f} '
                        f'L={self.min_left:.2f} R={self.min_right:.2f}). '
                        f'EKF DERIVO. Sigo navegando.',
                        throttle_duration_sec=2.0)
                    # No retornar — seguir con la logica del controlador

            v, w = self._go_to_goal_control(d_gtg, theta_gtg)

            # HIT_POINT a wall_follow DESHABILITADO: el nuevo _go_to_goal_control
            # maneja paredes con centrado + busqueda de huecos. Solo entra a
            # wall_follow si el robot esta REALMENTE atorado (no progresa por
            # mucho tiempo).
            # if self.min_front < self.front_stop_distance:
            if False:
                self.get_logger().info(f'Hit point -> {self.controller_type}')
                self.d_gtg_at_hit = d_gtg
                self.fw_direction = self._choose_fw_direction()
                if self.controller_type == 'BUG2':
                    self._compute_start_line()
                self.state = self.STATE_FOLLOW_WALL
                # Reset stuck detector
                self._wf_stuck_start = self.get_clock().now()
                self._wf_stuck_d = d_gtg
                v, w = self._follow_wall_control()

        else:  # FOLLOW_WALL
            # Detector de STUCK en wall-follow: si llevamos > 10s en wall_follow
            # con la misma distancia al goal, cambiar direccion de wall-follow
            if not hasattr(self, '_wf_stuck_start'):
                self._wf_stuck_start = self.get_clock().now()
                self._wf_stuck_d = d_gtg
            stuck_elapsed = (self.get_clock().now() -
                             self._wf_stuck_start).nanoseconds * 1e-9
            # STUCK = NO progresamos al goal (no avanzar hacia el goal en 15s)
            # NOTA: solo dispara si NO bajamos al menos 5cm de d en el periodo
            if stuck_elapsed > 15.0:
                if d_gtg > self._wf_stuck_d - 0.05:
                    self.fw_direction = 'fwcw' if self.fw_direction == 'fwccw' \
                        else 'fwccw'
                    self.get_logger().warn(
                        f'STUCK en wall_follow ({stuck_elapsed:.1f}s sin '
                        f'progreso, d {self._wf_stuck_d:.2f}->{d_gtg:.2f}) '
                        f'-> cambio direccion a {self.fw_direction}'
                    )
                self._wf_stuck_start = self.get_clock().now()
                self._wf_stuck_d = d_gtg

            if (self.get_clock().now() - self.collision_time <
                    rclpy.duration.Duration(seconds=0.75)):
                v = -0.08
                w = (-1 if self.fw_direction == 'fwccw' else 1) * self.w_max
            else:
                v, w = self._follow_wall_control()
                if self.min_front < self.lidar_min_range + 0.01:
                    self.collision_time = self.get_clock().now()

            if self.controller_type == 'BUG0':
                if self._bug0_leave_condition(d_gtg, theta_gtg):
                    self.get_logger().info('Bug0: clear shot -> go_to_goal')
                    self.state = self.STATE_GO_TO_GOAL
            elif self.controller_type == 'BUG2':
                if self._bug2_leave_condition(d_gtg):
                    self.get_logger().info('Bug2: m-line + progreso -> go_to_goal')
                    self.state = self.STATE_GO_TO_GOAL

        v = float(np.clip(v, -self.v_max, self.v_max))
        w = float(np.clip(w, -self.w_max, self.w_max))
        if self.max_w_accel > 0.0:
            dw_max = self.max_w_accel / self.update_rate
            w = self.prev_w + float(np.clip(w - self.prev_w, -dw_max, dw_max))
        self.prev_w = w

        twist.linear.x  = v
        twist.angular.z = w
        self.cmd_vel_publisher.publish(twist)

        self.get_logger().info(
            f'[{self.state}] WP{self.goal_index} d={d_gtg:.2f} '
            f'front={self.min_front:.2f} L={self.min_left:.2f} R={self.min_right:.2f} '
            f'v={v:+.2f} w={w:+.2f}',
            throttle_duration_sec=1.0)

    # === Controladores ===
    def _safety_wall_follow(self, target_dist):
        """Sigue la pared mas cercana (segun self._safety_wall) hasta
        encontrar camino libre. Mantiene distancia=target_dist de la pared.

        Si hay pared al frente, rota AWAY (hacia el lado opuesto al wall
        que sigue) para escapar del corner.
        """
        # Si pared al frente, rotar para escapar (no avanzar)
        if self.min_front < 0.20:
            # Rotar hacia el lado contrario del wall que sigue
            # (si sigue left wall, rotar a la der; si sigue right, rotar a izq)
            w = -0.6 if self._safety_wall == 'left' else 0.6
            return 0.0, w
        if self.min_front < 0.35:
            # Algo de pared al frente: avanza muy lento mientras rota
            w_rotate = -0.4 if self._safety_wall == 'left' else 0.4
            return 0.02, w_rotate

        # Wall-follow proporcional: mantener distancia=target_dist
        if self._safety_wall == 'left':
            error = target_dist - self.min_left
            # error > 0 -> muy cerca, rotar DERECHA (negativo)
            # error < 0 -> muy lejos, rotar IZQUIERDA (positivo)
            w = -error * 3.0
        else:  # right
            error = target_dist - self.min_right
            # error > 0 -> muy cerca, rotar IZQUIERDA (positivo)
            w = error * 3.0
        w = max(-0.6, min(0.6, w))
        v = 0.05   # 5 cm/s lento
        return v, w

    def _go_to_goal_control(self, d_gtg, theta_gtg):
        """Control nuevo: navegacion CENTRADA en corredores + busqueda de
        huecos hacia el goal. No usa wall-following (no se pega a paredes).

        Estrategia:
          - Si ambos lados tienen pared cerca (corredor) -> SE CENTRA
          - Si un lado abre (hueco) Y goal va por ese lado -> entra al hueco
          - Si no hay razon para rotar -> avanza recto
          - Si pared al frente -> rota al lado con mas espacio (que apunte
            hacia el goal)
        """
        e_theta = norm_angle(theta_gtg - self.robot_pose.theta)
        abs_e = abs(e_theta)
        # Goal "a la izq" si e_theta > 0, "a la der" si e_theta < 0.
        goal_dir_left = e_theta > 0

        left = self.min_left
        right = self.min_right

        # Constantes
        CORRIDOR_THRESH = 0.55     # ambos < esto -> corredor angosto
        OPENING_THRESH = 1.0       # un lado > esto -> hueco / pasillo lateral
        ASYMMETRY_THRESH = 0.40    # diferencia para considerar "un lado mas abierto"

        # === 1. Velocidad base hacia el goal ===
        v = min(self.p2p_v_Kp * d_gtg, self.v_max)
        if abs_e > math.pi / 2:
            v = 0.0
        elif abs_e > math.pi / 4:
            v = v * 0.4

        # Frenado por pared al frente
        if self.min_front < self.front_stop_distance * 2:
            ratio = (self.min_front - self.front_stop_distance) / \
                    self.front_stop_distance
            ratio = max(0.0, min(1.0, ratio))
            v = v * ratio

        # === 2. Angular: prioridades en orden ===
        in_corridor = (left < CORRIDOR_THRESH and right < CORRIDOR_THRESH)
        left_open = (left > OPENING_THRESH)
        right_open = (right > OPENING_THRESH)

        if self.min_front < self.front_stop_distance:
            # Pared al frente: girar al lado MAS ABIERTO que apunte al goal
            if left > right:
                w = +0.6   # rotar izquierda
            else:
                w = -0.6   # rotar derecha
        elif in_corridor:
            # CORREDOR: centrar entre paredes
            # w positivo = rotar izquierda (hacia el lado izquierdo)
            # Si left > right (mas espacio izq), debe estar mas centrado a der
            # asi que w = pequeno hacia izquierda
            centering = (left - right) * 0.8
            # Bias suave hacia el goal si esta cerca de la direccion actual
            goal_bias = self.p2p_w_Kp * e_theta * 0.3
            w = centering + goal_bias
        elif (left_open and goal_dir_left) or (right_open and not goal_dir_left):
            # HUECO al lado del goal: rotar hacia ese hueco
            w = self.p2p_w_Kp * e_theta
        elif left_open or right_open:
            # Hueco pero NO en direccion al goal: ignorar, seguir recto
            # con pequeno bias al goal
            w = self.p2p_w_Kp * e_theta * 0.5
        else:
            # ESPACIO ABIERTO sin paredes cercanas: ir al goal normal
            w = self.p2p_w_Kp * e_theta

        # Cap w
        w = max(-self.w_max, min(self.w_max, w))
        return v, w

    def _follow_wall_control(self):
        theta_ao = norm_angle(self.closest_object_angle + math.pi)
        theta_fw = norm_angle(theta_ao + (math.pi / 2 if self.fw_direction == 'fwccw' else -math.pi / 2))

        ed = (self.min_side - self.d_wall) if self.fw_direction == 'fwccw' else (self.d_wall - self.min_side)
        w_theta = self.fw_w_Kp * theta_fw
        max_w_theta = self.w_max * 0.5
        w_theta = max(-max_w_theta, min(max_w_theta, w_theta))
        w = w_theta + self.fw_e_Kp * ed
        v = self.fw_linear_speed

        # NUEVA logica: graduacion suave en vez de stop+rotate brusco
        # Cuanto mas cerca la pared, mas lento avanza pero NO se detiene.
        # Esto permite navegar esquinas estrechas sin quedarse oscilando.
        if self.min_front < self.front_stop_distance * 2:
            # Escala v entre 100% (front=2*stop) y 20% (front=stop)
            ratio = (self.min_front - self.front_stop_distance) / \
                    self.front_stop_distance
            ratio = max(0.2, min(1.0, ratio))
            v = self.fw_linear_speed * ratio
            # Aumenta la rotacion al acercarse a pared frontal
            w_corner = (-1.0 if self.fw_direction == 'fwccw' else 1.0) * \
                       self.w_max * 0.6 * (1.0 - ratio)
            w = w + w_corner
            self._outer_corner_count = 0
        elif (self.min_back_side < self.lookahead_distance and
              self.min_back_side_out > self.lookahead_distance):
            # OUTER CORNER con debounce
            self._outer_corner_count = getattr(self, '_outer_corner_count', 0) + 1
            if self._outer_corner_count >= 5:
                v = self.fw_outer_corner_linear_speed
                w = (1.0 if self.fw_direction == 'fwccw' else -1.0) * \
                    self.fw_outer_corner_angular_speed
        else:
            self._outer_corner_count = 0

        return v, w

    # === Condiciones de salida ===
    def _bug0_leave_condition(self, d_gtg, theta_gtg):
        theta_ao   = norm_angle(self.closest_object_angle + math.pi)
        angle_diff = abs(norm_angle(theta_ao - theta_gtg))
        progress   = d_gtg < (self.d_gtg_at_hit - self.distance_tolerance)
        clear_shot = angle_diff < math.pi / 2
        return progress and clear_shot

    def _bug2_leave_condition(self, d_gtg):
        on_line  = self._distance_to_start_line() < self.bug2_line_tol
        progress = d_gtg < (self.d_gtg_at_hit - self.min_wall_follow_distance)
        front_ok = self.min_front > self.front_stop_distance + 0.05
        return on_line and progress and front_ok

    # === Helpers ===
    def _dist_to_goal(self):
        return math.hypot(self.goal_pose.x - self.robot_pose.x,
                          self.goal_pose.y - self.robot_pose.y)

    def _angle_to_goal(self):
        return math.atan2(self.goal_pose.y - self.robot_pose.y,
                          self.goal_pose.x - self.robot_pose.x)

    def _choose_fw_direction(self):
        theta_ao   = norm_angle(self.closest_object_angle + math.pi)
        theta_fwc  = norm_angle(theta_ao - math.pi / 2)
        theta_fwcc = norm_angle(theta_ao + math.pi / 2)
        direction  = 'fwcw' if abs(theta_fwc) <= abs(theta_fwcc) else 'fwccw'
        self.get_logger().info(f'fw_direction: {direction}')
        return direction

    def _compute_start_line(self):
        dx = self.goal_pose.x - self.robot_pose.x
        dy = self.goal_pose.y - self.robot_pose.y
        if abs(dx) < 1e-6:
            self.line_A, self.line_B, self.line_C = 1.0, 0.0, -self.robot_pose.x
        else:
            m = dy / dx
            self.line_A = m
            self.line_B = -1.0
            self.line_C = self.robot_pose.y - m * self.robot_pose.x

    def _distance_to_start_line(self):
        num = abs(self.line_A * self.robot_pose.x +
                  self.line_B * self.robot_pose.y + self.line_C)
        den = math.sqrt(self.line_A ** 2 + self.line_B ** 2)
        return num / den if den > 1e-9 else float('inf')

    def _side_center(self, direction, angle_min):
        return (math.pi / 2 if direction == 'fwccw' else 3 * math.pi / 2) - angle_min

    @staticmethod
    def _norm_shift(a):
        a = math.atan2(math.sin(a), math.cos(a))
        return a if a >= 0 else 2.0 * math.pi + a

    def _region_min(self, r, center, offset, front_open, back_open, inc, rmin):
        if center < math.pi:
            a0 = self._norm_shift(center - offset - front_open)
            a1 = self._norm_shift(center - offset + back_open)
        else:
            a0 = self._norm_shift(center - offset - back_open)
            a1 = self._norm_shift(center - offset + front_open)
        return self._min_idx(r, int(a0 / inc), int(a1 / inc), rmin)

    def _region_min_outside(self, r, center, offset, front_open, back_open, inc, rmin):
        a0 = self._norm_shift(center - offset - front_open)
        a1 = self._norm_shift(center - offset + back_open)
        return self._min_idx_outside(r, int(a0 / inc), int(a1 / inc), rmin)

    @staticmethod
    def _min_idx(r, i0, i1, rmin):
        vals = np.concatenate((r[i0:], r[:i1])) if i0 > i1 else r[i0:i1]
        return float('inf') if vals.size == 0 else max(float(np.min(vals)), rmin)

    @staticmethod
    def _min_idx_outside(r, i0, i1, rmin):
        vals = r[i1:i0] if i0 > i1 else np.concatenate((r[i1:], r[:i0]))
        return float('inf') if vals.size == 0 else max(float(np.min(vals)), rmin)

    # === Marker RViz ===
    def _publish_goal_marker(self):
        if not self.have_goal:
            return
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'bug2_goal'
        m.id = 0
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose.position.x = float(self.goal_pose.x)
        m.pose.position.y = float(self.goal_pose.y)
        m.pose.position.z = 0.05
        m.pose.orientation.w = 1.0
        m.scale.x = 0.15
        m.scale.y = 0.15
        m.scale.z = 0.10
        m.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)
        self.goal_marker_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = Bug2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            for _ in range(5):
                node.cmd_vel_publisher.publish(Twist())
                rclpy.spin_once(node, timeout_sec=0.05)
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()
