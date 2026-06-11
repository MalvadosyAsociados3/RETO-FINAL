"""
MINI CHALLENGE 6 - BUG 0 NAVIGATION ALGORITHM

Suscribe:
  /odom         (nav_msgs/Odometry)       - pose estimada (localisation con covarianza)
  /scan         (sensor_msgs/LaserScan)   - LiDAR (Gazebo de robotec_sim_ws)
  /current_goal (geometry_msgs/PoseStamped) - waypoint actual (point_generator)
  /aruco_correction (std_msgs/Empty)      - notificacion de correccion ArUco aceptada

Publica:
  /cmd_vel      (geometry_msgs/Twist)     - comando al simulador
  /goal_reached (std_msgs/Empty)          - handshake con point_generator
  /bug0_state_text (visualization_msgs/Marker) - texto del estado en RViz
  /bug0_goal_line  (visualization_msgs/Marker) - linea robot->goal en RViz
  /bug0_path_trail (visualization_msgs/Marker) - trail del path recorrido

State machine:
  GO_TO_GOAL:
    - Apuntar al objetivo + avanzar con controlador P (NumPy puro).
    - Si hay obstaculo en el cono frontal (+-15 deg) a menos de 0.5 m -> FOLLOW_WALL.
  FOLLOW_WALL:
    - Mantener la pared a la IZQUIERDA del robot a ~0.35 m (P sobre el error de distancia).
    - Si el cono hacia el goal queda libre Y nos hemos acercado al goal vs el hit_point,
      regresar a GO_TO_GOAL.
  ARUCO_PAUSE:
    - ArUco correccion aceptada: detenerse 1s, luego avanzar lento.

Wall following: pared a la izquierda (obstaculo a mano izquierda del robot).
Restriccion del challenge: solo NumPy + libreria estandar de Python.
"""

import math
import numpy as np
import rclpy
import rclpy.duration
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       DurabilityPolicy, qos_profile_sensor_data)
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty, ColorRGBA
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration


def quat_to_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def normalize_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class Bug0(Node):

    STATE_GO_TO_GOAL = 0
    STATE_FOLLOW_WALL = 1
    STATE_GOAL_REACHED = 2
    STATE_ARUCO_PAUSE = 3

    STATE_NAMES = {
        STATE_GO_TO_GOAL: 'GO_TO_GOAL',
        STATE_FOLLOW_WALL: 'FOLLOW_WALL',
        STATE_GOAL_REACHED: 'GOAL_REACHED',
        STATE_ARUCO_PAUSE: 'ARUCO_PAUSE',
    }

    STATE_COLORS = {
        STATE_GO_TO_GOAL: ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0),    # verde
        STATE_FOLLOW_WALL: ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),   # rojo
        STATE_GOAL_REACHED: ColorRGBA(r=0.0, g=0.5, b=1.0, a=1.0),  # azul
        STATE_ARUCO_PAUSE: ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0),   # amarillo
    }

    def __init__(self):
        super().__init__('bug0')

        # --- Parametros ---
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('goal_tolerance', 0.15)
        self.declare_parameter('obstacle_distance', 0.5)
        self.declare_parameter('forward_cone_deg', 30.0)
        self.declare_parameter('wall_target_distance', 0.35)
        self.declare_parameter('wall_sector_min_deg', 60.0)
        self.declare_parameter('wall_sector_max_deg', 120.0)
        self.declare_parameter('clear_path_cone_deg', 50.0)
        self.declare_parameter('clear_path_distance', 0.8)
        self.declare_parameter('k_linear', 0.5)
        self.declare_parameter('k_angular', 1.5)
        self.declare_parameter('max_linear', 0.18)
        self.declare_parameter('max_angular', 1.0)
        self.declare_parameter('turn_max_angular', 1.5)  # angular alto para giros de 90
        self.declare_parameter('align_threshold_deg', 25.0)
        self.declare_parameter('wall_k', 2.5)
        self.declare_parameter('wall_linear', 0.10)
        self.declare_parameter('wall_progress_threshold', 0.10)
        self.declare_parameter('min_follow_wall_time', 1.5)  # segundos minimos en FOLLOW_WALL
        # ArUco pause
        self.declare_parameter('aruco_pause_duration', 2.0)      # segundos de pausa total
        self.declare_parameter('aruco_cooldown', 5.0)            # segundos antes de permitir otra pausa
        # Path trail
        self.declare_parameter('trail_sample_dist', 0.03)        # metros entre puntos del trail

        rate = float(self.get_parameter('control_rate').value)
        self.goal_tol = float(self.get_parameter('goal_tolerance').value)
        self.obs_dist = float(self.get_parameter('obstacle_distance').value)
        self.front_cone = math.radians(float(self.get_parameter('forward_cone_deg').value))
        self.wall_target = float(self.get_parameter('wall_target_distance').value)
        self.wall_sec_min = math.radians(float(self.get_parameter('wall_sector_min_deg').value))
        self.wall_sec_max = math.radians(float(self.get_parameter('wall_sector_max_deg').value))
        self.clear_cone = math.radians(float(self.get_parameter('clear_path_cone_deg').value))
        self.clear_dist = float(self.get_parameter('clear_path_distance').value)
        self.kv = float(self.get_parameter('k_linear').value)
        self.kw = float(self.get_parameter('k_angular').value)
        self.vmax = float(self.get_parameter('max_linear').value)
        self.wmax = float(self.get_parameter('max_angular').value)
        self.turn_wmax = float(self.get_parameter('turn_max_angular').value)
        self.align_thr = math.radians(float(self.get_parameter('align_threshold_deg').value))
        self.wall_k = float(self.get_parameter('wall_k').value)
        self.wall_v = float(self.get_parameter('wall_linear').value)
        self.wall_progress = float(self.get_parameter('wall_progress_threshold').value)
        self.min_fw_time = float(self.get_parameter('min_follow_wall_time').value)
        self.aruco_pause_dur = float(self.get_parameter('aruco_pause_duration').value)
        self.aruco_cooldown_dur = float(self.get_parameter('aruco_cooldown').value)
        self.trail_sample_dist = float(self.get_parameter('trail_sample_dist').value)

        # --- Estado del robot ---
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.have_odom = False

        # Scan
        self.scan_ranges = None
        self.scan_angle_min = 0.0
        self.scan_angle_inc = 0.0
        self.scan_range_max = 10.0

        # Goal
        self.goal_x = None
        self.goal_y = None

        # State machine
        self.state = self.STATE_GO_TO_GOAL
        self.last_state = None
        self.goal_reached_published = False
        self.wall_follow_start_dist_to_goal = None
        self._follow_wall_start_time = None
        self._tick_count = 0
        self._log_interval = int(rate * 2)  # log cada ~2 seg

        # ArUco pause state
        self._aruco_pause_start = None   # timestamp de inicio de pausa
        self._state_before_pause = self.STATE_GO_TO_GOAL
        self._aruco_cooldown_until = None  # ignora correcciones hasta este timestamp

        # Anti-spin: detectar si GO_TO_GOAL solo gira sin avanzar
        self._gtg_align_start = None     # timestamp cuando empezo a girar en GTG

        # Path trail
        self._trail_points = []    # list of (x, y, state)
        self._last_trail_x = None
        self._last_trail_y = None

        # --- QoS ---
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

        # --- Subs ---
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_cb, reliable_qos)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_cb, qos_profile_sensor_data)
        self.goal_sub = self.create_subscription(
            PoseStamped, 'current_goal', self.goal_cb, latched_qos)
        self.aruco_sub = self.create_subscription(
            Empty, 'aruco_correction', self.aruco_correction_cb, reliable_qos)

        # --- Pubs ---
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.reached_pub = self.create_publisher(Empty, 'goal_reached', reliable_qos)
        self.goal_marker_pub = self.create_publisher(Marker, 'goal_marker', 10)
        self.state_text_pub = self.create_publisher(Marker, 'bug0_state_text', 10)
        self.goal_line_pub = self.create_publisher(Marker, 'bug0_goal_line', 10)
        self.path_trail_pub = self.create_publisher(Marker, 'bug0_path_trail', 10)

        # --- Timer ---
        self.timer = self.create_timer(1.0 / rate, self.tick)

        self.get_logger().info(
            f'Bug 0 iniciado: obs_dist={self.obs_dist} m, '
            f'cono_frontal={math.degrees(self.front_cone):.0f} deg, '
            f'wall_target={self.wall_target} m (IZQUIERDA), '
            f'vmax={self.vmax}, wmax={self.wmax}, turn_wmax={self.turn_wmax}, '
            f'aruco_pause={self.aruco_pause_dur}s, aruco_cooldown={self.aruco_cooldown_dur}s, '
            f'ctrl_rate={rate} Hz'
        )

    # ----------------------------------------------------------- Callbacks

    def odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.theta = quat_to_yaw(msg.pose.pose.orientation)
        self.have_odom = True

    def scan_cb(self, msg: LaserScan):
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        bad = ~np.isfinite(ranges)
        ranges = np.where(bad, msg.range_max, ranges)
        self.scan_ranges = ranges
        self.scan_angle_min = float(msg.angle_min)
        self.scan_angle_inc = float(msg.angle_increment)
        self.scan_range_max = float(msg.range_max)

    def goal_cb(self, msg: PoseStamped):
        new_x = float(msg.pose.position.x)
        new_y = float(msg.pose.position.y)
        if (self.goal_x is None or
                abs(new_x - self.goal_x) > 0.01 or
                abs(new_y - self.goal_y) > 0.01):
            self.goal_x = new_x
            self.goal_y = new_y
            self.state = self.STATE_GO_TO_GOAL
            self.goal_reached_published = False
            self.wall_follow_start_dist_to_goal = None
            self._aruco_pause_start = None
            self._publish_goal_marker()
            self.get_logger().info(
                f'Bug 0: nuevo goal ({new_x:.2f}, {new_y:.2f})')

    def aruco_correction_cb(self, _msg: Empty):
        """ArUco correccion aceptada por el EKF -> pausar para estabilizar.
        En FOLLOW_WALL solo pausa si el frente esta libre (tramo recto);
        si hay pared al frente (girando en esquina), no pausar."""
        if self.state == self.STATE_GOAL_REACHED:
            return
        # En FOLLOW_WALL: pausar solo en tramo recto (frente libre)
        if self.state == self.STATE_FOLLOW_WALL:
            if self.scan_ranges is not None and self.front_min() < self.obs_dist:
                return  # girando en esquina, no pausar
        # Ignorar si ya estamos en pausa (evita resetear el timer)
        if self.state == self.STATE_ARUCO_PAUSE:
            return
        # Ignorar si estamos en cooldown post-pausa
        now = self.get_clock().now()
        if self._aruco_cooldown_until is not None:
            if now < self._aruco_cooldown_until:
                return
            self._aruco_cooldown_until = None
        self._state_before_pause = self.state
        self._set_state(self.STATE_ARUCO_PAUSE)
        self._aruco_pause_start = now
        self.get_logger().info(
            f'Bug 0: ArUco correccion -> ARUCO_PAUSE ({self.aruco_pause_dur}s)')

    def _publish_goal_marker(self):
        if self.goal_x is None:
            return
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'bug0_goal'
        m.id = 0
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose.position.x = float(self.goal_x)
        m.pose.position.y = float(self.goal_y)
        m.pose.position.z = 0.05
        m.pose.orientation.w = 1.0
        m.scale.x = 0.15
        m.scale.y = 0.15
        m.scale.z = 0.10
        if self.goal_reached_published:
            m.color = ColorRGBA(r=0.0, g=0.5, b=1.0, a=0.8)
        else:
            m.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)
        self.goal_marker_pub.publish(m)

    # --------------------------------------------------------- Visualization

    def _publish_state_text(self):
        """Publica un texto flotante sobre el robot con el estado actual."""
        m = Marker()
        m.header.frame_id = 'base_footprint'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'bug0_state'
        m.id = 0
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.z = 0.35  # encima del robot
        m.pose.orientation.w = 1.0
        m.scale.z = 0.12  # tamano del texto
        m.color = self.STATE_COLORS.get(self.state, ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0))
        m.text = self.STATE_NAMES.get(self.state, '???')
        m.lifetime = Duration(sec=0, nanosec=500_000_000)  # 0.5s
        self.state_text_pub.publish(m)

    def _publish_goal_line(self):
        """Linea desde el robot al goal en frame map (fija en el mapa)."""
        if self.goal_x is None or not self.have_odom:
            return
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'bug0_goal_line'
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.02  # grosor de la linea

        # Color segun estado
        if self.state == self.STATE_GO_TO_GOAL:
            m.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.7)  # verde
        elif self.state == self.STATE_FOLLOW_WALL:
            m.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.7)  # naranja
        else:
            m.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.7)  # amarillo

        p1 = Point(x=float(self.x), y=float(self.y), z=0.02)
        p2 = Point(x=float(self.goal_x), y=float(self.goal_y), z=0.02)
        m.points = [p1, p2]
        m.lifetime = Duration(sec=0, nanosec=500_000_000)
        self.goal_line_pub.publish(m)

    def _update_trail(self):
        """Agrega punto al trail si el robot se movio suficiente."""
        if not self.have_odom:
            return
        if self._last_trail_x is None:
            self._last_trail_x = self.x
            self._last_trail_y = self.y
            self._trail_points.append((self.x, self.y, self.state))
            return
        dx = self.x - self._last_trail_x
        dy = self.y - self._last_trail_y
        if math.hypot(dx, dy) >= self.trail_sample_dist:
            self._trail_points.append((self.x, self.y, self.state))
            self._last_trail_x = self.x
            self._last_trail_y = self.y
            # Limitar a 5000 puntos maximo
            if len(self._trail_points) > 5000:
                self._trail_points = self._trail_points[-4000:]

    def _publish_path_trail(self):
        """Publica el trail del path recorrido, coloreado por estado."""
        if len(self._trail_points) < 2:
            return
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'bug0_trail'
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.015  # grosor
        m.pose.orientation.w = 1.0

        for (px, py, st) in self._trail_points:
            m.points.append(Point(x=float(px), y=float(py), z=0.01))
            m.colors.append(self.STATE_COLORS.get(st, ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)))

        self.path_trail_pub.publish(m)

    # ---------------------------------------------------- LiDAR utilities

    def _sector_min(self, angle_lo: float, angle_hi: float) -> float:
        """Devuelve el rango minimo en el sector angular [angle_lo, angle_hi] rad."""
        if self.scan_ranges is None or self.scan_angle_inc == 0.0:
            return float('inf')
        n = len(self.scan_ranges)
        idx_lo = int(round((angle_lo - self.scan_angle_min) / self.scan_angle_inc))
        idx_hi = int(round((angle_hi - self.scan_angle_min) / self.scan_angle_inc))
        idx_lo = max(0, min(n - 1, idx_lo))
        idx_hi = max(0, min(n - 1, idx_hi))
        if idx_lo > idx_hi:
            idx_lo, idx_hi = idx_hi, idx_lo
        sector = self.scan_ranges[idx_lo:idx_hi + 1]
        if sector.size == 0:
            return float('inf')
        return float(np.min(sector))

    def front_min(self) -> float:
        half = self.front_cone / 2.0
        return self._sector_min(-half, half)

    def left_min(self) -> float:
        return self._sector_min(self.wall_sec_min, self.wall_sec_max)

    def goal_direction_angle(self) -> float:
        """Angulo hacia el goal en el frame del robot ([-pi, pi])."""
        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        ang_world = math.atan2(dy, dx)
        return normalize_angle(ang_world - self.theta)

    def distance_to_goal(self) -> float:
        return math.hypot(self.goal_x - self.x, self.goal_y - self.y)

    def path_to_goal_clear(self) -> bool:
        """Cono en direccion al goal mas alla de clear_path_distance."""
        if self.scan_ranges is None or self.scan_angle_inc == 0.0:
            return True
        goal_dir = self.goal_direction_angle()
        half = self.clear_cone / 2.0
        return self._sector_min(goal_dir - half, goal_dir + half) > self.clear_dist

    # ----------------------------------------------------- Tick principal

    def tick(self):
        if not self.have_odom or self.goal_x is None:
            return

        have_scan = self.scan_ranges is not None
        if not have_scan:
            self.cmd_pub.publish(Twist())
            return

        dist = self.distance_to_goal()

        # --- Trail + visualizacion periodica ---
        self._update_trail()
        self._tick_count += 1
        if self._tick_count % self._log_interval == 0:
            self._publish_goal_marker()
            self._publish_path_trail()
            front = self.front_min() if have_scan else float('inf')
            left = self.left_min() if have_scan else float('inf')
            ang = self.goal_direction_angle()
            self.get_logger().info(
                f'[DIAG] st={self.STATE_NAMES[self.state]} '
                f'pose=({self.x:.2f},{self.y:.2f},{math.degrees(self.theta):.0f}) '
                f'goal=({self.goal_x:.2f},{self.goal_y:.2f}) dist={dist:.2f}m '
                f'ang={math.degrees(ang):.0f} front={front:.2f} left={left:.2f}'
            )
        # Estado y linea al goal: cada 5 ticks (~4Hz a 20Hz)
        if self._tick_count % 5 == 0:
            self._publish_state_text()
            self._publish_goal_line()

        # --- ARUCO_PAUSE: detenerse para que el EKF estabilice ---
        if self.state == self.STATE_ARUCO_PAUSE:
            now = self.get_clock().now()
            if self._aruco_pause_start is not None:
                elapsed = (now - self._aruco_pause_start).nanoseconds * 1e-9
                if elapsed < self.aruco_pause_dur:
                    # Parado total: el EKF sigue corrigiendo con ArUco
                    self.cmd_pub.publish(Twist())
                    return
                else:
                    # Pausa terminada -> cooldown y volver al estado anterior
                    self._aruco_cooldown_until = now + rclpy.duration.Duration(
                        seconds=self.aruco_cooldown_dur)
                    self._set_state(self._state_before_pause)
                    self._aruco_pause_start = None
                    # Reiniciar timer de FOLLOW_WALL para que goal_path_free
                    # no salga prematuramente con tiempo acumulado pre-pausa
                    if self.state == self.STATE_FOLLOW_WALL:
                        self.wall_follow_start_time = now
                    self.get_logger().info(
                        f'Bug 0: pausa terminada -> {self.STATE_NAMES[self.state]} '
                        f'(cooldown {self.aruco_cooldown_dur:.0f}s)')

        # --- Llegada al goal ---
        if dist < self.goal_tol:
            if not self.goal_reached_published:
                self.cmd_pub.publish(Twist())
                self.reached_pub.publish(Empty())
                self.goal_reached_published = True
                self._set_state(self.STATE_GOAL_REACHED)
                self.get_logger().info(
                    f'Bug 0: goal alcanzado en ({self.x:.2f}, {self.y:.2f}), '
                    f'dist={dist:.2f}m'
                )
            return

        # --- Maquina de estados ---
        if self.state == self.STATE_GO_TO_GOAL:
            front = self.front_min()
            ang = abs(self.goal_direction_angle())
            if front < self.obs_dist:
                self.wall_follow_start_dist_to_goal = dist
                self._follow_wall_start_time = self.get_clock().now()
                self._gtg_align_start = None
                self._set_state(self.STATE_FOLLOW_WALL)
                self.get_logger().info(
                    f'Bug 0: obstaculo a {front:.2f}m -> FOLLOW_WALL, '
                    f'hit_dist_to_goal={dist:.2f}m'
                )
                self._wall_follow_step()
            else:
                # Anti-spin: si lleva >4s girando (goal detras), ir a FOLLOW_WALL
                if ang > self.align_thr:
                    now = self.get_clock().now()
                    if self._gtg_align_start is None:
                        self._gtg_align_start = now
                    elif (now - self._gtg_align_start).nanoseconds * 1e-9 > 4.0:
                        self.wall_follow_start_dist_to_goal = dist
                        self._follow_wall_start_time = now
                        self._gtg_align_start = None
                        self._set_state(self.STATE_FOLLOW_WALL)
                        self.get_logger().warn(
                            f'Bug 0: SPIN detectado (>{4.0}s girando, ang={math.degrees(ang):.0f}) '
                            f'-> FOLLOW_WALL')
                        self._wall_follow_step()
                        return
                else:
                    self._gtg_align_start = None
                self._go_to_goal_step()

        elif self.state == self.STATE_FOLLOW_WALL:
            left = self.left_min()
            front_clear = self.front_min() > self.obs_dist
            # Tiempo que llevamos en FOLLOW_WALL
            fw_elapsed = 0.0
            if self._follow_wall_start_time is not None:
                fw_elapsed = (self.get_clock().now() - self._follow_wall_start_time).nanoseconds * 1e-9
            # wall_lost solo cuenta si:
            # 1. Llevamos suficiente tiempo en FW (evita loop al girar en dead-end)
            # 2. El goal esta DELANTE del robot (|ang| < 90°) — si esta detras,
            #    seguir la pared es mejor que girar 180° y quedarse oscilando
            goal_ang = abs(self.goal_direction_angle())
            goal_ahead = goal_ang < math.radians(70.0)
            wall_lost = (left > self.wall_target * 2.5 and front_clear
                         and fw_elapsed > self.min_fw_time and goal_ahead)
            progressed = (
                self.wall_follow_start_dist_to_goal is not None
                and dist < self.wall_follow_start_dist_to_goal - self.wall_progress
                and goal_ahead
            )

            # Bug0 puro: si el camino al goal esta libre y el goal esta
            # CASI enfrente, salir de FOLLOW_WALL.  Condiciones estrictas
            # para no salir prematuramente despues del primer giro:
            #   - goal a < 30° (no basta con < 70°)
            #   - frente despejado
            #   - path_to_goal_clear (cono libre de obstaculos)
            #   - al menos 5 s en FOLLOW_WALL (tiempo real, no min_fw_time)
            goal_path_free = (goal_ang < math.radians(30.0)
                              and front_clear
                              and self.path_to_goal_clear()
                              and fw_elapsed > 5.0)

            if (progressed or wall_lost or goal_path_free) and self.path_to_goal_clear():
                reason = ('camino al goal libre' if goal_path_free
                          else 'progreso+camino libre' if progressed else 'pared perdida')
                self._gtg_align_start = None
                self._set_state(self.STATE_GO_TO_GOAL)
                self.get_logger().info(
                    f'Bug 0: {reason}, dist={dist:.2f}m -> GO_TO_GOAL')
                self._go_to_goal_step()
            else:
                self._wall_follow_step()

        elif self.state == self.STATE_GOAL_REACHED:
            self.cmd_pub.publish(Twist())

    def _set_state(self, new_state):
        if new_state != self.state:
            old_name = self.STATE_NAMES.get(self.state, '?')
            new_name = self.STATE_NAMES.get(new_state, '?')
            self.get_logger().info(
                f'Bug 0: {old_name} -> {new_name} at ({self.x:.2f},{self.y:.2f})')
            self.last_state = self.state
            self.state = new_state

    # ---------------------------------------------------- Acciones de control

    def _go_to_goal_step(self, override_v=None):
        """Controlador P al goal (NumPy puro, sin librerias externas)."""
        dist = self.distance_to_goal()
        ang = self.goal_direction_angle()

        if abs(ang) > self.align_thr:
            # Giro de alineacion: usar turn_wmax (alto para menos patinaje)
            v = 0.0
            w = self.kw * ang
            w = max(-self.turn_wmax, min(self.turn_wmax, w))
        else:
            v_target = override_v if override_v is not None else min(self.kv * dist, self.vmax)
            v = max(0.0, min(self.vmax, v_target))
            w = self.kw * ang
            w = max(-self.wmax, min(self.wmax, w))

        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)

    def _wall_follow_step(self, override_v=None):
        """Wall following con la pared a la IZQUIERDA del robot."""
        front = self.front_min()
        left = self.left_min()
        wv = override_v if override_v is not None else self.wall_v

        # Pared al frente: gira derecha con turn_wmax
        if front < self.obs_dist:
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = -self.turn_wmax * 0.7
            self.cmd_pub.publish(msg)
            return

        # No hay pared visible por la izquierda: girar izquierda
        if left > self.wall_target * 2.5:
            msg = Twist()
            msg.linear.x = float(wv)
            msg.angular.z = float(self.wmax * 0.5)
            self.cmd_pub.publish(msg)
            return

        # Sigue la pared con un P saturado
        error = max(-self.wall_target, min(self.wall_target,
                                           left - self.wall_target))
        w = self.wall_k * error
        w = max(-self.wmax, min(self.wmax, w))

        msg = Twist()
        msg.linear.x = float(wv)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Bug0()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = Twist()
        try:
            node.timer.cancel()
            for _ in range(10):
                node.cmd_pub.publish(stop)
                rclpy.spin_once(node, timeout_sec=0.05)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
