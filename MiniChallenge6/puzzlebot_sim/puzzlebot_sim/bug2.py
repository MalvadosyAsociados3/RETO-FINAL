"""
BUG 2 NAVIGATION ALGORITHM — Final Challenge

Diferencia vs Bug 0: usa la "linea M" (start -> goal). Cuando encuentra
un obstaculo, registra un hit_point H, sigue la pared, y solo deja la
pared cuando re-intersecta la linea M en un leave_point L MAS CERCANO al
goal que H.

Esto es mas predecible que Bug0 porque la condicion de salida de
FOLLOW_WALL es geometrica (volver a la M-line) en vez de heuristica
(progreso + cono libre).

Suscribe:
  /odom         (nav_msgs/Odometry)        - pose estimada (EKF)
  /scan         (sensor_msgs/LaserScan)    - LiDAR
  /current_goal (geometry_msgs/PoseStamped) - waypoint (point_generator)

Publica:
  /cmd_vel      (geometry_msgs/Twist)      - comando al robot
  /goal_reached (std_msgs/Empty)           - handshake con point_generator
  /goal_marker  (visualization_msgs/Marker) - cilindro visual en RViz

State machine:
  GO_TO_GOAL:
    - Sigue la linea M con controlador P.
    - Obstaculo en cono frontal -> registra H, salta a FOLLOW_WALL.
  FOLLOW_WALL:
    - Pared a la IZQUIERDA a wall_target.
    - LEAVE cuando: sobre M-line AND alejado de H AND mas cerca del goal que H.
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       DurabilityPolicy, qos_profile_sensor_data)
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty, ColorRGBA
from visualization_msgs.msg import Marker


def quat_to_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def normalize_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class Bug2(Node):

    STATE_GO_TO_GOAL = 0
    STATE_FOLLOW_WALL = 1
    STATE_GOAL_REACHED = 2

    STATE_NAMES = {
        STATE_GO_TO_GOAL: 'GO_TO_GOAL',
        STATE_FOLLOW_WALL: 'FOLLOW_WALL',
        STATE_GOAL_REACHED: 'GOAL_REACHED',
    }

    def __init__(self):
        super().__init__('bug2')

        # --- Parametros (lee de la seccion bug2: del YAML) ---
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('goal_tolerance', 0.15)
        self.declare_parameter('obstacle_distance', 0.25)
        self.declare_parameter('forward_cone_deg', 30.0)
        self.declare_parameter('wall_target_distance', 0.20)
        self.declare_parameter('wall_sector_min_deg', 60.0)
        self.declare_parameter('wall_sector_max_deg', 120.0)
        self.declare_parameter('m_line_tolerance', 0.10)
        self.declare_parameter('hit_min_distance', 0.20)
        self.declare_parameter('progress_threshold', 0.05)
        self.declare_parameter('k_linear', 0.40)
        self.declare_parameter('k_angular', 0.8)
        self.declare_parameter('max_linear', 0.10)
        self.declare_parameter('max_angular', 0.5)
        self.declare_parameter('align_threshold_deg', 35.0)
        self.declare_parameter('right_sector_min_deg', -120.0)
        self.declare_parameter('right_sector_max_deg', -60.0)
        self.declare_parameter('right_emergency_distance', 0.15)
        self.declare_parameter('wall_lost_min_distance', 3.5)
        self.declare_parameter('stuck_timeout', 8.0)
        self.declare_parameter('stuck_distance', 0.05)
        self.declare_parameter('wall_k', 2.0)
        self.declare_parameter('wall_linear', 0.05)

        rate = float(self.get_parameter('control_rate').value)
        self.goal_tol = float(self.get_parameter('goal_tolerance').value)
        self.obs_dist = float(self.get_parameter('obstacle_distance').value)
        self.front_cone = math.radians(float(self.get_parameter('forward_cone_deg').value))
        self.wall_target = float(self.get_parameter('wall_target_distance').value)
        self.wall_sec_min = math.radians(float(self.get_parameter('wall_sector_min_deg').value))
        self.wall_sec_max = math.radians(float(self.get_parameter('wall_sector_max_deg').value))
        self.m_line_tol = float(self.get_parameter('m_line_tolerance').value)
        self.hit_min_dist = float(self.get_parameter('hit_min_distance').value)
        self.progress_thr = float(self.get_parameter('progress_threshold').value)
        self.kv = float(self.get_parameter('k_linear').value)
        self.kw = float(self.get_parameter('k_angular').value)
        self.vmax = float(self.get_parameter('max_linear').value)
        self.wmax = float(self.get_parameter('max_angular').value)
        self.align_thr = math.radians(float(self.get_parameter('align_threshold_deg').value))
        self.right_sec_min = math.radians(float(self.get_parameter('right_sector_min_deg').value))
        self.right_sec_max = math.radians(float(self.get_parameter('right_sector_max_deg').value))
        self.right_emergency = float(self.get_parameter('right_emergency_distance').value)
        self.wall_lost_min = float(self.get_parameter('wall_lost_min_distance').value)
        self.stuck_timeout = float(self.get_parameter('stuck_timeout').value)
        self.stuck_dist = float(self.get_parameter('stuck_distance').value)
        self.wall_k = float(self.get_parameter('wall_k').value)
        self.wall_v = float(self.get_parameter('wall_linear').value)

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

        # Goal + linea M
        self.goal_x = None
        self.goal_y = None
        self.start_x = None
        self.start_y = None
        self.m_dx = 0.0
        self.m_dy = 0.0
        self.m_len = 0.0

        # Hit point
        self.hit_x = None
        self.hit_y = None
        self.hit_dist_to_goal = None

        # State machine
        self.state = self.STATE_GO_TO_GOAL
        self.goal_reached_published = False
        self._tick_count = 0
        self._log_interval = int(rate * 2)  # log cada ~2 seg

        # Stuck detection
        self._stuck_ref_x = 0.0
        self._stuck_ref_y = 0.0
        self._stuck_ref_time = 0.0
        self._stuck_initialized = False

        # Cooldown: tras LEAVE, ignorar HIT por N ticks para no re-entrar
        self._leave_cooldown = 0

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

        # --- Pubs ---
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.reached_pub = self.create_publisher(Empty, 'goal_reached', reliable_qos)
        self.goal_marker_pub = self.create_publisher(Marker, 'goal_marker', 10)

        # --- Timer ---
        self.timer = self.create_timer(1.0 / rate, self.tick)

        self.get_logger().info(
            f'Bug 2 iniciado: obs_dist={self.obs_dist}m, '
            f'M-line_tol={self.m_line_tol}m, '
            f'wall_target={self.wall_target}m (IZQUIERDA), '
            f'ctrl_rate={rate}Hz'
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
            self.start_x = self.x
            self.start_y = self.y
            self.m_dx = self.goal_x - self.start_x
            self.m_dy = self.goal_y - self.start_y
            self.m_len = math.hypot(self.m_dx, self.m_dy)
            self.hit_x = None
            self.hit_y = None
            self.hit_dist_to_goal = None
            self.state = self.STATE_GO_TO_GOAL
            self.goal_reached_published = False
            self._publish_goal_marker()
            self.get_logger().info(
                f'Bug 2: nuevo goal ({new_x:.2f}, {new_y:.2f}) | '
                f'M-line start=({self.start_x:.2f},{self.start_y:.2f}) '
                f'len={self.m_len:.2f}m'
            )

    def _publish_goal_marker(self):
        if self.goal_x is None:
            return
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'bug2_goal'
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

    # ---------------------------------------------------- LiDAR utilities

    def _sector_min(self, angle_lo: float, angle_hi: float) -> float:
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

    def right_min(self) -> float:
        return self._sector_min(self.right_sec_min, self.right_sec_max)

    def front_right_min(self) -> float:
        """Sector frontal-derecho: -50° a -5°. Detecta paredes en angulo."""
        return self._sector_min(math.radians(-50.0), math.radians(-5.0))

    def front_left_min(self) -> float:
        """Sector frontal-izquierdo: 5° a 50°."""
        return self._sector_min(math.radians(5.0), math.radians(50.0))

    def goal_direction_angle(self) -> float:
        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        return normalize_angle(math.atan2(dy, dx) - self.theta)

    def distance_to_goal(self) -> float:
        return math.hypot(self.goal_x - self.x, self.goal_y - self.y)

    def distance_to_m_line(self) -> float:
        """Distancia perpendicular del robot a la linea M (start -> goal)."""
        if self.m_len < 1e-6:
            return 0.0
        num = abs(self.m_dx * (self.start_y - self.y)
                  - (self.start_x - self.x) * self.m_dy)
        return num / self.m_len

    def distance_to_hit(self) -> float:
        if self.hit_x is None:
            return 0.0
        return math.hypot(self.x - self.hit_x, self.y - self.hit_y)

    # ----------------------------------------------------- Tick principal

    def tick(self):
        if not self.have_odom or self.goal_x is None:
            return

        # No moverse sin LiDAR — evita chocar al arrancar.
        if self.scan_ranges is None:
            self.cmd_pub.publish(Twist())
            return

        dist_goal = self.distance_to_goal()
        front = self.front_min()
        left = self.left_min()
        right = self.right_min()

        # --- Log periodico + marker visual (cada ~2 seg) ---
        self._tick_count += 1
        if self._tick_count % self._log_interval == 0:
            self._publish_goal_marker()
            ang = self.goal_direction_angle()
            d_mline = self.distance_to_m_line()
            d_hit = self.distance_to_hit()
            self.get_logger().info(
                f'[DIAG] st={self.STATE_NAMES[self.state]} '
                f'pose=({self.x:.2f},{self.y:.2f},{math.degrees(self.theta):.0f}deg) '
                f'goal=({self.goal_x:.2f},{self.goal_y:.2f}) dist={dist_goal:.2f}m '
                f'ang={math.degrees(ang):.0f}deg front={front:.2f} left={left:.2f} '
                f'right={right:.2f} d_Mline={d_mline:.3f} d_hit={d_hit:.2f}'
            )

        # --- Frenado de emergencia: pared derecha demasiado cerca ---
        if right < self.right_emergency:
            msg = Twist()
            msg.linear.x = float(self.vmax * 0.2)   # avanzar lento, no pivotear
            msg.angular.z = float(self.wmax * 0.5)   # gira izquierda para alejarse
            self.cmd_pub.publish(msg)
            return

        # --- Detección de stuck: si no avanzamos en N segundos, escapar ---
        now = self.get_clock().now().nanoseconds * 1e-9
        if not self._stuck_initialized:
            self._stuck_ref_x = self.x
            self._stuck_ref_y = self.y
            self._stuck_ref_time = now
            self._stuck_initialized = True
        else:
            moved = math.hypot(self.x - self._stuck_ref_x,
                               self.y - self._stuck_ref_y)
            elapsed = now - self._stuck_ref_time
            if moved > self.stuck_dist:
                self._stuck_ref_x = self.x
                self._stuck_ref_y = self.y
                self._stuck_ref_time = now
            elif elapsed > self.stuck_timeout and self.state == self.STATE_FOLLOW_WALL:
                self.get_logger().warn(
                    f'STUCK detectado: {moved:.3f}m en {elapsed:.1f}s -> escape'
                )
                # Solo retroceder, sin girar. Girar en falso acumula
                # drift de theta en los encoders y destruye la localizacion.
                msg = Twist()
                msg.linear.x = -float(self.vmax)
                msg.angular.z = 0.0
                self.cmd_pub.publish(msg)
                self._stuck_ref_x = self.x
                self._stuck_ref_y = self.y
                self._stuck_ref_time = now
                return

        # --- Llegada al goal ---
        if dist_goal < self.goal_tol:
            if not self.goal_reached_published:
                self.cmd_pub.publish(Twist())
                self.reached_pub.publish(Empty())
                self.goal_reached_published = True
                self.state = self.STATE_GOAL_REACHED
                self._publish_goal_marker()
                self.get_logger().info(
                    f'Bug 2: GOAL ALCANZADO en ({self.x:.2f}, {self.y:.2f}), '
                    f'dist={dist_goal:.2f}m')
            return

        # --- Maquina de estados ---
        if self.state == self.STATE_GO_TO_GOAL:
            goal_ang = abs(self.goal_direction_angle())
            front_right = self.front_right_min()
            front_left = self.front_left_min()
            # HIT: obstaculo en frente, frente-derecho, o frente-izquierdo
            obstacle_ahead = front < self.obs_dist
            obstacle_fr = front_right < self.obs_dist
            obstacle_fl = front_left < self.obs_dist
            # Cooldown: tras LEAVE, no hacer HIT por unos ticks
            if self._leave_cooldown > 0:
                self._leave_cooldown -= 1

            if (obstacle_ahead or obstacle_fr or obstacle_fl) and goal_ang < math.radians(90.0) and self._leave_cooldown == 0:
                self.hit_x = self.x
                self.hit_y = self.y
                self.hit_dist_to_goal = dist_goal
                self.state = self.STATE_FOLLOW_WALL
                hit_sector = 'front' if obstacle_ahead else ('front-R' if obstacle_fr else 'front-L')
                hit_dist = min(front, front_right, front_left)
                self.get_logger().info(
                    f'Bug 2: HIT en ({self.x:.2f},{self.y:.2f}), '
                    f'd_goal={dist_goal:.2f}m, {hit_sector}={hit_dist:.2f}m -> FOLLOW_WALL'
                )
                self._wall_follow_step()
            else:
                self._go_to_goal_step()

        elif self.state == self.STATE_FOLLOW_WALL:
            # Bug2 LEAVE conditions:
            # 1. Sobre la linea M (dentro de m_line_tol)
            # 2. Alejado del hit point (anti-bucle inmediato)
            # 3. Mas cerca del goal que el hit point
            on_m_line = self.distance_to_m_line() < self.m_line_tol
            away_from_hit = self.distance_to_hit() > self.hit_min_dist
            closer_to_goal = (self.hit_dist_to_goal is not None and
                              dist_goal < self.hit_dist_to_goal - self.progress_thr)

            # Salvavidas: pared perdida (espacio abierto) + alejado del hit
            front_clear = front > self.obs_dist
            wall_lost = left > self.wall_target * self.wall_lost_min and front_clear

            # Hibrido Bug0/Bug2: si el camino al goal esta libre, salir
            # sin necesitar la M-line. Util en laberintos angostos donde
            # la M-line nunca se cruza durante wall-follow.
            goal_ang = abs(self.goal_direction_angle())
            goal_path_clear = (front > self.obs_dist * 2.0 and
                               self.front_right_min() > self.obs_dist * 1.5 and
                               self.front_left_min() > self.obs_dist * 1.5 and
                               goal_ang < math.radians(60.0))

            if on_m_line and away_from_hit and closer_to_goal:
                self.state = self.STATE_GO_TO_GOAL
                self._leave_cooldown = 40  # 2s de gracia antes de nuevo HIT
                self.get_logger().info(
                    f'Bug 2: LEAVE (M-line) en ({self.x:.2f},{self.y:.2f}), '
                    f'd_goal={dist_goal:.2f}m, d_Mline={self.distance_to_m_line():.3f}m '
                    f'-> GO_TO_GOAL'
                )
                self._go_to_goal_step()
            elif goal_path_clear and away_from_hit and closer_to_goal:
                self.state = self.STATE_GO_TO_GOAL
                self._leave_cooldown = 40
                self.get_logger().info(
                    f'Bug 2: LEAVE (goal visible) en ({self.x:.2f},{self.y:.2f}), '
                    f'd_goal={dist_goal:.2f}m, ang={math.degrees(goal_ang):.0f}deg '
                    f'-> GO_TO_GOAL'
                )
                self._go_to_goal_step()
            elif wall_lost and away_from_hit:
                self.state = self.STATE_GO_TO_GOAL
                self._leave_cooldown = 40
                self.get_logger().info(
                    f'Bug 2: pared perdida en ({self.x:.2f},{self.y:.2f}), '
                    f'd_goal={dist_goal:.2f}m -> GO_TO_GOAL'
                )
                self._go_to_goal_step()
            else:
                self._wall_follow_step()

        elif self.state == self.STATE_GOAL_REACHED:
            self.cmd_pub.publish(Twist())

    # ---------------------------------------------------- Acciones de control

    def _go_to_goal_step(self):
        dist = self.distance_to_goal()
        ang = self.goal_direction_angle()

        if abs(ang) > self.align_thr:
            v = self.vmax * 0.25  # nunca pivotear puro: rueda > patina
            w = self.kw * ang
        else:
            v = min(self.kv * dist, self.vmax)
            w = self.kw * ang

        # Evasion suave: si hay pared cerca por el frente-derecho, sesgar izquierda
        fr = self.front_right_min()
        if fr < self.obs_dist * 1.5:
            w += self.wmax * 0.3  # empujar hacia la izquierda
            v = min(v, self.vmax * 0.5)  # reducir velocidad

        v = max(0.0, min(self.vmax, v))
        w = max(-self.wmax, min(self.wmax, w))

        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)

    def _wall_follow_step(self):
        """Wall following con pared a la IZQUIERDA del robot."""
        front = self.front_min()
        left = self.left_min()
        right = self.right_min()
        front_right = self.front_right_min()

        # Esquina: pared al frente, o frente cerca + frente-derecho cerca.
        # obs_dist * 1.2 = 30cm, suficiente para pasillos de 60cm.
        is_corner = (front < self.obs_dist or
                     (front < self.obs_dist * 1.2 and front_right < self.obs_dist))
        if is_corner:
            msg = Twist()
            msg.linear.x = -float(self.wall_v * 0.3)  # retroceder lento
            msg.angular.z = -self.wmax * 0.4           # girar derecha suave
            self.cmd_pub.publish(msg)
            return

        # Pared derecha demasiado cerca: corregir a la izquierda
        if right < self.right_emergency * 1.5:
            msg = Twist()
            msg.linear.x = float(self.wall_v * 0.5)
            msg.angular.z = float(self.wmax * 0.4)
            self.cmd_pub.publish(msg)
            return

        # No hay pared por la izquierda: gira izquierda para rodear esquina
        if left > self.wall_target * 2.5:
            msg = Twist()
            msg.linear.x = float(self.wall_v)
            msg.angular.z = float(self.wmax * 0.5)
            self.cmd_pub.publish(msg)
            return

        # P saturado: mantener wall_target con pared a la izquierda
        error = max(-self.wall_target, min(self.wall_target,
                                           left - self.wall_target))
        w = self.wall_k * error

        # Sesgar izquierda si frente-derecho esta cerca (zona de riesgo)
        if front_right < self.obs_dist * 2.0:
            w += self.wmax * 0.2

        # Limitar w para que no domine sobre v (evita pivoteo puro)
        max_w_wall = self.wmax * 0.6
        w = max(-max_w_wall, min(max_w_wall, w))

        msg = Twist()
        msg.linear.x = float(self.wall_v)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Bug2()
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
