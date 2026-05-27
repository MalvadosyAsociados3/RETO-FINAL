"""
MINI CHALLENGE 6 - BUG 2 NAVIGATION ALGORITHM

Diferencia vs Bug 0: usa la "linea M" (start -> goal). Cuando encuentra
un obstaculo, registra un hit_point H, sigue la pared, y solo deja la
pared cuando re-intersecta la linea M en un leave_point L MAS CERCANO al
goal que H.

Suscribe:
  /odom         (nav_msgs/Odometry)        - pose estimada (localisation)
  /scan         (sensor_msgs/LaserScan)    - LiDAR (Gazebo robotec_sim_ws)
  /current_goal (geometry_msgs/PoseStamped) - waypoint (point_generator)

Publica:
  /cmd_vel      (geometry_msgs/Twist)      - comando al simulador
  /goal_reached (std_msgs/Empty)           - handshake con point_generator

State machine:
  GO_TO_GOAL:
    - Sigue la linea M (start -> goal) con controlador P.
    - Obstaculo en cono frontal -> registra H, salta a FOLLOW_WALL.
  FOLLOW_WALL:
    - Pared a la IZQUIERDA del robot a wall_target.
    - Si re-intersecta la M-line con dist(actual, goal) < dist(H, goal),
      vuelve a GO_TO_GOAL.

Restriccion del challenge: solo NumPy + libreria estandar de Python.
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
from std_msgs.msg import Empty


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

        # --- Parametros ---
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('goal_tolerance', 0.15)
        self.declare_parameter('obstacle_distance', 0.5)
        self.declare_parameter('forward_cone_deg', 30.0)
        self.declare_parameter('wall_target_distance', 0.35)
        self.declare_parameter('wall_sector_min_deg', 60.0)
        self.declare_parameter('wall_sector_max_deg', 120.0)
        self.declare_parameter('m_line_tolerance', 0.10)
        self.declare_parameter('hit_min_distance', 0.20)
        self.declare_parameter('progress_threshold', 0.10)
        self.declare_parameter('k_linear', 0.5)
        self.declare_parameter('k_angular', 1.5)
        self.declare_parameter('max_linear', 0.18)
        self.declare_parameter('max_angular', 1.0)
        self.declare_parameter('align_threshold_deg', 25.0)
        self.declare_parameter('wall_k', 2.5)
        self.declare_parameter('wall_linear', 0.10)

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

        # --- Timer ---
        self.timer = self.create_timer(1.0 / rate, self.tick)

        self.get_logger().info(
            f'Bug 2 iniciado: obs_dist={self.obs_dist} m, '
            f'M-line_tol={self.m_line_tol} m, '
            f'wall_target={self.wall_target} m (IZQUIERDA), '
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
            # Reset M-line with current pos as start (recalculo cuando hay un goal nuevo).
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
            self.get_logger().info(
                f'Bug 2: nuevo goal ({new_x:.2f}, {new_y:.2f}) | '
                f'linea M start=({self.start_x:.2f},{self.start_y:.2f}) '
                f'len={self.m_len:.2f}m'
            )

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
        # Forma del producto cruzado:
        #   d = |(m_dx)*(start_y - y) - (start_x - x)*(m_dy)| / |M|
        num = abs(self.m_dx * (self.start_y - self.y)
                  - (self.start_x - self.x) * self.m_dy)
        return num / self.m_len

    def distance_to_hit(self) -> float:
        if self.hit_x is None:
            return 0.0
        return math.hypot(self.x - self.hit_x, self.y - self.hit_y)

    # ----------------------------------------------------- Tick principal

    def tick(self):
        if (not self.have_odom or self.goal_x is None or
                self.scan_ranges is None):
            return

        dist_goal = self.distance_to_goal()

        # --- Llegada al goal ---
        if dist_goal < self.goal_tol:
            if not self.goal_reached_published:
                self.cmd_pub.publish(Twist())
                self.reached_pub.publish(Empty())
                self.goal_reached_published = True
                self.state = self.STATE_GOAL_REACHED
                self.get_logger().info(
                    f'Bug 2: goal alcanzado en ({self.x:.2f}, {self.y:.2f})')
            return

        # --- Maquina de estados ---
        if self.state == self.STATE_GO_TO_GOAL:
            front = self.front_min()
            if front < self.obs_dist:
                # Registra hit point
                self.hit_x = self.x
                self.hit_y = self.y
                self.hit_dist_to_goal = dist_goal
                self.state = self.STATE_FOLLOW_WALL
                self.get_logger().info(
                    f'Bug 2: HIT en ({self.x:.2f},{self.y:.2f}), '
                    f'd_goal={dist_goal:.2f}m, front={front:.2f}m -> FOLLOW_WALL'
                )
                self._wall_follow_step()
            else:
                self._go_to_goal_step()

        elif self.state == self.STATE_FOLLOW_WALL:
            # Condiciones para dejar la pared (LEAVE):
            # 1. Estamos sobre la linea M (dentro de m_line_tol)
            # 2. Nos hemos alejado del hit point al menos hit_min_dist (anti-bucle)
            # 3. Estamos mas cerca del goal que el hit point por margen progress_thr
            on_m_line = self.distance_to_m_line() < self.m_line_tol
            away_from_hit = self.distance_to_hit() > self.hit_min_dist
            closer_to_goal = (self.hit_dist_to_goal is not None and
                              dist_goal < self.hit_dist_to_goal - self.progress_thr)

            if on_m_line and away_from_hit and closer_to_goal:
                self.state = self.STATE_GO_TO_GOAL
                self.get_logger().info(
                    f'Bug 2: LEAVE en ({self.x:.2f},{self.y:.2f}), '
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
            v = 0.0
            w = self.kw * ang
        else:
            v = min(self.kv * dist, self.vmax)
            w = self.kw * ang

        v = max(0.0, min(self.vmax, v))
        w = max(-self.wmax, min(self.wmax, w))

        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)

    def _wall_follow_step(self):
        """Wall following con pared a la IZQUIERDA del robot.

        Misma logica que en Bug 0 (vease bug0.py): si hay algo en frente
        cercano gira a la derecha en sitio; si no, P sobre el error de
        distancia a la pared izquierda.
        """
        front = self.front_min()
        if front < self.obs_dist * 0.6:
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = -self.wmax * 0.7
            self.cmd_pub.publish(msg)
            return

        left = self.left_min()
        error = left - self.wall_target
        w = self.wall_k * error
        w = max(-self.wmax, min(self.wmax, w))

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
        try:
            node.cmd_pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
