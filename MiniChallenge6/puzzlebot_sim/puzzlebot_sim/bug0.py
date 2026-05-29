"""
MINI CHALLENGE 6 - BUG 0 NAVIGATION ALGORITHM

Suscribe:
  /odom         (nav_msgs/Odometry)       - pose estimada (localisation con covarianza)
  /scan         (sensor_msgs/LaserScan)   - LiDAR (Gazebo de robotec_sim_ws)
  /current_goal (geometry_msgs/PoseStamped) - waypoint actual (point_generator)

Publica:
  /cmd_vel      (geometry_msgs/Twist)     - comando al simulador
  /goal_reached (std_msgs/Empty)          - handshake con point_generator

State machine:
  GO_TO_GOAL:
    - Apuntar al objetivo + avanzar con controlador P (NumPy puro).
    - Si hay obstaculo en el cono frontal (+-15 deg) a menos de 0.5 m -> FOLLOW_WALL.
  FOLLOW_WALL:
    - Mantener la pared a la IZQUIERDA del robot a ~0.35 m (P sobre el error de distancia).
    - Si el cono hacia el goal queda libre Y nos hemos acercado al goal vs el hit_point,
      regresar a GO_TO_GOAL.

Wall following: pared a la izquierda (obstaculo a mano izquierda del robot).
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


class Bug0(Node):

    STATE_GO_TO_GOAL = 0
    STATE_FOLLOW_WALL = 1
    STATE_GOAL_REACHED = 2

    STATE_NAMES = {
        STATE_GO_TO_GOAL: 'GO_TO_GOAL',
        STATE_FOLLOW_WALL: 'FOLLOW_WALL',
        STATE_GOAL_REACHED: 'GOAL_REACHED',
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
        self.declare_parameter('align_threshold_deg', 25.0)
        self.declare_parameter('wall_k', 2.5)
        self.declare_parameter('wall_linear', 0.10)
        self.declare_parameter('wall_progress_threshold', 0.10)

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
        self.align_thr = math.radians(float(self.get_parameter('align_threshold_deg').value))
        self.wall_k = float(self.get_parameter('wall_k').value)
        self.wall_v = float(self.get_parameter('wall_linear').value)
        self.wall_progress = float(self.get_parameter('wall_progress_threshold').value)

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
            f'Bug 0 iniciado: obs_dist={self.obs_dist} m, '
            f'cono_frontal={math.degrees(self.front_cone):.0f} deg, '
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
        # Sustituye inf/nan por range_max para poder usar np.min sin problemas
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
        # Trata como goal nuevo si cambio
        if (self.goal_x is None or
                abs(new_x - self.goal_x) > 0.01 or
                abs(new_y - self.goal_y) > 0.01):
            self.goal_x = new_x
            self.goal_y = new_y
            self.state = self.STATE_GO_TO_GOAL
            self.goal_reached_published = False
            self.wall_follow_start_dist_to_goal = None
            self.get_logger().info(
                f'Bug 0: nuevo goal ({new_x:.2f}, {new_y:.2f})')

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
            return False
        goal_dir = self.goal_direction_angle()
        half = self.clear_cone / 2.0
        return self._sector_min(goal_dir - half, goal_dir + half) > self.clear_dist

    # ----------------------------------------------------- Tick principal

    def tick(self):
        if (not self.have_odom or self.goal_x is None or
                self.scan_ranges is None):
            return

        dist = self.distance_to_goal()

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
            if front < self.obs_dist:
                self.wall_follow_start_dist_to_goal = dist
                self._set_state(self.STATE_FOLLOW_WALL)
                self.get_logger().info(
                    f'Bug 0: obstaculo a {front:.2f}m -> FOLLOW_WALL, '
                    f'hit_dist_to_goal={dist:.2f}m'
                )
                self._wall_follow_step()
            else:
                self._go_to_goal_step()

        elif self.state == self.STATE_FOLLOW_WALL:
            # Volver a GO_TO_GOAL si:
            #   (a) progreso desde el hit point + cono al goal libre, O
            #   (b) perdimos la pared (left muy lejos) + cono al goal libre
            #       — evita quedar girando en circulos en espacio abierto
            left = self.left_min()
            front_clear = self.front_min() > self.obs_dist
            wall_lost = left > self.wall_target * 3.0 and front_clear
            progressed = (
                self.wall_follow_start_dist_to_goal is not None
                and dist < self.wall_follow_start_dist_to_goal - self.wall_progress
            )

            if (progressed or wall_lost) and self.path_to_goal_clear():
                reason = 'progreso+camino libre' if progressed else 'pared perdida'
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
            self.last_state = self.state
            self.state = new_state

    # ---------------------------------------------------- Acciones de control

    def _go_to_goal_step(self):
        """Controlador P al goal (NumPy puro, sin librerias externas)."""
        dist = self.distance_to_goal()
        ang = self.goal_direction_angle()

        if abs(ang) > self.align_thr:
            # Muy desalineado: gira en sitio (v=0)
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
        """Wall following con la pared a la IZQUIERDA del robot.

        - Si hay pared al frente (<obs_dist): gira a la derecha en sitio
          para poner la pared en la IZQUIERDA.
        - Si no hay pared cercana por la izquierda (left > 3*wall_target):
          avanza recto (no aplicar PID, evita "circles in open space" cuando
          se entra a FOLLOW_WALL antes de poder ver la pared por el costado).
        - Si hay pared a la izquierda: P sobre (left_min - wall_target),
          saturando el error a |+- wall_target| para evitar w al maximo.
        """
        front = self.front_min()
        left = self.left_min()

        # Pared al frente (entrando a FOLLOW_WALL o callejon): gira derecha
        if front < self.obs_dist:
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = -self.wmax * 0.7
            self.cmd_pub.publish(msg)
            return

        # No hay pared visible por la izquierda: avanza recto buscando salida
        if left > self.wall_target * 3.0:
            msg = Twist()
            msg.linear.x = float(self.wall_v * 1.5)
            msg.angular.z = 0.0
            self.cmd_pub.publish(msg)
            return

        # Sigue la pared con un P saturado (evita w explosivo si left es grande)
        error = max(-self.wall_target, min(self.wall_target,
                                           left - self.wall_target))
        w = self.wall_k * error
        w = max(-self.wmax, min(self.wmax, w))

        msg = Twist()
        msg.linear.x = float(self.wall_v)
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
        # Asegura que el robot se detenga al salir
        try:
            node.cmd_pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
