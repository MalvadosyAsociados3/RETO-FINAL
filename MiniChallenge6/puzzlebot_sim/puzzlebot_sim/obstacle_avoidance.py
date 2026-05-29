"""
Obstacle avoidance reactiva — capa entre multi_point_nav y el robot.

Subscribe:
  /pre_cmd_vel  (geometry_msgs/Twist)     - comando "deseado" del nav (go-to-goal)
  /scan         (sensor_msgs/LaserScan)   - LiDAR

Publica:
  /cmd_vel      (geometry_msgs/Twist)     - comando final al robot

Logica (Bug 0 reactivo):
  GO_TO_GOAL:
    - Mientras el cono frontal este libre, pasamos /pre_cmd_vel tal cual a /cmd_vel.
    - Si aparece un obstaculo en el cono frontal -> FOLLOW_WALL.

  FOLLOW_WALL:
    - Pared a la izquierda del robot a ~wall_target. Controlador P sobre el error.
    - Se sale a GO_TO_GOAL cuando el frente vuelve a estar libre Y la pared se ha
      "perdido" por la izquierda (rodearemos el obstaculo). El criterio es
      conservador para evitar oscilaciones (mismas reglas que el MC6 bug0
      arreglado).

Notas:
  - No conoce el goal: solo reacciona al /pre_cmd_vel + /scan.
  - El frame del LiDAR es el del Puzzlebot (laser_frame); el angulo 0 apunta hacia
    adelante.
  - El sentido del wall-following queda fijo a "pared a la izquierda". Si tu
    obstacle_avoidance espera otro sentido, ajusta wall_sector_min/max_deg.
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data,
)
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class ObstacleAvoidance(Node):

    STATE_GO_TO_GOAL = 0
    STATE_FOLLOW_WALL = 1
    STATE_NAMES = {0: 'GO_TO_GOAL', 1: 'FOLLOW_WALL'}

    def __init__(self):
        super().__init__('obstacle_avoidance')

        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('obstacle_distance', 0.5)
        self.declare_parameter('forward_cone_deg', 30.0)
        self.declare_parameter('wall_target_distance', 0.35)
        self.declare_parameter('wall_sector_min_deg', 60.0)
        self.declare_parameter('wall_sector_max_deg', 120.0)
        self.declare_parameter('wall_k', 2.5)
        self.declare_parameter('wall_linear', 0.10)
        self.declare_parameter('max_angular', 1.0)
        self.declare_parameter('clear_front_hysteresis', 1.3)
        self.declare_parameter('publish_zero_when_idle', True)

        rate = float(self.get_parameter('control_rate').value)
        self.obs_dist = float(self.get_parameter('obstacle_distance').value)
        self.front_cone = math.radians(
            float(self.get_parameter('forward_cone_deg').value)
        )
        self.wall_target = float(self.get_parameter('wall_target_distance').value)
        self.wall_sec_min = math.radians(
            float(self.get_parameter('wall_sector_min_deg').value)
        )
        self.wall_sec_max = math.radians(
            float(self.get_parameter('wall_sector_max_deg').value)
        )
        self.wall_k = float(self.get_parameter('wall_k').value)
        self.wall_v = float(self.get_parameter('wall_linear').value)
        self.wmax = float(self.get_parameter('max_angular').value)
        self.clear_hyst = float(self.get_parameter('clear_front_hysteresis').value)
        self.zero_when_idle = bool(self.get_parameter('publish_zero_when_idle').value)

        # Estado
        self.state = self.STATE_GO_TO_GOAL
        self.pre_cmd = Twist()
        self.have_pre = False
        self.have_scan = False
        self.scan_ranges = None
        self.scan_angle_min = 0.0
        self.scan_angle_inc = 0.0
        self.scan_range_max = 10.0

        # QoS
        reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.pre_sub = self.create_subscription(
            Twist, 'pre_cmd_vel', self.pre_cb, reliable,
        )
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_cb, qos_profile_sensor_data,
        )
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.timer = self.create_timer(1.0 / rate, self.tick)

        self.get_logger().info(
            f'ObstacleAvoidance iniciado: obs_dist={self.obs_dist} m, '
            f'wall_target={self.wall_target} m, ctrl_rate={rate} Hz'
        )

    # -------------------------------------------------------- Callbacks

    def pre_cb(self, msg: Twist):
        self.pre_cmd = msg
        self.have_pre = True

    def scan_cb(self, msg: LaserScan):
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        bad = ~np.isfinite(ranges)
        ranges = np.where(bad, msg.range_max, ranges)
        self.scan_ranges = ranges
        self.scan_angle_min = float(msg.angle_min)
        self.scan_angle_inc = float(msg.angle_increment)
        self.scan_range_max = float(msg.range_max)
        self.have_scan = True

    # ------------------------------------------------------ LiDAR helpers

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

    # ----------------------------------------------------- Main loop

    def tick(self):
        if not self.have_scan:
            if self.zero_when_idle:
                self.cmd_pub.publish(Twist())
            return

        front = self.front_min()
        left = self.left_min()

        # Transiciones de estado
        if self.state == self.STATE_GO_TO_GOAL:
            if front < self.obs_dist:
                self.state = self.STATE_FOLLOW_WALL
                self.get_logger().info(
                    f'Obstaculo a {front:.2f}m -> FOLLOW_WALL'
                )
        else:  # FOLLOW_WALL
            # Reglas para salir: frente despejado con histeresis Y pared a la
            # izquierda fuera del rango (la rodeamos completamente).
            front_clear = front > self.obs_dist * self.clear_hyst
            wall_far = left > self.wall_target * 3.0
            if front_clear and wall_far:
                self.state = self.STATE_GO_TO_GOAL
                self.get_logger().info('Pared rodeada -> GO_TO_GOAL')

        # Acciones por estado
        if self.state == self.STATE_GO_TO_GOAL:
            # Pasa el pre_cmd_vel tal cual.
            out = self.pre_cmd if self.have_pre else Twist()
        else:
            out = self._wall_follow_cmd(front, left)

        self.cmd_pub.publish(out)

    def _wall_follow_cmd(self, front: float, left: float) -> Twist:
        msg = Twist()

        if front < self.obs_dist:
            # Pared al frente: gira a la derecha en sitio.
            msg.linear.x = 0.0
            msg.angular.z = -self.wmax * 0.7
            return msg

        if left > self.wall_target * 3.0:
            # No vemos pared a la izquierda: avanza recto buscando salida.
            msg.linear.x = float(self.wall_v * 1.5)
            msg.angular.z = 0.0
            return msg

        # P saturado: mantener wall_target con la pared a la izquierda.
        err = max(-self.wall_target,
                  min(self.wall_target, left - self.wall_target))
        w = self.wall_k * err
        w = max(-self.wmax, min(self.wmax, w))
        msg.linear.x = float(self.wall_v)
        msg.angular.z = float(w)
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
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
