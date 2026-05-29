"""
Multi-point navigation: Go-To-Goal puro.

Subscribe:
  /odom         (nav_msgs/Odometry)         - pose actual
  /current_goal (geometry_msgs/PoseStamped) - waypoint a alcanzar

Publica:
  /pre_cmd_vel  (geometry_msgs/Twist)       - comando deseado al goal
  /goal_reached (std_msgs/Empty)            - handshake al point_generator

Controlador:
  - Si error angular |ang| > align_threshold: gira en sitio (v=0).
  - Si esta alineado: avanza con v = k_linear * dist (saturado), w = k_angular * ang.

Esto reemplaza la parte GO_TO_GOAL de bug0/bug2: ahora ese codigo se separa
en este nodo, y bug0/bug2 quedan como una capa REACTIVA de obstacle avoidance
que recibe /pre_cmd_vel y publica /cmd_vel. Asi el grafo coincide con el
diagrama del Final Challenge:

  Goals -> Multi-point navigation -> /pre_cmd_vel -> Obstacle avoidance -> /cmd_vel
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy,
)
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty


def quat_to_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def normalize_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class MultiPointNav(Node):

    def __init__(self):
        super().__init__('multi_point_nav')

        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('goal_tolerance', 0.15)
        self.declare_parameter('k_linear', 0.5)
        self.declare_parameter('k_angular', 1.5)
        self.declare_parameter('max_linear', 0.18)
        self.declare_parameter('max_angular', 1.0)
        self.declare_parameter('align_threshold_deg', 25.0)

        rate = float(self.get_parameter('control_rate').value)
        self.goal_tol = float(self.get_parameter('goal_tolerance').value)
        self.kv = float(self.get_parameter('k_linear').value)
        self.kw = float(self.get_parameter('k_angular').value)
        self.vmax = float(self.get_parameter('max_linear').value)
        self.wmax = float(self.get_parameter('max_angular').value)
        self.align_thr = math.radians(
            float(self.get_parameter('align_threshold_deg').value)
        )

        # Estado
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.have_odom = False
        self.goal_x = None
        self.goal_y = None
        self.goal_reached_published = False

        # QoS
        reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_cb, reliable,
        )
        self.goal_sub = self.create_subscription(
            PoseStamped, 'current_goal', self.goal_cb, latched,
        )

        self.cmd_pub = self.create_publisher(Twist, 'pre_cmd_vel', 10)
        self.reached_pub = self.create_publisher(Empty, 'goal_reached', reliable)

        self.timer = self.create_timer(1.0 / rate, self.tick)

        self.get_logger().info(
            f'MultiPointNav iniciado: tol={self.goal_tol} m, vmax={self.vmax}, '
            f'wmax={self.wmax}, align={math.degrees(self.align_thr):.0f} deg'
        )

    def odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.theta = quat_to_yaw(msg.pose.pose.orientation)
        self.have_odom = True

    def goal_cb(self, msg: PoseStamped):
        new_x = float(msg.pose.position.x)
        new_y = float(msg.pose.position.y)
        if (self.goal_x is None or
                abs(new_x - self.goal_x) > 0.01 or
                abs(new_y - self.goal_y) > 0.01):
            self.goal_x = new_x
            self.goal_y = new_y
            self.goal_reached_published = False
            self.get_logger().info(
                f'MultiPointNav: nuevo goal ({new_x:.2f}, {new_y:.2f})'
            )

    def tick(self):
        if not self.have_odom or self.goal_x is None:
            return

        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        dist = math.hypot(dx, dy)

        if dist < self.goal_tol:
            if not self.goal_reached_published:
                self.cmd_pub.publish(Twist())
                self.reached_pub.publish(Empty())
                self.goal_reached_published = True
                self.get_logger().info(
                    f'MultiPointNav: goal alcanzado ({self.x:.2f}, {self.y:.2f}) '
                    f'dist={dist:.2f}m'
                )
            return

        ang = normalize_angle(math.atan2(dy, dx) - self.theta)

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


def main(args=None):
    rclpy.init(args=args)
    node = MultiPointNav()
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
