"""
Nodo que expone los setpoints (waypoints) como parametros y los alimenta al
nodo de control uno por uno, via un handshake por tópicos.

Flujo:
  point_generator  --/current_goal-->    control
  point_generator  <--/goal_reached--    control   (cuando control termina)

Al recibir /goal_reached, avanza al siguiente waypoint (o loopea / termina).

Subscribe:
  /goal_reached (std_msgs/Empty) - confirmacion del control

Publica:
  /current_goal (geometry_msgs/PoseStamped, TRANSIENT_LOCAL) - setpoint actual
  /planned_path (nav_msgs/Path, TRANSIENT_LOCAL)             - trayectoria completa
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Empty
from rcl_interfaces.msg import SetParametersResult


class PointGenerator(Node):

    def __init__(self):
        super().__init__('point_generator')

        # --- Parametros ---
        self.declare_parameter('waypoints_x', [1.0, 1.0, 0.0, 0.0])
        self.declare_parameter('waypoints_y', [0.0, 1.0, 1.0, 0.0])
        self.declare_parameter('loop_trajectory', True)
        self.declare_parameter('startup_delay', 1.5)
        self.declare_parameter('frame_id', 'odom')

        self._wx_raw = list(self.get_parameter('waypoints_x').value)
        self._wy_raw = list(self.get_parameter('waypoints_y').value)
        self.loop = bool(self.get_parameter('loop_trajectory').value)
        self.startup_delay = float(self.get_parameter('startup_delay').value)
        self.frame_id = str(self.get_parameter('frame_id').value)

        if len(self._wx_raw) != len(self._wy_raw) or len(self._wx_raw) == 0:
            self.get_logger().error('waypoints_x y waypoints_y deben tener mismo tamano (>0).')
            self.waypoints = []
        else:
            self.waypoints = [(float(a), float(b)) for a, b in zip(self._wx_raw, self._wy_raw)]

        # --- Estado ---
        self.wp_idx = 0
        self.finished = False

        # --- QoS ---
        latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        vol_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # --- Pub / Sub ---
        self.goal_pub = self.create_publisher(PoseStamped, 'current_goal', latched)
        self.path_pub = self.create_publisher(Path, 'planned_path', latched)
        self.reached_sub = self.create_subscription(
            Empty, 'goal_reached', self.reached_cb, vol_qos
        )

        self.publish_planned_path()

        # Retrasa la publicacion del primer goal para dar tiempo al control a suscribirse
        self.startup_timer = self.create_timer(self.startup_delay, self._send_first_goal)

        # Actualizacion en caliente de parametros
        self.add_on_set_parameters_callback(self._on_set_params)

        self.get_logger().info(
            f'PointGenerator iniciado con {len(self.waypoints)} waypoints, loop={self.loop}.'
        )

    # ---------- Primer goal ----------
    def _send_first_goal(self):
        # Timer one-shot
        self.startup_timer.cancel()
        self.publish_current_goal()

    # ---------- Publicacion de goal / path ----------
    def publish_current_goal(self):
        if not self.waypoints or self.finished:
            return
        gx, gy = self.waypoints[self.wp_idx]
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = self.frame_id
        ps.pose.position.x = float(gx)
        ps.pose.position.y = float(gy)
        ps.pose.orientation.w = 1.0
        self.goal_pub.publish(ps)
        self.get_logger().info(f'Publicando WP{self.wp_idx}: ({gx:.2f}, {gy:.2f})')

    def publish_planned_path(self):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.frame_id
        for wx, wy in self.waypoints:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(wx)
            ps.pose.position.y = float(wy)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        if self.loop and self.waypoints:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(self.waypoints[0][0])
            ps.pose.position.y = float(self.waypoints[0][1])
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.path_pub.publish(path)

    # ---------- Callback de confirmacion ----------
    def reached_cb(self, _msg: Empty):
        if self.finished or not self.waypoints:
            return
        self.get_logger().info(f'WP{self.wp_idx} confirmado por el control.')
        self.wp_idx += 1
        if self.wp_idx >= len(self.waypoints):
            if self.loop:
                self.wp_idx = 0
                self.get_logger().info('Reiniciando trayectoria (loop).')
            else:
                self.finished = True
                self.get_logger().info('Trayectoria completada.')
                return
        self.publish_current_goal()

    # ---------- Update en caliente ----------
    def _on_set_params(self, params):
        new_wx = list(self._wx_raw)
        new_wy = list(self._wy_raw)
        new_loop = self.loop
        for p in params:
            if p.name == 'waypoints_x':
                new_wx = list(p.value)
            elif p.name == 'waypoints_y':
                new_wy = list(p.value)
            elif p.name == 'loop_trajectory':
                new_loop = bool(p.value)

        self._wx_raw = new_wx
        self._wy_raw = new_wy
        self.loop = new_loop

        if len(new_wx) == len(new_wy) and len(new_wx) > 0:
            self.waypoints = [(float(a), float(b)) for a, b in zip(new_wx, new_wy)]
            self.wp_idx = 0
            self.finished = False
            self.publish_planned_path()
            self.publish_current_goal()
            self.get_logger().info(
                f'Trayectoria actualizada ({len(self.waypoints)} waypoints, loop={self.loop}).'
            )
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    node = PointGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
