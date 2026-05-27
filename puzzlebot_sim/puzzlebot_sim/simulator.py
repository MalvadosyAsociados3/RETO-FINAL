"""

Subscribe:
  /cmd_vel  (geometry_msgs/Twist)   -> comandos de velocidad lineal y angular

Publica:
  /wr  (std_msgs/Float32)           -> velocidad angular de la rueda derecha
  /wl  (std_msgs/Float32)           -> velocidad angular de la rueda izquierda
  /joint_states (sensor_msgs/JointState) -> posiciones de las ruedas (para RViz)
  /sim_pose (geometry_msgs/PoseStamped) -> pose simulada "ground truth"
  TF: odom -> base_footprint   (solo si publish_tf=true)

Modelo cinematico directo (forward):
    v = r*(wr + wl)/2
    w = r*(wr - wl)/L

Modelo cinematico inverso (usado aqui para simular el actuador):
    wr = (v + w*L/2)/r
    wl = (v - w*L/2)/r

donde r = radio de rueda, L = separacion entre ruedas.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster


def yaw_to_quat(yaw: float):
    """Devuelve (w, x, y, z) para una rotacion pura alrededor de Z."""
    return (math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0))


class PuzzlebotSimulator(Node):

    def __init__(self):
        super().__init__('puzzlebot_sim')

        # --- Parametros ---
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_base', 0.19)
        self.declare_parameter('update_rate', 50.0)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_theta', 0.0)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')

        self.r = float(self.get_parameter('wheel_radius').value)
        self.L = float(self.get_parameter('wheel_base').value)
        rate = float(self.get_parameter('update_rate').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.dt = 1.0 / rate

        # --- Estado ---
        self.v_cmd = 0.0
        self.w_cmd = 0.0
        self.x = float(self.get_parameter('initial_x').value)
        self.y = float(self.get_parameter('initial_y').value)
        self.theta = float(self.get_parameter('initial_theta').value)
        self.phi_r = 0.0
        self.phi_l = 0.0

        # --- QoS ---
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # --- Pub / Sub ---
        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_cb, qos)
        self.wr_pub = self.create_publisher(Float32, 'wr', qos)
        self.wl_pub = self.create_publisher(Float32, 'wl', qos)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos)
        self.pose_pub = self.create_publisher(PoseStamped, 'sim_pose', qos)
        self.gt_odom_pub = self.create_publisher(Odometry, 'sim_pose_odom', qos)

        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.gt_tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(self.dt, self.step)
        self.get_logger().info(
            f'Simulator iniciado: r={self.r} m, L={self.L} m, dt={self.dt:.3f} s, '
            f'publish_tf={self.publish_tf}, init=({self.x:.2f},{self.y:.2f},{self.theta:.2f}), '
            f'odom_frame={self.odom_frame}, base_frame={self.base_frame}'
        )

    def cmd_cb(self, msg: Twist):
        self.v_cmd = float(msg.linear.x)
        self.w_cmd = float(msg.angular.z)

    def step(self):
        # --- Cinematica inversa: velocidades de ruedas ---
        wr = (self.v_cmd + self.w_cmd * self.L / 2.0) / self.r
        wl = (self.v_cmd - self.w_cmd * self.L / 2.0) / self.r

        # --- Cinematica directa: reconstrucción de v, w ---
        v = self.r * (wr + wl) / 2.0
        w = self.r * (wr - wl) / self.L

        # --- Integracion de la pose ---
        self.x += v * math.cos(self.theta) * self.dt
        self.y += v * math.sin(self.theta) * self.dt
        self.theta += w * self.dt
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # --- Integracion del angulo de las ruedas ---
        self.phi_r += wr * self.dt
        self.phi_l += wl * self.dt

        now = self.get_clock().now().to_msg()

        # --- Publicar velocidades de ruedas ---
        mr = Float32(); mr.data = float(wr); self.wr_pub.publish(mr)
        ml = Float32(); ml.data = float(wl); self.wl_pub.publish(ml)

        # --- Publicar joint states ---
        js = JointState()
        js.header.stamp = now
        js.name = ['wheel_r_joint', 'wheel_l_joint']
        js.position = [float(self.phi_r), float(self.phi_l)]
        js.velocity = [float(wr), float(wl)]
        js.effort = []
        self.joint_pub.publish(js)

        # --- Publicar pose simulada como PoseStamped  ---
        q = yaw_to_quat(self.theta)  # (w,x,y,z)
        ps = PoseStamped()
        ps.header.stamp = now
        ps.header.frame_id = self.odom_frame
        ps.pose.position.x = float(self.x)
        ps.pose.position.y = float(self.y)
        ps.pose.position.z = 0.0
        ps.pose.orientation.w = float(q[0])
        ps.pose.orientation.x = float(q[1])
        ps.pose.orientation.y = float(q[2])
        ps.pose.orientation.z = float(q[3])
        self.pose_pub.publish(ps)

        # --- TF odom -> base_footprint ---
        if self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header.stamp = now
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = float(self.x)
            tf.transform.translation.y = float(self.y)
            tf.transform.translation.z = 0.0
            tf.transform.rotation.w = float(q[0])
            tf.transform.rotation.x = float(q[1])
            tf.transform.rotation.y = float(q[2])
            tf.transform.rotation.z = float(q[3])
            self.tf_broadcaster.sendTransform(tf)

        # --- Ground-truth Odometry (sim_pose_odom) ---
        gt_odom = Odometry()
        gt_odom.header.stamp = now
        gt_odom.header.frame_id = 'map'
        gt_odom.child_frame_id = 'sim_base_footprint'
        gt_odom.pose.pose.position.x = float(self.x)
        gt_odom.pose.pose.position.y = float(self.y)
        gt_odom.pose.pose.position.z = 0.0
        gt_odom.pose.pose.orientation.w = float(q[0])
        gt_odom.pose.pose.orientation.x = float(q[1])
        gt_odom.pose.pose.orientation.y = float(q[2])
        gt_odom.pose.pose.orientation.z = float(q[3])
        gt_odom.pose.covariance = [0.0] * 36
        gt_odom.twist.twist.linear.x = float(v)
        gt_odom.twist.twist.angular.z = float(w)
        gt_odom.twist.covariance = [0.0] * 36
        self.gt_odom_pub.publish(gt_odom)

        # --- TF dinámico: map -> sim_base_footprint ---
        gt_tf = TransformStamped()
        gt_tf.header.stamp = now
        gt_tf.header.frame_id = 'map'
        gt_tf.child_frame_id = 'sim_base_footprint'
        gt_tf.transform.translation.x = float(self.x)
        gt_tf.transform.translation.y = float(self.y)
        gt_tf.transform.translation.z = 0.0
        gt_tf.transform.rotation.w = float(q[0])
        gt_tf.transform.rotation.x = float(q[1])
        gt_tf.transform.rotation.y = float(q[2])
        gt_tf.transform.rotation.z = float(q[3])
        self.gt_tf_broadcaster.sendTransform(gt_tf)


def main(args=None):
    rclpy.init(args=args)
    node = PuzzlebotSimulator()
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
