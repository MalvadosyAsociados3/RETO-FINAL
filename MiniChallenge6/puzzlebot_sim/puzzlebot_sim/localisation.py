"""
MINI CHALLENGE 5 - LOCALISATION + UNCERTAINTY PROPAGATION

Subscribe:
  /wr  (std_msgs/Float32) -> velocidad angular de la rueda derecha
  /wl  (std_msgs/Float32) -> velocidad angular de la rueda izquierda

Publica:
  /odom  (nav_msgs/Odometry) -> pose, twist y MATRIZ DE COVARIANZA estimadas
  TF: world -> odom              (estatico, identidad)
  TF: odom  -> base_footprint    (dinamico, dead-reckoning)

Modelo cinematico (Euler, dt fijo):
    v_k          = r*(wr_k + wl_k)/2
    w_k          = r*(wr_k - wl_k)/L
    x_k     = x_{k-1}     + v_k * dt * cos(theta_{k-1})
    y_k     = y_{k-1}     + v_k * dt * sin(theta_{k-1})
    theta_k = theta_{k-1} + w_k * dt

Propagacion de covarianza 3x3 (s = [x, y, theta]):
    Sigma_k = H_k * Sigma_{k-1} * H_k^T + Q_k
    Q_k     = grad_w * Sigma_Delta * grad_w^T

    H_k = [[1, 0, -v_k*dt*sin(theta_{k-1})],
           [0, 1,  v_k*dt*cos(theta_{k-1})],
           [0, 0, 1]]

    grad_w = (r*dt/2) * [[ cos(theta_{k-1}),  cos(theta_{k-1})],
                         [ sin(theta_{k-1}),  sin(theta_{k-1})],
                         [        2/L,              -2/L     ]]

    Sigma_Delta = diag(kr*|wr|, kl*|wl|)   (ruido proporcional a la velocidad)

Solo se usa NumPy + libreria estandar (regla de MC5).
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


def yaw_to_quat(yaw: float):
    """Devuelve (w, x, y, z) para una rotacion pura alrededor de Z."""
    return (math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0))


class Localisation(Node):

    def __init__(self):
        super().__init__('localisation')

        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_base', 0.19)
        self.declare_parameter('update_rate', 50.0)
        self.declare_parameter('publish_map_odom_tf', True)
        self.declare_parameter('world_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_theta', 0.0)
        # Constantes de ruido por rueda (Task 2: a calibrar).
        self.declare_parameter('kr', 0.02)
        self.declare_parameter('kl', 0.02)

        self.r = float(self.get_parameter('wheel_radius').value)
        self.L = float(self.get_parameter('wheel_base').value)
        rate = float(self.get_parameter('update_rate').value)
        self.publish_map_odom = bool(self.get_parameter('publish_map_odom_tf').value)
        self.world_frame = str(self.get_parameter('world_frame').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.kr = float(self.get_parameter('kr').value)
        self.kl = float(self.get_parameter('kl').value)
        self.dt = 1.0 / rate

        self.x = float(self.get_parameter('initial_x').value)
        self.y = float(self.get_parameter('initial_y').value)
        self.theta = float(self.get_parameter('initial_theta').value)
        self.wr = 0.0
        self.wl = 0.0

        # Covarianza inicial: arrancamos sin incertidumbre (pose conocida).
        self.Sigma = np.zeros((3, 3))

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.wr_sub = self.create_subscription(Float32, 'wr', self.wr_cb, qos)
        self.wl_sub = self.create_subscription(Float32, 'wl', self.wl_cb, qos)

        self.odom_pub = self.create_publisher(Odometry, 'odom', qos)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        if self.publish_map_odom:
            self.publish_static_map_odom()

        self.timer = self.create_timer(self.dt, self.step)
        self.get_logger().info(
            f'Localisation iniciada: r={self.r} m, L={self.L} m, dt={self.dt:.3f} s, '
            f'init=({self.x:.2f},{self.y:.2f},{self.theta:.2f}), '
            f'world={self.world_frame}, odom={self.odom_frame}, base={self.base_frame}, '
            f'kr={self.kr}, kl={self.kl}'
        )

    def publish_static_map_odom(self):
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.world_frame
        tf.child_frame_id = self.odom_frame
        tf.transform.translation.x = 0.0
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = 0.0
        tf.transform.rotation.x = 0.0
        tf.transform.rotation.y = 0.0
        tf.transform.rotation.z = 0.0
        tf.transform.rotation.w = 1.0
        self.static_broadcaster.sendTransform(tf)

    def wr_cb(self, msg: Float32):
        self.wr = float(msg.data)

    def wl_cb(self, msg: Float32):
        self.wl = float(msg.data)

    def step(self):
        # Velocidades del cuerpo a partir de las velocidades de rueda.
        v = self.r * (self.wr + self.wl) / 2.0
        w_ang = self.r * (self.wr - self.wl) / self.L

        # Jacobianos evaluados en la pose ANTERIOR (Euler hacia adelante).
        c = math.cos(self.theta)
        s = math.sin(self.theta)

        H = np.array([
            [1.0, 0.0, -v * self.dt * s],
            [0.0, 1.0,  v * self.dt * c],
            [0.0, 0.0, 1.0],
        ])

        grad_w = 0.5 * self.r * self.dt * np.array([
            [c, c],
            [s, s],
            [2.0 / self.L, -2.0 / self.L],
        ])

        Sigma_delta = np.array([
            [self.kr * abs(self.wr), 0.0],
            [0.0, self.kl * abs(self.wl)],
        ])

        Q = grad_w @ Sigma_delta @ grad_w.T

        # Propagacion: Sigma_k = H Sigma_{k-1} H^T + Q_k
        self.Sigma = H @ self.Sigma @ H.T + Q

        # Pose por Euler.
        self.x += v * c * self.dt
        self.y += v * s * self.dt
        self.theta += w_ang * self.dt
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        now = self.get_clock().now().to_msg()
        q = yaw_to_quat(self.theta)

        # --- Odometry ---
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = float(self.x)
        odom.pose.pose.position.y = float(self.y)
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.w = float(q[0])
        odom.pose.pose.orientation.x = float(q[1])
        odom.pose.pose.orientation.y = float(q[2])
        odom.pose.pose.orientation.z = float(q[3])
        odom.twist.twist.linear.x = float(v)
        odom.twist.twist.angular.z = float(w_ang)
        odom.pose.covariance = self._pack_pose_covariance()
        self.odom_pub.publish(odom)

        # --- TF odom -> base_footprint ---
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

    def _pack_pose_covariance(self):
        """
        Mete la Sigma 3x3 (x, y, yaw) en la 6x6 (x, y, z, roll, pitch, yaw)
        del mensaje Odometry. Layout row-major: indice = fila*6 + col.
        """
        cov = [0.0] * 36
        s = self.Sigma
        # x-x, x-y, x-yaw
        cov[0]  = float(s[0, 0]); cov[1]  = float(s[0, 1]); cov[5]  = float(s[0, 2])
        # y-x, y-y, y-yaw
        cov[6]  = float(s[1, 0]); cov[7]  = float(s[1, 1]); cov[11] = float(s[1, 2])
        # yaw-x, yaw-y, yaw-yaw
        cov[30] = float(s[2, 0]); cov[31] = float(s[2, 1]); cov[35] = float(s[2, 2])
        return cov


def main(args=None):
    rclpy.init(args=args)
    node = Localisation()
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
