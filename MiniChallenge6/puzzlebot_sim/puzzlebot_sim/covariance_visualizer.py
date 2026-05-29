"""
Publica el elipsoide de confianza 2D de la pose como un visualization_msgs/Marker
para que RViz lo pinte y se vea crecer/encoger con el tiempo.

Subscribe:
  /odom  (nav_msgs/Odometry)  con covarianza en pose.covariance (6x6 row-major).

Publica:
  /pose_covariance_marker  (visualization_msgs/MarkerArray):
      - id 0: CYLINDER plano en (x, y) con escala (2*sigma_a, 2*sigma_b) y la
              orientacion del eje mayor (95% confianza, k=2 desv. estandar).
      - id 1: ARROW del heading.
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


def yaw_to_quat(yaw: float):
    return (math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0))


class CovarianceVisualizer(Node):

    def __init__(self):
        super().__init__('covariance_visualizer')

        self.declare_parameter('sigma_scale', 2.0)  # 2-sigma ~ 95%
        self.declare_parameter('min_axis', 0.02)    # m, evita marker invisible
        self.declare_parameter('marker_frame', '')  # vacio = usar header del odom

        self.k = float(self.get_parameter('sigma_scale').value)
        self.min_axis = float(self.get_parameter('min_axis').value)
        self.frame_param = str(self.get_parameter('marker_frame').value).strip()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(Odometry, 'odom', self.odom_cb, qos)
        self.pub = self.create_publisher(
            MarkerArray, 'pose_covariance_marker', 1,
        )

        self.get_logger().info(
            f'CovarianceVisualizer: sigma_scale={self.k}, min_axis={self.min_axis} m'
        )

    def odom_cb(self, msg: Odometry):
        # Extraer bloque 2x2 (x, y) de la covarianza 6x6.
        cov6 = np.array(msg.pose.covariance, dtype=np.float64).reshape(6, 6)
        Pxy = cov6[0:2, 0:2]

        # Eigendescomposicion para sacar ejes y rotacion.
        try:
            evals, evecs = np.linalg.eigh(Pxy)
        except np.linalg.LinAlgError:
            return

        evals = np.clip(evals, 0.0, None)
        sigma_minor = math.sqrt(float(evals[0]))
        sigma_major = math.sqrt(float(evals[1]))
        vec_major = evecs[:, 1]
        yaw_ellipse = math.atan2(float(vec_major[1]), float(vec_major[0]))

        sx = max(self.k * sigma_major, self.min_axis)
        sy = max(self.k * sigma_minor, self.min_axis)

        frame = self.frame_param if self.frame_param else msg.header.frame_id
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        arr = MarkerArray()

        # CYLINDER plano que representa la elipse 2D.
        ell = Marker()
        ell.header.stamp = msg.header.stamp
        ell.header.frame_id = frame
        ell.ns = 'pose_cov'
        ell.id = 0
        ell.type = Marker.CYLINDER
        ell.action = Marker.ADD
        ell.pose.position.x = x
        ell.pose.position.y = y
        ell.pose.position.z = 0.01
        qw, qx, qy, qz = yaw_to_quat(yaw_ellipse)
        ell.pose.orientation.w = qw
        ell.pose.orientation.x = qx
        ell.pose.orientation.y = qy
        ell.pose.orientation.z = qz
        ell.scale.x = sx
        ell.scale.y = sy
        ell.scale.z = 0.01
        ell.color.r = 0.1
        ell.color.g = 0.5
        ell.color.b = 1.0
        ell.color.a = 0.35
        arr.markers.append(ell)

        # ARROW del heading.
        arrow = Marker()
        arrow.header.stamp = msg.header.stamp
        arrow.header.frame_id = frame
        arrow.ns = 'pose_cov'
        arrow.id = 1
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose.position.x = x
        arrow.pose.position.y = y
        arrow.pose.position.z = 0.05
        arrow.pose.orientation = msg.pose.pose.orientation
        arrow.scale.x = 0.30
        arrow.scale.y = 0.04
        arrow.scale.z = 0.04
        arrow.color.r = 1.0
        arrow.color.g = 0.2
        arrow.color.b = 0.2
        arrow.color.a = 0.9
        arr.markers.append(arrow)

        self.pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = CovarianceVisualizer()
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
