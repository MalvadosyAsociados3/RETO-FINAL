"""
Adapter aruco_ros -> puzzlebot_msgs.

aruco_ros/marker_publisher publica aruco_msgs/MarkerArray con cada detection
como (id, PoseWithCovariance, confidence). Este nodo lo convierte a nuestro
puzzlebot_msgs/ArucoDetectionArray para que el EKF no dependa de aruco_msgs.

Por que un adapter:
  - El EKF de puzzlebot_sim usa nuestro tipo (no requiere ros-humble-aruco-msgs
    instalado para construir).
  - El robot real ya corre aruco_ros (per la presentacion del simulador del
    Puzzlebot), asi que reutilizamos su deteccion (que vive en el Jetson y por
    WiFi solo viajan los Pose messages, no las imagenes).

Para que este nodo CORRA hace falta que aruco_msgs si este instalado en la PC:
    sudo apt install ros-humble-aruco-msgs

Subscribe:
  /marker_publisher/markers   (aruco_msgs/MarkerArray)

Publica:
  /aruco_detections           (puzzlebot_msgs/ArucoDetectionArray)
"""

import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from puzzlebot_msgs.msg import ArucoDetection, ArucoDetectionArray

try:
    from aruco_msgs.msg import MarkerArray as ArucoRosMarkerArray
    HAS_ARUCO_MSGS = True
except Exception as e:
    ArucoRosMarkerArray = None
    HAS_ARUCO_MSGS = False
    _IMPORT_ERROR = str(e)


class ArucoRosBridge(Node):

    def __init__(self):
        super().__init__('aruco_ros_bridge')

        if not HAS_ARUCO_MSGS:
            self.get_logger().error(
                'aruco_msgs no esta instalado en esta PC. '
                'Instala con: sudo apt install ros-humble-aruco-msgs'
            )
            self.get_logger().error(f'Import error: {_IMPORT_ERROR}')
            return

        self.declare_parameter('input_topic', '/marker_publisher/markers')
        self.declare_parameter('output_topic', '/aruco_detections')

        in_topic = str(self.get_parameter('input_topic').value)
        out_topic = str(self.get_parameter('output_topic').value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.pub = self.create_publisher(ArucoDetectionArray, out_topic, qos)
        self.sub = self.create_subscription(
            ArucoRosMarkerArray, in_topic, self.cb, qos,
        )

        self.get_logger().info(
            f'aruco_ros_bridge: {in_topic} -> {out_topic} '
            f'(aruco_msgs/MarkerArray -> puzzlebot_msgs/ArucoDetectionArray)'
        )

    def cb(self, msg):
        out = ArucoDetectionArray()
        out.header = msg.header  # mismo frame_id (segun como configures aruco_ros)
        for m in msg.markers:
            d = ArucoDetection()
            d.id = int(m.id)
            d.pose = m.pose.pose   # PoseWithCovariance -> Pose
            out.detections.append(d)
        self.pub.publish(out)


def main(args=None):
    if not HAS_ARUCO_MSGS:
        sys.stderr.write(
            'aruco_ros_bridge: ros-humble-aruco-msgs no esta instalado.\n'
            'Instalalo en la PC con: sudo apt install ros-humble-aruco-msgs\n'
        )
    rclpy.init(args=args)
    node = ArucoRosBridge()
    try:
        if HAS_ARUCO_MSGS:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
