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
import copy
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

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
        # Filtro opcional: solo deja pasar markers cuyo id este en esta lista.
        # Si la lista contiene solo [-1] (default), NO filtra (deja pasar todo).
        # Util para eliminar falsos positivos de aruco_ros (e.g., un 249 que
        # aparece encima de un 702 real por desenfoque).
        # Tipo declarado explicitamente como INTEGER_ARRAY (sin esto, rclpy
        # asume BYTE_ARRAY para [] vacio y crashea al leer el yaml).
        self.declare_parameter(
            'allowed_ids',
            [-1],
            ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER_ARRAY),
        )
        # Corrector de escala para compensar calibracion de camara mala.
        # Si aruco_ros reporta tvec.z = 38 cm cuando la distancia real son
        # 26 cm, el ratio es 26/38 = 0.684. Pose corregida = pose * 0.684.
        # En condiciones normales (calibracion buena) usar 1.0.
        self.declare_parameter('scale_correction', 1.0)

        in_topic = str(self.get_parameter('input_topic').value)
        out_topic = str(self.get_parameter('output_topic').value)
        self.scale = float(self.get_parameter('scale_correction').value)
        ids_param = list(self.get_parameter('allowed_ids').value)
        # [-1] => sentinel "no filtrar"
        if ids_param == [-1] or len(ids_param) == 0:
            self.allowed_ids = None
        else:
            self.allowed_ids = set(int(v) for v in ids_param)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.pub = self.create_publisher(ArucoDetectionArray, out_topic, qos)
        self.sub = self.create_subscription(
            ArucoRosMarkerArray, in_topic, self.cb, qos,
        )

        filter_info = ('sin filtro' if not self.allowed_ids
                       else f'filtrando a ids {sorted(self.allowed_ids)}')
        self.get_logger().info(
            f'aruco_ros_bridge: {in_topic} -> {out_topic} ({filter_info})'
        )

    def cb(self, msg):
        out = ArucoDetectionArray()
        out.header = msg.header  # mismo frame_id (segun como configures aruco_ros)
        for m in msg.markers:
            mid = int(m.id)
            if self.allowed_ids and mid not in self.allowed_ids:
                continue
            d = ArucoDetection()
            d.id = mid
            # copia profunda: NO mutar el Pose del mensaje de entrada (es del
            # buffer de DDS y puede ser reusado / leido por otros callbacks).
            d.pose = copy.deepcopy(m.pose.pose)   # PoseWithCovariance -> Pose
            # Aplica correccion de escala a la posicion (no a la orientacion).
            # Compensa errores de calibracion de camara.
            if self.scale != 1.0:
                d.pose.position.x = float(d.pose.position.x * self.scale)
                d.pose.position.y = float(d.pose.position.y * self.scale)
                d.pose.position.z = float(d.pose.position.z * self.scale)
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
