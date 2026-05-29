"""
Detector de markers ArUco para simulacion (y para el PC en el robot real
si la imagen llega por topico).

Subscribe:
  /camera        (sensor_msgs/Image)
  /camera_info   (sensor_msgs/CameraInfo)

Publica:
  /aruco_detections     (puzzlebot_msgs/ArucoDetectionArray) con la pose de
                        cada marker en el frame de la camara optica.
  /aruco_image          (sensor_msgs/Image) con los markers dibujados (debug)

Diccionario:
  Se prueban varios diccionarios y se queda con el primero que detecte algo.
  El usuario puede forzar uno con el parametro `dictionary_name`.

Restricciones del challenge: se permite OpenCV/ArUco y NumPy.
"""

import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

from puzzlebot_msgs.msg import ArucoDetection, ArucoDetectionArray


DICT_CANDIDATES = [
    ('DICT_ARUCO_ORIGINAL', cv2.aruco.DICT_ARUCO_ORIGINAL),
    ('DICT_4X4_50', cv2.aruco.DICT_4X4_50),
    ('DICT_5X5_50', cv2.aruco.DICT_5X5_50),
    ('DICT_6X6_50', cv2.aruco.DICT_6X6_50),
    ('DICT_7X7_50', cv2.aruco.DICT_7X7_50),
    ('DICT_4X4_250', cv2.aruco.DICT_4X4_250),
    ('DICT_5X5_250', cv2.aruco.DICT_5X5_250),
    ('DICT_6X6_250', cv2.aruco.DICT_6X6_250),
]


class ArucoDetector(Node):

    def __init__(self):
        super().__init__('aruco_detector')

        self.declare_parameter('marker_size', 0.18)  # lado del marker en metros
        self.declare_parameter('dictionary_name', '')  # vacio = auto-detectar
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('image_topic', 'camera')
        self.declare_parameter('info_topic', 'camera_info')

        self.marker_size = float(self.get_parameter('marker_size').value)
        forced = str(self.get_parameter('dictionary_name').value).strip()
        self.publish_dbg = bool(self.get_parameter('publish_debug_image').value)

        img_topic = str(self.get_parameter('image_topic').value)
        info_topic = str(self.get_parameter('info_topic').value)

        # Diccionarios candidatos
        self.dict_candidates = list(DICT_CANDIDATES)
        if forced:
            forced_id = getattr(cv2.aruco, forced, None)
            if forced_id is not None:
                self.dict_candidates = [(forced, forced_id)]
            else:
                self.get_logger().warn(
                    f'dictionary_name="{forced}" no existe en cv2.aruco; usando auto.'
                )

        # Detectores. Se construye el ArucoDetector una sola vez con el dict
        # que arroje al menos una deteccion (auto).
        self.detector = None
        self.dict_used_name = None

        self.K = None
        self.D = None
        self.bridge = CvBridge()

        info_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.image_sub = self.create_subscription(
            Image, img_topic, self.image_cb, qos_profile_sensor_data,
        )
        self.info_sub = self.create_subscription(
            CameraInfo, info_topic, self.info_cb, info_qos,
        )

        self.det_pub = self.create_publisher(
            ArucoDetectionArray, 'aruco_detections', 10,
        )
        if self.publish_dbg:
            self.dbg_pub = self.create_publisher(Image, 'aruco_image', 1)
        else:
            self.dbg_pub = None

        self.get_logger().info(
            f'ArucoDetector: marker_size={self.marker_size} m, '
            f'forzado="{forced or "auto"}", subs={img_topic}/{info_topic}'
        )

    # -------------------------------------------------------- Callbacks

    def info_cb(self, msg: CameraInfo):
        # K (3x3 fila-major en msg.k) y distorsion (D)
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.D = np.array(msg.d, dtype=np.float64).reshape(-1)
        if self.D.size == 0:
            self.D = np.zeros(5, dtype=np.float64)

    def image_cb(self, msg: Image):
        if self.K is None:
            return  # esperar camera_info
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge fallo: {e}', throttle_duration_sec=2.0)
            return

        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)

        corners, ids = self._detect(gray)
        if ids is None or len(ids) == 0:
            self._publish(msg, [], cv_img)
            return

        # Estimar pose de cada marker en el frame optico de la camara.
        # cv2.aruco.estimatePoseSingleMarkers fue deprecado en OpenCV 4.7+.
        # Usamos solvePnP directo con los corners del marker (object_points es
        # un cuadrado de lado marker_size centrado en el origen).
        s = self.marker_size / 2.0
        obj_points = np.array([
            [-s,  s, 0.0],
            [ s,  s, 0.0],
            [ s, -s, 0.0],
            [-s, -s, 0.0],
        ], dtype=np.float32)

        detections = []
        for i, marker_id in enumerate(ids.flatten().tolist()):
            img_points = corners[i].reshape(-1, 2).astype(np.float32)
            ok, rvec, tvec = cv2.solvePnP(
                obj_points, img_points, self.K, self.D,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
            if not ok:
                continue

            tvec = tvec.flatten()
            R, _ = cv2.Rodrigues(rvec)
            qx, qy, qz, qw = rotation_matrix_to_quat(R)

            d = ArucoDetection()
            d.id = int(marker_id)
            d.pose.position.x = float(tvec[0])
            d.pose.position.y = float(tvec[1])
            d.pose.position.z = float(tvec[2])
            d.pose.orientation.x = float(qx)
            d.pose.orientation.y = float(qy)
            d.pose.orientation.z = float(qz)
            d.pose.orientation.w = float(qw)
            detections.append(d)

        self._publish(msg, detections, cv_img, corners=corners, ids=ids)

    # --------------------------------------------------------- Detector

    def _detect(self, gray):
        """Devuelve (corners, ids). Si self.detector aun no se eligio, prueba
        cada diccionario y se queda con el primero que detecte algo."""
        if self.detector is not None:
            return self.detector.detectMarkers(gray)[:2]

        for name, dn in self.dict_candidates:
            d = cv2.aruco.getPredefinedDictionary(dn)
            params = cv2.aruco.DetectorParameters()
            det = cv2.aruco.ArucoDetector(d, params)
            corners, ids, _ = det.detectMarkers(gray)
            if ids is not None and len(ids) > 0:
                self.detector = det
                self.dict_used_name = name
                self.get_logger().info(
                    f'Diccionario detectado: {name} (ids={ids.flatten().tolist()})'
                )
                return corners, ids
        return None, None

    # ---------------------------------------------------------- Publish

    def _publish(self, src_msg, detections, cv_img, corners=None, ids=None):
        arr = ArucoDetectionArray()
        arr.header = src_msg.header  # mismo frame_id (camera optical) y stamp
        arr.detections = detections
        self.det_pub.publish(arr)

        if self.dbg_pub is not None:
            dbg = cv_img.copy()
            if corners is not None and ids is not None:
                cv2.aruco.drawDetectedMarkers(dbg, corners, ids)
            try:
                out = self.bridge.cv2_to_imgmsg(dbg, encoding='bgr8')
                out.header = src_msg.header
                self.dbg_pub.publish(out)
            except Exception:
                pass


def rotation_matrix_to_quat(R):
    """3x3 -> (qx, qy, qz, qw). Sin scipy."""
    t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s
    return qx, qy, qz, qw


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
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
