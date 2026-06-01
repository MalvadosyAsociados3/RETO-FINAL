"""
EKF de localizacion para el Puzzlebot — Final Challenge Part 1.

Estado:
    x = [x, y, theta]^T  en el frame 'odom'

Prediccion (encoders, dead reckoning):
    v          = r*(wr + wl)/2
    w          = r*(wr - wl)/L
    x_k        = x_{k-1} + v*dt*cos(theta_{k-1})
    y_k        = y_{k-1} + v*dt*sin(theta_{k-1})
    theta_k    = theta_{k-1} + w*dt

    F_k        = [[1, 0, -v*dt*sin(theta)],
                  [0, 1,  v*dt*cos(theta)],
                  [0, 0, 1]]
    Q_k        = grad_w Sigma_delta grad_w^T
    grad_w     = (r*dt/2) * [[c, c],
                             [s, s],
                             [2/L, -2/L]]
    Sigma_delta = diag(kr*|wr|, kl*|wl|)
    P_k        = F_k P_{k-1} F_k^T + Q_k

Correccion (deteccion de ArUco con mapa conocido):
    Para cada marker detectado con id en el mapa (mx, my):

    Observacion: posicion del marker en el frame BASE del robot (2D)
        (X_obs, Y_obs) = R(-cam_yaw) * (z_optical, -x_optical) + (cam_x, cam_y)
                         expresada relativa a base_footprint
    Modelo:
        dxr = (mx-x)*cos(theta) + (my-y)*sin(theta)
        dyr = -(mx-x)*sin(theta) + (my-y)*cos(theta)
        h(state) = (dxr - cam_x, dyr - cam_y)   # marker visto en base, descontando offset de camara

    Jacobiano H = d h / d state:
        H = [[-cos(theta), -sin(theta),  dyr],
             [ sin(theta), -cos(theta), -dxr]]

    Innovacion:    y = z - h(state)
    Ganancia:      K = P H^T (H P H^T + R)^-1
    Actualizacion: state = state + K y
                   P = (I - K H) P

Restriccion del challenge: solo NumPy + libreria estandar.
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

from puzzlebot_msgs.msg import ArucoDetectionArray


def yaw_to_quat(yaw: float):
    """Devuelve (w, x, y, z) para una rotacion pura alrededor de Z."""
    return (math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0))


def normalize_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class EkfLocalisation(Node):

    def __init__(self):
        super().__init__('ekf_localisation')

        # --- Parametros de odometria/ruido ---
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
        self.declare_parameter('kr', 0.02)
        self.declare_parameter('kl', 0.02)

        # --- Mapa de markers (ids, posiciones, yaw) ---
        self.declare_parameter('marker_ids', [0])
        self.declare_parameter('marker_xs', [0.0])
        self.declare_parameter('marker_ys', [0.0])
        self.declare_parameter('marker_yaws', [0.0])

        # --- Camara: offset respecto a base_footprint ---
        self.declare_parameter('camera_x', 0.10)
        self.declare_parameter('camera_y', 0.0)
        self.declare_parameter('camera_yaw', 0.0)

        # --- Frame de la observacion ---
        # 'camera_optical'  -> pose.position con eje optico Z=forward (sim/raw cv2.aruco)
        # 'base'            -> pose.position en frame base (aruco_ros con
        #                      reference_frame=base ya lo transforma en el Jetson)
        self.declare_parameter('observation_frame', 'camera_optical')

        # --- Ruido de la medicion ArUco (varianza por componente) ---
        # Se aumenta con la distancia al marker (mas lejos = mas ruidoso).
        self.declare_parameter('aruco_var_base', 0.01)        # m^2 a 0 m
        self.declare_parameter('aruco_var_per_meter', 0.005)  # m^2 por metro adicional
        self.declare_parameter('aruco_max_distance', 4.0)     # m: descartar medidas mas lejos

        # Cargar parametros
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

        self.cam_x = float(self.get_parameter('camera_x').value)
        self.cam_y = float(self.get_parameter('camera_y').value)
        self.cam_yaw = float(self.get_parameter('camera_yaw').value)
        self.obs_frame = str(self.get_parameter('observation_frame').value).lower()
        if self.obs_frame not in ('camera_optical', 'base'):
            self.get_logger().warn(
                f'observation_frame="{self.obs_frame}" desconocido; uso camera_optical.'
            )
            self.obs_frame = 'camera_optical'

        self.aruco_var_base = float(self.get_parameter('aruco_var_base').value)
        self.aruco_var_per_m = float(self.get_parameter('aruco_var_per_meter').value)
        self.aruco_max_dist = float(self.get_parameter('aruco_max_distance').value)

        # Mapa: dict {id: (mx, my, m_yaw)}
        ids = [int(v) for v in self.get_parameter('marker_ids').value]
        xs = [float(v) for v in self.get_parameter('marker_xs').value]
        ys = [float(v) for v in self.get_parameter('marker_ys').value]
        yaws = [float(v) for v in self.get_parameter('marker_yaws').value]
        if not (len(ids) == len(xs) == len(ys) == len(yaws)):
            self.get_logger().error(
                'marker_ids/xs/ys/yaws deben tener el mismo tamano.'
            )
            self.marker_map = {}
        else:
            self.marker_map = {
                mid: (mx, my, myaw)
                for mid, mx, my, myaw in zip(ids, xs, ys, yaws)
            }

        # --- Estado ---
        self.x = float(self.get_parameter('initial_x').value)
        self.y = float(self.get_parameter('initial_y').value)
        self.theta = float(self.get_parameter('initial_theta').value)
        self.wr = 0.0
        self.wl = 0.0

        # Covarianza 3x3. Arrancamos sin incertidumbre (pose conocida).
        self.Sigma = np.zeros((3, 3))

        # --- QoS ---
        # 'qos' RELIABLE para topicos ROS2 normales (ArUco bridge, /odom).
        # 'sensor_qos' BEST_EFFORT para los encoders del Puzzlebot real:
        # micro_ros_agent publica los encoders con BEST_EFFORT (estandar
        # micro-ROS para sensor data), un subscriber RELIABLE NO los recibe
        # ("New publisher discovered ... incompatible QoS").
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # --- Subs ---
        self.wr_sub = self.create_subscription(Float32, 'wr', self.wr_cb, sensor_qos)
        self.wl_sub = self.create_subscription(Float32, 'wl', self.wl_cb, sensor_qos)
        self.aruco_sub = self.create_subscription(
            ArucoDetectionArray, 'aruco_detections', self.aruco_cb, qos,
        )

        # --- Pubs ---
        self.odom_pub = self.create_publisher(Odometry, 'odom', qos)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        if self.publish_map_odom:
            self.publish_static_map_odom()

        self.timer = self.create_timer(self.dt, self.step)

        self.get_logger().info(
            f'EKF iniciado: r={self.r}, L={self.L}, dt={self.dt:.3f}s, '
            f'init=({self.x:.2f},{self.y:.2f},{self.theta:.2f}), '
            f'markers={list(self.marker_map.keys())}, '
            f'cam=({self.cam_x},{self.cam_y},yaw={self.cam_yaw})'
        )

    # ------------------------------------------------------- TF / msgs --

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

    def publish_odom_and_tf(self):
        now = self.get_clock().now().to_msg()
        qw, qx, qy, qz = yaw_to_quat(self.theta)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = float(self.x)
        odom.pose.pose.position.y = float(self.y)
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.w = float(qw)
        odom.pose.pose.orientation.x = float(qx)
        odom.pose.pose.orientation.y = float(qy)
        odom.pose.pose.orientation.z = float(qz)

        # Empacar covarianza 3x3 -> 6x6 ROS (x,y,z,rx,ry,rz)
        cov6 = np.zeros((6, 6))
        cov6[0, 0] = self.Sigma[0, 0]
        cov6[0, 1] = self.Sigma[0, 1]
        cov6[1, 0] = self.Sigma[1, 0]
        cov6[1, 1] = self.Sigma[1, 1]
        cov6[0, 5] = self.Sigma[0, 2]
        cov6[5, 0] = self.Sigma[2, 0]
        cov6[1, 5] = self.Sigma[1, 2]
        cov6[5, 1] = self.Sigma[2, 1]
        cov6[5, 5] = self.Sigma[2, 2]
        odom.pose.covariance = cov6.flatten().tolist()
        self.odom_pub.publish(odom)

        # TF odom -> base_footprint
        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = float(self.x)
        tf.transform.translation.y = float(self.y)
        tf.transform.translation.z = 0.0
        tf.transform.rotation.w = float(qw)
        tf.transform.rotation.x = float(qx)
        tf.transform.rotation.y = float(qy)
        tf.transform.rotation.z = float(qz)
        self.tf_broadcaster.sendTransform(tf)

    # ------------------------------------------------------- Callbacks --

    def wr_cb(self, msg: Float32):
        self.wr = float(msg.data)

    def wl_cb(self, msg: Float32):
        self.wl = float(msg.data)

    def aruco_cb(self, msg: ArucoDetectionArray):
        # Procesa todas las detecciones que correspondan a un marker conocido.
        for det in msg.detections:
            if det.id not in self.marker_map:
                continue
            self.correct_with_marker(det)

    # ---------------------------------------------------- EKF predict --

    def predict(self):
        v = self.r * (self.wr + self.wl) / 2.0
        w_ang = self.r * (self.wr - self.wl) / self.L

        c = math.cos(self.theta)
        s = math.sin(self.theta)

        F = np.array([
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

        # Sigma_k = F Sigma_{k-1} F^T + Q
        self.Sigma = F @ self.Sigma @ F.T + Q

        # Pose por Euler
        self.x += v * c * self.dt
        self.y += v * s * self.dt
        self.theta = normalize_angle(self.theta + w_ang * self.dt)

    # ---------------------------------------------------- EKF correct --

    def correct_with_marker(self, detection):
        """Aplica la actualizacion EKF con UN marker detectado."""
        mid = int(detection.id)
        if mid not in self.marker_map:
            return
        mx, my, _myaw = self.marker_map[mid]

        # Convertir la pose del marker al frame base 2D del robot.
        #   camera_optical: x=derecha, y=abajo, z=adelante (REP 105).
        #   base (REP 103): x=adelante, y=izquierda.
        # Si la observacion viene en 'base' (aruco_ros con reference_frame=base
        # ya hizo la TF en el Jetson), tomamos x,y directos.
        # Si viene en 'camera_optical', mapeamos (z, -x) y aplicamos la rotacion
        # de la camara cam_yaw respecto a base.
        if self.obs_frame == 'base':
            X_obs = float(detection.pose.position.x)
            Y_obs = float(detection.pose.position.y)
        else:
            zc = float(detection.pose.position.z)
            xc = float(detection.pose.position.x)
            cc = math.cos(self.cam_yaw)
            sc = math.sin(self.cam_yaw)
            X_body_cam = zc      # forward
            Y_body_cam = -xc     # left
            X_obs = cc * X_body_cam - sc * Y_body_cam
            Y_obs = sc * X_body_cam + cc * Y_body_cam
        # Trasladamos al origen de base (no descontamos cam_x, cam_y: la observacion
        # es la posicion del marker MEDIDA por la camara, en frame base, sin offset).
        # Pero la posicion ESPERADA del marker en base parte del marker en world
        # y descuenta la camara, asi que comparamos en el mismo punto: el centro
        # de la camara (cam_x, cam_y).

        z_meas = np.array([X_obs, Y_obs])

        # Distancia desde la camara al marker (en base 2D, sin descontar offset).
        dist = math.hypot(X_obs, Y_obs)
        if dist > self.aruco_max_dist:
            return

        # Posicion esperada del marker EN BASE (mismo punto: visto desde la camara),
        # tomando el marker conocido (mx, my) en world y la pose actual (x,y,theta).
        c = math.cos(self.theta)
        s = math.sin(self.theta)
        dx = mx - self.x
        dy = my - self.y
        # Vector marker-robot en frame base:
        Xp_base = c * dx + s * dy
        Yp_base = -s * dx + c * dy
        # Restamos offset de camara para tener la posicion vista desde la camara:
        X_pred = Xp_base - self.cam_x
        Y_pred = Yp_base - self.cam_y

        h = np.array([X_pred, Y_pred])

        # Jacobiano H = dh / d[x,y,theta]
        H = np.array([
            [-c, -s,  Yp_base],
            [ s, -c, -Xp_base],
        ])

        # Innovacion
        innov = z_meas - h

        # Ruido de la medicion (crece con la distancia).
        sigma2 = self.aruco_var_base + self.aruco_var_per_m * dist
        R = np.array([
            [sigma2, 0.0],
            [0.0, sigma2],
        ])

        # Ganancia K = P H^T (H P H^T + R)^-1
        S = H @ self.Sigma @ H.T + R
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return
        K = self.Sigma @ H.T @ S_inv

        # Actualizar estado
        delta = K @ innov
        self.x += float(delta[0])
        self.y += float(delta[1])
        self.theta = normalize_angle(self.theta + float(delta[2]))

        # Actualizar covarianza
        I3 = np.eye(3)
        self.Sigma = (I3 - K @ H) @ self.Sigma

    # ------------------------------------------------------------ Loop --

    def step(self):
        self.predict()
        self.publish_odom_and_tf()


def main(args=None):
    rclpy.init(args=args)
    node = EkfLocalisation()
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
