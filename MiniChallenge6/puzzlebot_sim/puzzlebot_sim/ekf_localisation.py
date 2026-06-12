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
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster

from std_msgs.msg import Empty as EmptyMsg
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
        self.declare_parameter('aruco_min_distance', 0.30)     # m: descartar medidas muy cerca (solvePnP ruidoso)
        self.declare_parameter('aruco_max_distance', 4.0)     # m: descartar medidas mas lejos
        self.declare_parameter('aruco_mahal_gate', 5.0)      # umbral Mahalanobis: rechazar outliers
        self.declare_parameter('aruco_max_jump', 0.15)       # m: clamp maximo del salto por correccion
        self.declare_parameter('aruco_max_angle_jump', 0.175)  # rad (~10 deg): clamp independiente de heading

        # --- Q dinamico: inflar ruido de theta durante giros ---
        self.declare_parameter('turn_q_multiplier', 8.0)     # factor de inflacion de Q_theta en giros
        self.declare_parameter('turn_threshold', 0.15)       # rad/s: umbral para considerar "girando"

        # --- Correccion de heading por LiDAR (fit de pared) ---
        self.declare_parameter('lidar_heading_enable', True)
        self.declare_parameter('lidar_heading_sector_min_deg', 60.0)   # sector izquierdo
        self.declare_parameter('lidar_heading_sector_max_deg', 120.0)
        self.declare_parameter('lidar_heading_max_range', 0.80)        # solo puntos cercanos
        self.declare_parameter('lidar_heading_min_points', 8)          # minimo para fit
        self.declare_parameter('lidar_heading_max_residual', 0.02)     # m: max error del fit
        self.declare_parameter('lidar_heading_var', 0.01)              # rad^2: varianza de la medicion
        self.declare_parameter('lidar_heading_innov_gate_deg', 15.0)   # max innovacion permitida (deg)
        self.declare_parameter('lidar_heading_cooldown_steps', 25)    # bloqueo post-giro (~5s a 5Hz rate-limited)

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
        self.aruco_min_dist = float(self.get_parameter('aruco_min_distance').value)
        self.aruco_max_dist = float(self.get_parameter('aruco_max_distance').value)
        self.mahal_gate = float(self.get_parameter('aruco_mahal_gate').value)
        self.max_jump = float(self.get_parameter('aruco_max_jump').value)
        self.max_angle_jump = float(self.get_parameter('aruco_max_angle_jump').value)

        # Q dinamico
        self.turn_q_mult = float(self.get_parameter('turn_q_multiplier').value)
        self.turn_thr = float(self.get_parameter('turn_threshold').value)

        # LiDAR heading
        self.lidar_heading_en = bool(self.get_parameter('lidar_heading_enable').value)
        self.lidar_sec_min = math.radians(float(self.get_parameter('lidar_heading_sector_min_deg').value))
        self.lidar_sec_max = math.radians(float(self.get_parameter('lidar_heading_sector_max_deg').value))
        self.lidar_max_range = float(self.get_parameter('lidar_heading_max_range').value)
        self.lidar_min_pts = int(self.get_parameter('lidar_heading_min_points').value)
        self.lidar_max_resid = float(self.get_parameter('lidar_heading_max_residual').value)
        self.lidar_heading_var = float(self.get_parameter('lidar_heading_var').value)
        self.lidar_innov_gate = math.radians(float(self.get_parameter('lidar_heading_innov_gate_deg').value))
        self.lidar_cooldown_steps = int(self.get_parameter('lidar_heading_cooldown_steps').value)
        self._is_turning = False
        self._cooldown_counter = 0

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
        # Pose en MAP frame (estado del EKF, usado por correct_with_marker)
        self.x = float(self.get_parameter('initial_x').value)
        self.y = float(self.get_parameter('initial_y').value)
        self.theta = float(self.get_parameter('initial_theta').value)

        # Pose en ODOM frame (solo encoders, suave, sin saltos)
        self.x_odom = 0.0
        self.y_odom = 0.0
        self.theta_odom = 0.0

        # Offset map->odom (se actualiza con correcciones ArUco)
        self.map_odom_x = self.x
        self.map_odom_y = self.y
        self.map_odom_theta = self.theta

        self.wr = 0.0
        self.wl = 0.0

        # Covarianza 3x3. Incertidumbre inicial realista: la pose de
        # arranque se mide a mano con cinta, no es perfecta.
        self.declare_parameter('initial_cov_xy', 0.05)    # m^2
        self.declare_parameter('initial_cov_theta', 0.1)  # rad^2
        cov_xy = float(self.get_parameter('initial_cov_xy').value)
        cov_th = float(self.get_parameter('initial_cov_theta').value)
        self.Sigma = np.diag([cov_xy, cov_xy, cov_th])

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

        # --- LiDAR state ---
        self._scan_ranges = None
        self._scan_angle_min = 0.0
        self._scan_angle_inc = 0.0

        # --- Subs ---
        self.wr_sub = self.create_subscription(Float32, 'wr', self.wr_cb, sensor_qos)
        self.wl_sub = self.create_subscription(Float32, 'wl', self.wl_cb, sensor_qos)
        self.aruco_sub = self.create_subscription(
            ArucoDetectionArray, 'aruco_detections', self.aruco_cb, qos,
        )
        if self.lidar_heading_en:
            self.scan_sub = self.create_subscription(
                LaserScan, 'scan', self.scan_cb, qos_profile_sensor_data,
            )
        # /initialpose viene del boton "2D Pose Estimate" de RViz: permite
        # resetear la pose del EKF en vivo sin reiniciar el nodo.
        self.initpose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self.initpose_cb, qos,
        )

        # --- Contadores internos ---
        self._last_wr_time = self.get_clock().now()
        self._last_wl_time = self.get_clock().now()
        self._lidar_tick = 0
        self._diag_count = 0

        # --- Pubs ---
        self.odom_pub = self.create_publisher(Odometry, 'odom', qos)
        self.aruco_correction_pub = self.create_publisher(EmptyMsg, 'aruco_correction', qos)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(self.dt, self.step)

        self.get_logger().info(
            f'EKF iniciado: r={self.r}, L={self.L}, dt={self.dt:.3f}s, '
            f'init=({self.x:.2f},{self.y:.2f},{self.theta:.2f}), '
            f'markers={list(self.marker_map.keys())}, '
            f'cam=({self.cam_x},{self.cam_y},yaw={self.cam_yaw})'
        )

    # ------------------------------------------------------- TF / msgs --

    def publish_odom_and_tf(self):
        now = self.get_clock().now().to_msg()

        # --- TF odom -> base_footprint (solo encoders, suave) ---
        qw_o, qx_o, qy_o, qz_o = yaw_to_quat(self.theta_odom)
        tf_odom = TransformStamped()
        tf_odom.header.stamp = now
        tf_odom.header.frame_id = self.odom_frame
        tf_odom.child_frame_id = self.base_frame
        tf_odom.transform.translation.x = float(self.x_odom)
        tf_odom.transform.translation.y = float(self.y_odom)
        tf_odom.transform.translation.z = 0.0
        tf_odom.transform.rotation.w = float(qw_o)
        tf_odom.transform.rotation.x = float(qx_o)
        tf_odom.transform.rotation.y = float(qy_o)
        tf_odom.transform.rotation.z = float(qz_o)

        # --- TF map -> odom (correccion ArUco, dinamico) ---
        if self.publish_map_odom:
            qw_m, qx_m, qy_m, qz_m = yaw_to_quat(self.map_odom_theta)
            tf_map = TransformStamped()
            tf_map.header.stamp = now
            tf_map.header.frame_id = self.world_frame
            tf_map.child_frame_id = self.odom_frame
            tf_map.transform.translation.x = float(self.map_odom_x)
            tf_map.transform.translation.y = float(self.map_odom_y)
            tf_map.transform.translation.z = 0.0
            tf_map.transform.rotation.w = float(qw_m)
            tf_map.transform.rotation.x = float(qx_m)
            tf_map.transform.rotation.y = float(qy_m)
            tf_map.transform.rotation.z = float(qz_m)
            self.tf_broadcaster.sendTransform([tf_odom, tf_map])
        else:
            self.tf_broadcaster.sendTransform(tf_odom)

        # --- Odom msg: pose en MAP frame (para navigation) ---
        qw, qx, qy, qz = yaw_to_quat(self.theta)
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.world_frame
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

    # ------------------------------------------------------- Callbacks --

    def wr_cb(self, msg: Float32):
        self.wr = float(msg.data)
        self._last_wr_time = self.get_clock().now()

    def wl_cb(self, msg: Float32):
        self.wl = float(msg.data)
        self._last_wl_time = self.get_clock().now()

    def scan_cb(self, msg: LaserScan):
        self._scan_ranges = np.asarray(msg.ranges, dtype=np.float32)
        self._scan_angle_min = float(msg.angle_min)
        self._scan_angle_inc = float(msg.angle_increment)

    def aruco_cb(self, msg: ArucoDetectionArray):
        # Procesa todas las detecciones que correspondan a un marker conocido.
        for det in msg.detections:
            if det.id not in self.marker_map:
                continue
            self.correct_with_marker(det)

    def initpose_cb(self, msg: PoseWithCovarianceStamped):
        """Resetea el estado del EKF con la pose del boton '2D Pose Estimate'
        de RViz. Util para arrancar el demo poniendo el robot fisicamente y
        luego marcando su posicion exacta en RViz con un click."""
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        # quaternion -> yaw (asumimos rotacion solo en Z)
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.x = float(p.x)
        self.y = float(p.y)
        self.theta = normalize_angle(yaw)
        # Recalcular offset map->odom con la nueva pose
        self._update_map_odom_offset()
        # Toma la covarianza del mensaje si no es cero, sino reset a 0.
        cov = np.array(msg.pose.covariance, dtype=np.float64).reshape(6, 6)
        if float(np.abs(cov).sum()) > 1e-9:
            self.Sigma = np.array([
                [cov[0, 0], cov[0, 1], cov[0, 5]],
                [cov[1, 0], cov[1, 1], cov[1, 5]],
                [cov[5, 0], cov[5, 1], cov[5, 5]],
            ])
        else:
            self.Sigma = np.zeros((3, 3))
        self.get_logger().info(
            f'EKF reset por /initialpose: '
            f'x={self.x:.2f} y={self.y:.2f} theta={math.degrees(self.theta):.1f}deg'
        )

    # ---------------------------------------------------- EKF predict --

    def _odom_to_map(self):
        """Calcula la pose en MAP frame a partir de odom pose + offset map->odom."""
        c_mo = math.cos(self.map_odom_theta)
        s_mo = math.sin(self.map_odom_theta)
        self.x = c_mo * self.x_odom - s_mo * self.y_odom + self.map_odom_x
        self.y = s_mo * self.x_odom + c_mo * self.y_odom + self.map_odom_y
        self.theta = normalize_angle(self.theta_odom + self.map_odom_theta)

    def _update_map_odom_offset(self):
        """Recalcula offset map->odom a partir de la pose actual en map y odom."""
        self.map_odom_theta = normalize_angle(self.theta - self.theta_odom)
        c_mo = math.cos(self.map_odom_theta)
        s_mo = math.sin(self.map_odom_theta)
        self.map_odom_x = self.x - (c_mo * self.x_odom - s_mo * self.y_odom)
        self.map_odom_y = self.y - (s_mo * self.x_odom + c_mo * self.y_odom)

    def predict(self):
        v = self.r * (self.wr + self.wl) / 2.0
        w_ang = self.r * (self.wr - self.wl) / self.L

        # Actualizar pose en ODOM frame (solo encoders, suave)
        c_o = math.cos(self.theta_odom)
        s_o = math.sin(self.theta_odom)
        self.x_odom += v * c_o * self.dt
        self.y_odom += v * s_o * self.dt
        self.theta_odom = normalize_angle(self.theta_odom + w_ang * self.dt)

        # Calcular pose en MAP frame
        self._odom_to_map()

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

        # Q dinamico: inflar ruido de theta cuando el robot esta girando
        # (contra-rotacion o giro agresivo causa patinaje -> encoders mienten)
        if abs(w_ang) > self.turn_thr:
            Q[2, 2] *= self.turn_q_mult

        # Sigma_k = F Sigma_{k-1} F^T + Q
        self.Sigma = F @ self.Sigma @ F.T + Q

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
        if dist < self.aruco_min_dist:
            return
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
        innov_mag = math.hypot(float(innov[0]), float(innov[1]))

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

        # Gate de Mahalanobis: rechazar outliers
        mahal2 = float(innov @ S_inv @ innov)
        mahal = math.sqrt(mahal2)
        if mahal > self.mahal_gate:
            self.get_logger().warn(
                f'[ARUCO] id={mid} RECHAZADO: Mahalanobis={mahal:.2f} > gate={self.mahal_gate:.1f}, '
                f'innov=({innov[0]:.3f},{innov[1]:.3f}) |innov|={innov_mag:.3f}'
            )
            return

        K = self.Sigma @ H.T @ S_inv

        # Actualizar estado
        delta = K @ innov

        # Clamp: limitar salto maximo en posicion para evitar teleports
        raw_jump = math.hypot(float(delta[0]), float(delta[1]))
        if raw_jump > self.max_jump:
            scale = self.max_jump / raw_jump
            delta[0] *= scale
            delta[1] *= scale
            delta[2] *= scale
            self.get_logger().warn(
                f'[ARUCO] id={mid} jump CLAMPED: {raw_jump:.3f}m -> {self.max_jump:.3f}m'
            )

        # Clamp independiente de heading
        raw_angle_jump = abs(float(delta[2]))
        if raw_angle_jump > self.max_angle_jump:
            delta[2] *= self.max_angle_jump / raw_angle_jump
            self.get_logger().warn(
                f'[ARUCO] id={mid} angle CLAMPED: {math.degrees(raw_angle_jump):.1f}deg -> {math.degrees(self.max_angle_jump):.1f}deg'
            )

        old_x, old_y, old_th = self.x, self.y, self.theta
        self.x += float(delta[0])
        self.y += float(delta[1])
        self.theta = normalize_angle(self.theta + float(delta[2]))

        # Recalcular offset map->odom (la correccion mueve map->odom, no odom->base)
        self._update_map_odom_offset()

        # Actualizar covarianza
        I3 = np.eye(3)
        self.Sigma = (I3 - K @ H) @ self.Sigma

        # Notificar que hubo correccion ArUco aceptada
        self.aruco_correction_pub.publish(EmptyMsg())

        # Diagnostico: log cada correccion ArUco
        jump = math.hypot(float(delta[0]), float(delta[1]))
        self.get_logger().info(
            f'[ARUCO] id={mid} dist={dist:.2f}m '
            f'meas=({z_meas[0]:.3f},{z_meas[1]:.3f}) '
            f'pred=({h[0]:.3f},{h[1]:.3f}) '
            f'innov=({innov[0]:.3f},{innov[1]:.3f}) |innov|={innov_mag:.3f} mahal={mahal:.2f} '
            f'jump=({delta[0]:.3f},{delta[1]:.3f},{math.degrees(delta[2]):.1f}°) |jump|={jump:.3f}m '
            f'pose: ({old_x:.2f},{old_y:.2f})->({self.x:.2f},{self.y:.2f}) '
            f'θ: {math.degrees(old_th):.1f}°->{math.degrees(self.theta):.1f}°'
        )

    # ------------------------------------------------- LiDAR heading --

    def correct_heading_from_lidar(self):
        """Ajusta una linea a los puntos del sector izquierdo del LiDAR
        usando PCA. Las paredes del laberinto estan alineadas a 0/90/180/270.
        El angulo de la pared da theta absoluto modulo 90 deg.

        Rate-limited a ~5Hz y con gate de innovacion para evitar que un
        snap al multiplo de 90 deg incorrecto refuerce el error."""
        # Rate limit: solo cada 10 ciclos (~5Hz a 50Hz)
        self._lidar_tick += 1
        if self._lidar_tick % 10 != 0:
            return

        if self._scan_ranges is None or self._scan_angle_inc == 0.0:
            return

        # Detectar si estamos girando
        w_ang = abs(self.r * (self.wr - self.wl) / self.L)
        if w_ang > self.turn_thr:
            if not self._is_turning:
                self._is_turning = True
                self.get_logger().info('[LIDAR-HEADING] paused: robot turning')
            return  # NO corregir durante giros
        else:
            if self._is_turning:
                # Acaba de terminar el giro: cooldown para dar prioridad a ArUco
                self._is_turning = False
                self._cooldown_counter = self.lidar_cooldown_steps
                self.get_logger().info(
                    f'[LIDAR-HEADING] turn ended, cooldown {self.lidar_cooldown_steps} steps')

        # Cooldown post-giro: no corregir, dejar que ArUco haga la correccion grande
        if self._cooldown_counter > 0:
            self._cooldown_counter -= 1
            return

        ranges = self._scan_ranges
        n = len(ranges)

        # Extraer puntos del sector izquierdo en frame base (x,y)
        idx_lo = int(round((self.lidar_sec_min - self._scan_angle_min) / self._scan_angle_inc))
        idx_hi = int(round((self.lidar_sec_max - self._scan_angle_min) / self._scan_angle_inc))
        idx_lo = max(0, min(n - 1, idx_lo))
        idx_hi = max(0, min(n - 1, idx_hi))

        indices = np.arange(idx_lo, idx_hi + 1)
        angles = self._scan_angle_min + indices * self._scan_angle_inc
        r = ranges[idx_lo:idx_hi + 1]

        # Filtrar: solo puntos validos y cercanos
        valid = np.isfinite(r) & (r > 0.05) & (r < self.lidar_max_range)
        n_valid = int(np.sum(valid))
        if n_valid < self.lidar_min_pts:
            self.get_logger().debug(
                f'[LIDAR-HEADING] skip: only {n_valid} pts < {self.lidar_min_pts} min')
            return

        angles = angles[valid]
        r = r[valid]
        px = r * np.cos(angles)
        py = r * np.sin(angles)

        # PCA: direccion principal de los puntos = direccion de la pared
        cx = float(np.mean(px))
        cy = float(np.mean(py))
        dx = px - cx
        dy = py - cy
        cov_xx = float(np.mean(dx * dx))
        cov_xy = float(np.mean(dx * dy))
        cov_yy = float(np.mean(dy * dy))

        # Angulo del primer componente principal (direccion de la pared en base)
        wall_angle_base = 0.5 * math.atan2(2.0 * cov_xy, cov_xx - cov_yy)

        # Calidad del fit: ratio de varianza explicada (eigenvalue ratio)
        trace = cov_xx + cov_yy
        det = cov_xx * cov_yy - cov_xy * cov_xy
        disc = math.sqrt(max(0.0, trace * trace / 4.0 - det))
        lam1 = trace / 2.0 + disc
        lam2 = trace / 2.0 - disc
        if lam1 < 1e-9:
            return
        linearity = 1.0 - lam2 / lam1  # 1.0 = linea perfecta, 0.0 = circulo
        if linearity < 0.9:
            self.get_logger().debug(
                f'[LIDAR-HEADING] skip: linearity={linearity:.2f} < 0.9 pts={n_valid}')
            return  # puntos no forman una linea clara

        # Residual: distancia RMS de los puntos a la linea PCA
        # Normal de la pared: (-sin(wall_angle_base), cos(wall_angle_base))
        nx = -math.sin(wall_angle_base)
        ny = math.cos(wall_angle_base)
        dists = dx * nx + dy * ny
        residual = float(np.sqrt(np.mean(dists * dists)))
        if residual > self.lidar_max_resid:
            self.get_logger().debug(
                f'[LIDAR-HEADING] skip: residual={residual:.4f}m > {self.lidar_max_resid}m')
            return

        # Para cada cardinal, calcular que theta implicaria y elegir el mas
        # cercano al theta actual (respeta la intencion del giro de encoders)
        best_theta = None
        best_snap = None
        best_err = float('inf')
        for ref in [0.0, math.pi / 2, math.pi, -math.pi / 2]:
            theta_cand = normalize_angle(ref - wall_angle_base)
            err = abs(normalize_angle(theta_cand - self.theta))
            if err < best_err:
                best_err = err
                best_theta = theta_cand
                best_snap = ref

        # Verificar que el snap es razonable (pared debe estar cerca de un cardinal)
        wall_angle_world = normalize_angle(self.theta + wall_angle_base)
        snap_check = abs(normalize_angle(wall_angle_world - best_snap))
        if snap_check > math.radians(15.0):
            self.get_logger().debug(
                f'[LIDAR-HEADING-REJECT] snap_err={math.degrees(snap_check):.1f}deg '
                f'wall_world={math.degrees(wall_angle_world):.1f}deg pts={n_valid}')
            return  # pared no alineada a ejes

        theta_measured = best_theta
        innov_theta = normalize_angle(theta_measured - self.theta)

        # Gate de innovacion como red de seguridad
        if abs(innov_theta) > self.lidar_innov_gate:
            self.get_logger().info(
                f'[LIDAR-HEADING-GATE] innov={math.degrees(innov_theta):.1f}deg > '
                f'gate={math.degrees(self.lidar_innov_gate):.0f}deg '
                f'θ={math.degrees(self.theta):.1f}° snap={math.degrees(best_snap):.0f}° '
                f'wall_base={math.degrees(wall_angle_base):.1f}deg pts={n_valid}')
            return

        # EKF correction con H = [0, 0, 1] (solo mide theta)
        H = np.array([[0.0, 0.0, 1.0]])
        R_lidar = np.array([[self.lidar_heading_var]])

        S = H @ self.Sigma @ H.T + R_lidar
        K = self.Sigma @ H.T / float(S[0, 0])

        delta = (K * innov_theta).flatten()

        old_th = self.theta
        self.x += float(delta[0])
        self.y += float(delta[1])
        self.theta = normalize_angle(self.theta + float(delta[2]))
        self._update_map_odom_offset()

        self.Sigma = (np.eye(3) - K @ H) @ self.Sigma

        # Log solo si la correccion es significativa
        if abs(math.degrees(innov_theta)) > 1.0:
            self.get_logger().info(
                f'[LIDAR-HEADING] wall_base={math.degrees(wall_angle_base):.1f}deg '
                f'snap={math.degrees(best_snap):.0f}deg '
                f'innov={math.degrees(innov_theta):.1f}deg '
                f'θ: {math.degrees(old_th):.1f}°->{math.degrees(self.theta):.1f}° '
                f'resid={residual:.4f}m lin={linearity:.2f} pts={n_valid}'
            )

    # ------------------------------------------------------------ Loop --

    def step(self):
        # Watchdog: si no llegan encoders en >0.5s, asumir parado
        now = self.get_clock().now()
        stale = False
        for attr in ('_last_wr_time', '_last_wl_time'):
            if hasattr(self, attr):
                dt_enc = (now - getattr(self, attr)).nanoseconds * 1e-9
                if dt_enc > 0.5:
                    stale = True
        if stale:
            self.wr = 0.0
            self.wl = 0.0

        self.predict()
        if self.lidar_heading_en:
            self.correct_heading_from_lidar()
        self.publish_odom_and_tf()

        # Diagnostico periodico (~2s)
        self._diag_count += 1
        if self._diag_count % 100 == 0:
            v = self.r * (self.wr + self.wl) / 2.0
            w_ang = self.r * (self.wr - self.wl) / self.L
            self.get_logger().info(
                f'[EKF-DIAG] pose=({self.x:.2f},{self.y:.2f},{math.degrees(self.theta):.1f}deg) '
                f'wr={self.wr:.3f} wl={self.wl:.3f} v={v:.3f} w={w_ang:.4f} '
                f'cov_th={self.Sigma[2,2]:.4f}'
            )


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
