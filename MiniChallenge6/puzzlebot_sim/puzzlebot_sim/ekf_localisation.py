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
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster

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
        self.declare_parameter('aruco_mahal_gate', 5.0)      # umbral Mahalanobis: rechazar outliers
        self.declare_parameter('aruco_max_jump', 0.15)       # m: clamp maximo del salto por correccion

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
        self.mahal_gate = float(self.get_parameter('aruco_mahal_gate').value)
        self.max_jump = float(self.get_parameter('aruco_max_jump').value)

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

        # --- Subs ---
        self.wr_sub = self.create_subscription(Float32, 'wr', self.wr_cb, sensor_qos)
        self.wl_sub = self.create_subscription(Float32, 'wl', self.wl_cb, sensor_qos)
        self.aruco_sub = self.create_subscription(
            ArucoDetectionArray, 'aruco_detections', self.aruco_cb, qos,
        )
        # /initialpose viene del boton "2D Pose Estimate" de RViz: permite
        # resetear la pose del EKF en vivo sin reiniciar el nodo.
        self.initpose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self.initpose_cb, qos,
        )

        # --- Pubs ---
        self.odom_pub = self.create_publisher(Odometry, 'odom', qos)
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

    def wl_cb(self, msg: Float32):
        self.wl = float(msg.data)

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

        old_x, old_y, old_th = self.x, self.y, self.theta
        self.x += float(delta[0])
        self.y += float(delta[1])
        self.theta = normalize_angle(self.theta + float(delta[2]))

        # Recalcular offset map->odom (la correccion mueve map->odom, no odom->base)
        self._update_map_odom_offset()

        # Actualizar covarianza
        I3 = np.eye(3)
        self.Sigma = (I3 - K @ H) @ self.Sigma

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
