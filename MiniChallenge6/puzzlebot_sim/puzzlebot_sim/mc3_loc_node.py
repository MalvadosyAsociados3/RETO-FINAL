import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Float64MultiArray, String
import math
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np                 
from visualization_msgs.msg import Marker



class LocNode(Node):

    def __init__(self):
        super().__init__('loc_node')

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Suscriptores a velocidades de ruedas.
        self.sub_der = self.create_subscription(
            Float32,
            '/VelocityEncR',
            self.der_callback,
            qos
        )

        self.sub_izq = self.create_subscription(
            Float32,
            '/VelocityEncL',
            self.izq_callback,
            qos
        )

        self.sub_aruco = self.create_subscription(
            Float64MultiArray,
            '/aruco_measurement',
            self.aruco_callback,
            10
        )

        # Publicadores.
        self.pub_odom = self.create_publisher(Odometry, 'odom', 10)
        self.pub_ellipse = self.create_publisher(Marker, 'covariance_ellipse', 10)
        self.pub_aruco_correction = self.create_publisher(
            String,
            'aruco_correction',
            10
        )

        self.ns = self.get_namespace().strip('/')
        self.odom_frame = f"{self.ns}/odom" if self.ns else "odom"
        self.base_frame = f"{self.ns}/base_footprint" if self.ns else "base_footprint"

        # Parámetros posición inicial.
        self.declare_parameter('x0', 0.28)
        self.declare_parameter('y0', -0.23)
        self.declare_parameter('theta0', 0.0)

        self.x = self.get_parameter('x0').value
        self.y = self.get_parameter('y0').value
        self.theta = self.get_parameter('theta0').value

        # TF broadcaster.
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer.
        self.timer = self.create_timer(0.05, self.update)

        # Velocidades ruedas.
        self.v_der = 0.0
        self.v_izq = 0.0

        # Parámetros físicos robot.
        self.L = 0.19
        self.R = 0.05

        # Tiempo.
        self.last_time = self.get_clock().now()

        # Matriz de covarianza inicial.
        self.declare_parameter('sigma_x0', 0.05)
        self.declare_parameter('sigma_y0', 0.05)
        self.declare_parameter('sigma_theta0', 0.02)
        sigma_x0 = self.get_parameter('sigma_x0').value
        sigma_y0 = self.get_parameter('sigma_y0').value
        sigma_theta0 = self.get_parameter('sigma_theta0').value

        self.Sigma = np.diag([sigma_x0, sigma_y0, sigma_theta0])

        # Constantes de error de odometría.
        self.kr = 0.1
        self.kl = 0.1

        # Ruido de medición ArUco.
        self.declare_parameter('aruco_range_var', 0.05)
        self.declare_parameter('aruco_bearing_var', 0.02)
        self.aruco_range_var = self.get_parameter('aruco_range_var').value
        self.aruco_bearing_var = self.get_parameter('aruco_bearing_var').value

        # Matriz de ruido de observación R_obs
        self.R_obs = np.diag([self.aruco_range_var, self.aruco_bearing_var])

        # Buffer para medición asíncrona de ArUco
        self.aruco_measurement = None

        # Debug de covarianza publicada en /odom.
        self.declare_parameter('debug_odom_covariance', True)
        self.declare_parameter('debug_odom_covariance_every', 1)
        self.debug_odom_covariance = self.get_parameter('debug_odom_covariance').value
        self.debug_odom_covariance_every = int(
            self.get_parameter('debug_odom_covariance_every').value
        )
        self.odom_debug_count = 0

        # Parámetros de corrección con ArUco.
        # SUBIDOS del original (era 0.05/0.10/3.0) — el robot real tiene
        # mas drift de encoders que el simulado, asi que necesitamos
        # correcciones mas grandes y mas frecuentes para que la EKF
        # alcance la realidad fisica.
        self.declare_parameter('aruco_correction_gain', 1.0)
        self.declare_parameter('aruco_max_position_correction', 0.12)
        self.declare_parameter('aruco_max_theta_correction', 0.18)
        self.aruco_correction_gain = self.get_parameter('aruco_correction_gain').value
        self.aruco_max_position_correction = self.get_parameter(
            'aruco_max_position_correction'
        ).value
        self.aruco_max_theta_correction = self.get_parameter(
            'aruco_max_theta_correction'
        ).value

        self.aruco_reuse_time = 0.5  # antes 3.0 — reusa marker cada 0.5s
        self.last_marker_correction_time = {}

        #self.get_logger().info(
         #   f"LocNode EKF iniciado en ({self.x:.2f}, {self.y:.2f}, {self.theta:.2f})"
        #)


    def update(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time
        if dt <= 0.0:
            return

        # ── 1. PREDICCIÓN (Integración exacta + Covarianza) ───────────────
        self._prediction_step(dt)
 
        # ── 2. CORRECCIÓN (Solo si llegó medición de ArUco) ───────────────
        if self.aruco_measurement is not None:
            self._correction_step(self.aruco_measurement)
            self.aruco_measurement = None   # Consumida
 
        # ── 3. PUBLICACIONES ──────────────────────────────────────────────
        self._publish_odom(current_time)
        self.publish_covariance_ellipse(current_time)


    def _prediction_step(self, dt):
        # Cinemática.
        v = self.R * (self.v_der + self.v_izq) / 2.0 
        w = self.R * (self.v_der - self.v_izq) / self.L

        # #Integracion exacta para mejor precisión en giros
        if abs(w) < 1e-6:
            self.x += v * math.cos(self.theta) * dt
            self.y += v * math.sin(self.theta) * dt
        else:
            self.x += (v / w) * (math.sin(self.theta + w*dt) - math.sin(self.theta))
            self.y += -(v / w) * (math.cos(self.theta + w*dt) - math.cos(self.theta))

        self.theta += w * dt
        self.theta = self.normalize_angle(self.theta)

        # Jacobiano Fk (o Hk en tu primer script).
        F = np.array([
            [1, 0, -dt * v * math.sin(self.theta)],
            [0, 1,  dt * v * math.cos(self.theta)],
            [0, 0, 1]
        ])

        # Construir Matriz ΣΔk.
        Sigma_delta = np.array([
            [self.kr * abs(self.v_der), 0],
            [0, self.kl * abs(self.v_izq)]
        ])

        # Construir Matriz ∇ωk.
        nabla_w = (self.R * dt / 2.0) * np.array([
            [math.cos(self.theta), math.cos(self.theta)],
            [math.sin(self.theta), math.sin(self.theta)],
            [2/self.L, -2/self.L]
        ])

        # Construir Matriz Qk.
        Q = nabla_w @ Sigma_delta @ nabla_w.T

        # Propagación de covarianza.
        self.Sigma = F @ self.Sigma @ F.T + Q


    def _correction_step(self, meas):
        marker_id = meas['id']
        marker_x = meas['mx']
        marker_y = meas['my']
        measured_range = meas['range']
        measured_bearing = meas['bearing']

        now = self.get_clock().now()

        if marker_id in self.last_marker_correction_time:
            dt_marker = (now - self.last_marker_correction_time[marker_id]).nanoseconds * 1e-9

            if dt_marker < self.aruco_reuse_time:
                self.get_logger().info(
                    f'EKF ArUco ID={marker_id} ignorado temporalmente | '
                    f'dt={dt_marker:.2f}s',
                    throttle_duration_sec=2.0
                )
                return

        dx = marker_x - self.x
        dy = marker_y - self.y
        q = dx**2 + dy**2

        if q < 1e-9:
            self.get_logger().warn(
                f'Corrección ArUco omitida: el robot está demasiado cerca del marker {marker_id}'
            )
            return

        expected_range = math.sqrt(q)
        expected_bearing = math.atan2(dy, dx) - self.theta
        expected_bearing = self.normalize_angle(expected_bearing)

        # Vector de innovación (residual)
        innovation = np.array([
            measured_range - expected_range,
            self.normalize_angle(measured_bearing - expected_bearing)
        ])

        # Jacobiano de la observación (G o H)
        G = np.array([
            [-dx / expected_range, -dy / expected_range, 0.0],
            [ dy / q,              -dx / q,             -1.0]
        ])

        # Espacio de innovación y Ganancia de Kalman
        Z = G @ self.Sigma @ G.T + self.R_obs
        K = self.Sigma @ G.T @ np.linalg.inv(Z)

        # Cálculo de la corrección
        state = np.array([self.x, self.y, self.theta])
        raw_correction = K @ innovation
        limited_correction = self.limit_aruco_correction(raw_correction)
        applied_correction = self.aruco_correction_gain * limited_correction
        
        # Guardar posición anterior antes de la corrección
        prev_x = self.x
        prev_y = self.y
        prev_theta = self.theta

        # Aplicar corrección al estado
        corrected_state = state + applied_correction
        self.x = corrected_state[0]
        self.y = corrected_state[1]
        self.theta = self.normalize_angle(corrected_state[2])

        # Actualizar Matriz de Covarianza
        I = np.eye(3)
        self.Sigma = (I - self.aruco_correction_gain * K @ G) @ self.Sigma
        self.Sigma = 0.5 * (self.Sigma + self.Sigma.T)  # Garantizar simetría
        self.last_marker_correction_time[marker_id] = now

        # Impresión clara de la corrección de posición
        correction_msg = String()
        correction_msg.data = (
            f'================= CORRECCIÓN ARUCO =================\n'
            f'  ID ArUco Detectado: {marker_id}\n'
            f'  Posición ArUco en Mapa (x, y, theta): ({marker_x:.4f}, {marker_y:.4f})\n'
            f'  Posición Anterior del Robot (x, y, theta): ({prev_x:.4f}, {prev_y:.4f}, {prev_theta:.4f})\n'
            f'  Posición Nueva del Robot Corregida (x, y, theta): ({self.x:.4f}, {self.y:.4f}, {self.theta:.4f})\n'
            f'===================================================\n'
        )
        self.pub_aruco_correction.publish(correction_msg)


    def _publish_odom(self, current_time):
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        # Posición.
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y

        # Orientación.
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        # Llenado de la matriz de covarianza 6x6.
        cov = [0.0] * 36
        cov[0] = self.Sigma[0, 0]   # x
        cov[1] = self.Sigma[0, 1]   # xy
        cov[6] = self.Sigma[1, 0]   # yx
        cov[7] = self.Sigma[1, 1]   # y
        cov[35] = self.Sigma[2, 2]  # theta

        odom.pose.covariance = cov

        self.debug_published_odom_covariance(cov)

        # Publicar Odom
        self.pub_odom.publish(odom)

        # Publicar Transformada (TF)
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        t.transform.rotation.z = odom.pose.pose.orientation.z
        t.transform.rotation.w = odom.pose.pose.orientation.w

        self.tf_broadcaster.sendTransform(t)


    def publish_covariance_ellipse(self, current_time):
        P_xy = self.Sigma[0:2, 0:2]

        if np.any(np.isnan(P_xy)) or np.any(np.isinf(P_xy)):
            return

        try:
            eigenvalues, eigenvectors = np.linalg.eig(P_xy)
        except np.linalg.LinAlgError:
            return

        eigenvalues = np.real(eigenvalues)
        eigenvectors = np.real(eigenvectors)
        eigenvalues = np.maximum(eigenvalues, 0.0)

        order = eigenvalues.argsort()[::-1]
        eigenvalues = eigenvalues[order]
        eigenvectors = eigenvectors[:, order]

        angle = math.atan2(eigenvectors[1, 0], eigenvectors[0, 0])

        marker = Marker()
        marker.header.stamp = current_time.to_msg()
        marker.header.frame_id = self.odom_frame
        marker.ns = 'ekf_covariance'
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        marker.pose.position.x = self.x
        marker.pose.position.y = self.y
        marker.pose.position.z = 0.02

        marker.pose.orientation.z = math.sin(angle / 2.0)
        marker.pose.orientation.w = math.cos(angle / 2.0)

        # Elipse 1-sigma aproximada
        marker.scale.x = 2.0 * math.sqrt(eigenvalues[0])
        marker.scale.y = 2.0 * math.sqrt(eigenvalues[1])
        marker.scale.z = 0.01

        marker.color.r = 1.0
        marker.color.g = 0.4
        marker.color.b = 0.0
        marker.color.a = 0.45

        self.pub_ellipse.publish(marker)


    def aruco_callback(self, msg):
        if len(msg.data) < 6:
            self.get_logger().warn("aruco_measurement incompleto, ignorando.")
            return
 
        # Almacenamos la medición para procesarla en el siguiente ciclo del timer de forma segura
        self.aruco_measurement = {
            'id':      int(msg.data[0]),
            'mx':      msg.data[1],
            'my':      msg.data[2],
            'range':   msg.data[4],
            'bearing': msg.data[5],
        }
 
        #self.get_logger().info(
         #   f"[ArUco] Recibido ID={self.aruco_measurement['id']} "
          #  f"range={self.aruco_measurement['range']:.3f} m  "
           # f"bearing={self.aruco_measurement['bearing']:.3f} rad"
        #)


    def der_callback(self, msg):
        self.v_der = msg.data


    def izq_callback(self, msg):
        self.v_izq = msg.data


    def limit_aruco_correction(self, correction):
        limited = correction.copy()
        position_norm = math.hypot(limited[0], limited[1])

        if position_norm > self.aruco_max_position_correction:
            scale = self.aruco_max_position_correction / position_norm
            limited[0] *= scale
            limited[1] *= scale

        limited[2] = max(
            -self.aruco_max_theta_correction,
            min(self.aruco_max_theta_correction, limited[2])
        )
        return limited


    def debug_published_odom_covariance(self, cov):
        if not self.debug_odom_covariance:
            return

        self.odom_debug_count += 1
        if self.debug_odom_covariance_every > 1:
            if self.odom_debug_count % self.debug_odom_covariance_every != 0:
                return
        
        #self.get_logger().info(
         #   'Odom covariance publicada | '
          #  f'xx={cov[0]:.6f}, xy={cov[1]:.6f}, '
           # f'yx={cov[6]:.6f}, yy={cov[7]:.6f}, '
            #f'theta_theta={cov[35]:.6f}'
        #)


    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))


def main(args=None):
    rclpy.init(args=args)
    node = LocNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
