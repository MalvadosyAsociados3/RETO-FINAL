"""
Nodo para correr experimentos cronometrados de Task 2 (calibracion kr, kl).

Envia cmd_vel con trayectorias predefinidas (recta, rotacion, cuadrado)
y se detiene automaticamente al terminar, dejando un hold_time para
visualizar el elipsoide final en RViz.

Uso:
  ros2 run puzzlebot_sim experiment_runner --ros-args -p experiment:=straight
  ros2 run puzzlebot_sim experiment_runner --ros-args -p experiment:=rotate
  ros2 run puzzlebot_sim experiment_runner --ros-args -p experiment:=square
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class ExperimentRunner(Node):

    def __init__(self):
        super().__init__('experiment_runner')

        self.declare_parameter('experiment', 'straight')
        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('distance', 1.0)
        self.declare_parameter('rotation', 2.0 * math.pi)
        self.declare_parameter('hold_time', 2.0)

        self.experiment = str(self.get_parameter('experiment').value)
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.distance = float(self.get_parameter('distance').value)
        self.rotation = float(self.get_parameter('rotation').value)
        self.hold_time = float(self.get_parameter('hold_time').value)

        self.declare_parameter('report_kr', 0.0)
        self.declare_parameter('report_kl', 0.0)
        self.report_kr = float(self.get_parameter('report_kr').value)
        self.report_kl = float(self.get_parameter('report_kl').value)

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.last_gt = None
        self.last_odom = None
        self.gt_sub = self.create_subscription(
            Odometry, 'sim_pose_odom', self._gt_cb, qos)
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self._odom_cb, qos)

        # Build phase list: [(v, w, duration_s), ...]
        self.phases = self._build_phases()
        self.phase_idx = 0
        self.phase_elapsed = 0.0

        self.dt = 1.0 / 50.0
        self.total_duration = sum(d for _, _, d in self.phases)

        self._log_start()
        self.timer = self.create_timer(self.dt, self._tick)

    def _gt_cb(self, msg: Odometry):
        self.last_gt = msg

    def _odom_cb(self, msg: Odometry):
        self.last_odom = msg

    @staticmethod
    def _quat_to_yaw(q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    @staticmethod
    def _normalize_angle(a):
        return math.atan2(math.sin(a), math.cos(a))

    def _build_phases(self):
        if self.experiment == 'straight':
            t_move = self.distance / self.linear_speed
            return [
                (self.linear_speed, 0.0, t_move),
                (0.0, 0.0, self.hold_time),
            ]

        if self.experiment == 'rotate':
            t_rot = self.rotation / self.angular_speed
            return [
                (0.0, self.angular_speed, t_rot),
                (0.0, 0.0, self.hold_time),
            ]

        if self.experiment == 'square':
            t_side = self.distance / self.linear_speed
            t_turn = (math.pi / 2.0) / self.angular_speed
            phases = []
            for i in range(4):
                phases.append((self.linear_speed, 0.0, t_side))
                phases.append((0.0, self.angular_speed, t_turn))
            phases.append((0.0, 0.0, self.hold_time))
            return phases

        self.get_logger().error(
            f"Experimento '{self.experiment}' no reconocido. "
            "Valores validos: straight, rotate, square"
        )
        return [(0.0, 0.0, self.hold_time)]

    def _log_start(self):
        move_duration = self.total_duration - self.hold_time
        self.get_logger().info(
            f"Iniciando experimento '{self.experiment}' — "
            f"duracion movimiento: {move_duration:.2f}s, "
            f"hold: {self.hold_time:.1f}s"
        )

    def _tick(self):
        if self.phase_idx >= len(self.phases):
            self._finish()
            return

        v, w, duration = self.phases[self.phase_idx]

        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.cmd_pub.publish(msg)

        self.phase_elapsed += self.dt
        if self.phase_elapsed >= duration:
            self._log_phase_end(v, w, duration)
            self.phase_idx += 1
            self.phase_elapsed = 0.0

    def _log_phase_end(self, v, w, duration):
        if v == 0.0 and w == 0.0:
            self.get_logger().info(f"Hold terminado ({duration:.1f}s)")
        elif w == 0.0:
            self.get_logger().info(
                f"Tramo recto terminado: {v:.2f} m/s x {duration:.2f}s "
                f"= {v * duration:.2f}m"
            )
        else:
            self.get_logger().info(
                f"Rotacion terminada: {w:.2f} rad/s x {duration:.2f}s "
                f"= {math.degrees(w * duration):.1f} deg"
            )

    def _finish(self):
        self.timer.cancel()
        # Publish stop
        self.cmd_pub.publish(Twist())

        move_duration = self.total_duration - self.hold_time

        if self.experiment == 'straight':
            summary = (
                f"Experimento 'straight' terminado. "
                f"Duracion: {move_duration:.2f}s. "
                f"Velocidad: {self.linear_speed} m/s. "
                f"Distancia objetivo: {self.distance} m"
            )
        elif self.experiment == 'rotate':
            summary = (
                f"Experimento 'rotate' terminado. "
                f"Duracion: {move_duration:.2f}s. "
                f"Velocidad angular: {self.angular_speed} rad/s. "
                f"Rotacion objetivo: {math.degrees(self.rotation):.1f} deg"
            )
        elif self.experiment == 'square':
            summary = (
                f"Experimento 'square' terminado. "
                f"Duracion: {move_duration:.2f}s. "
                f"Lado: {self.distance} m a {self.linear_speed} m/s. "
                f"Giros: 90 deg a {self.angular_speed} rad/s"
            )
        else:
            summary = f"Experimento '{self.experiment}' terminado."

        self.get_logger().info(summary)
        self._print_report()
        raise SystemExit(0)


    def _get_target(self):
        """Devuelve (x, y, theta) esperado al final del experimento."""
        if self.experiment == 'straight':
            return (self.distance, 0.0, 0.0)
        if self.experiment == 'rotate':
            target_yaw = self._normalize_angle(self.rotation)
            return (0.0, 0.0, target_yaw)
        if self.experiment == 'square':
            return (0.0, 0.0, 0.0)
        return (0.0, 0.0, 0.0)

    def _get_objective_text(self):
        if self.experiment == 'straight':
            return f'avanzar {self.distance:.3f} m en X (theta = 0.000 rad)'
        if self.experiment == 'rotate':
            return (f'rotar {self.rotation:.3f} rad '
                    f'({math.degrees(self.rotation):.1f} deg) sobre si mismo')
        if self.experiment == 'square':
            return f'cerrar cuadrado de {self.distance:.1f} m por lado'
        return f'experimento desconocido'

    def _print_report(self):
        log = self.get_logger()
        if self.last_gt is None or self.last_odom is None:
            log.warn('No se recibieron mensajes de /sim_pose_odom y/o /odom. '
                     'No se puede generar reporte de consistencia.')
            return

        # --- Extraer poses ---
        gp = self.last_gt.pose.pose
        gt_x, gt_y = gp.position.x, gp.position.y
        gt_yaw = self._quat_to_yaw(gp.orientation)

        ep = self.last_odom.pose.pose
        est_x, est_y = ep.position.x, ep.position.y
        est_yaw = self._quat_to_yaw(ep.orientation)

        cov = self.last_odom.pose.covariance

        # --- Target ---
        tx, ty, tyaw = self._get_target()
        move_dist = self.distance if self.experiment != 'rotate' else self.rotation

        # --- Errores GT vs target ---
        err_gt_tgt_x = gt_x - tx
        err_gt_tgt_y = gt_y - ty
        err_gt_tgt_xy = math.sqrt(err_gt_tgt_x**2 + err_gt_tgt_y**2)
        err_gt_tgt_yaw = self._normalize_angle(gt_yaw - tyaw)

        # --- Errores localisation vs GT ---
        err_loc_x = est_x - gt_x
        err_loc_y = est_y - gt_y
        err_loc_xy = math.sqrt(err_loc_x**2 + err_loc_y**2)
        err_loc_yaw = self._normalize_angle(est_yaw - gt_yaw)

        # --- Sigmas ---
        sigma_x = math.sqrt(max(cov[0], 0.0))
        sigma_y = math.sqrt(max(cov[7], 0.0))
        sigma_yaw = math.sqrt(max(cov[35], 0.0))
        sigma_xy = math.sqrt(sigma_x**2 + sigma_y**2)

        # --- Consistencia 3-sigma ---
        pos_ok = err_loc_xy < 3.0 * sigma_xy if sigma_xy > 0 else False
        yaw_ok = abs(err_loc_yaw) < 3.0 * sigma_yaw if sigma_yaw > 0 else False

        pos_sym = 'CONSISTENTE' if pos_ok else 'INCONSISTENTE'
        yaw_sym = 'CONSISTENTE' if yaw_ok else 'INCONSISTENTE'

        pct_gt = (err_gt_tgt_xy / move_dist * 100.0) if move_dist > 0 else 0.0
        pct_loc = (err_loc_xy / move_dist * 100.0) if move_dist > 0 else 0.0

        # --- Kr/kl para conclusion ---
        kr_kl_str = ''
        if self.report_kr > 0.0 or self.report_kl > 0.0:
            kr_kl_str = f'kr={self.report_kr:.2f}, kl={self.report_kl:.2f} -> '

        conclusion = 'CONSISTENTE' if (pos_ok and yaw_ok) else 'INCONSISTENTE'

        sep = '=' * 60
        report = f"""
{sep}
RESULTADO DEL EXPERIMENTO '{self.experiment}'
{sep}

Objetivo:        {self._get_objective_text()}

Pose REAL (ground truth):
  x     = {gt_x:.3f} m   (error vs objetivo:  {err_gt_tgt_x:+.3f} m  /  {pct_gt:.1f}%)
  y     = {gt_y:.3f} m
  theta = {gt_yaw:.3f} rad ({math.degrees(gt_yaw):.1f} deg)

Pose ESTIMADA (localisation):
  x     = {est_x:.3f} m
  y     = {est_y:.3f} m
  theta = {est_yaw:.3f} rad ({math.degrees(est_yaw):.1f} deg)

Error de localisation (estimada vs real):
  distancia: {err_loc_xy:.3f} m  ({pct_loc:.1f}% del recorrido objetivo)
  yaw:       {math.degrees(abs(err_loc_yaw)):.1f} deg

Incertidumbre estimada (1-sigma):
  sigma_x   = {sigma_x:.3f} m
  sigma_y   = {sigma_y:.3f} m
  sigma_yaw = {sigma_yaw:.3f} rad ({math.degrees(sigma_yaw):.1f} deg)

Verificacion de consistencia (error_real < 3-sigma):
  Posicion: error {err_loc_xy:.3f} m {'<' if pos_ok else '>='} 3-sigma {3.0*sigma_xy:.3f} m  {pos_sym}
  Yaw:      error {abs(err_loc_yaw):.3f} rad {'<' if yaw_ok else '>='} 3-sigma {3.0*sigma_yaw:.3f} rad  {yaw_sym}

Conclusion: {kr_kl_str}filtro {conclusion}
{sep}"""
        log.info(report)

        # --- Línea machine-readable para sweep automático ---
        c_xy = '1' if pos_ok else '0'
        c_yaw = '1' if yaw_ok else '0'
        csv = (f"CSV_REPORT,{self.experiment},"
               f"{self.report_kr:.6f},{self.report_kl:.6f},"
               f"{gt_x:.6f},{gt_y:.6f},{gt_yaw:.6f},"
               f"{est_x:.6f},{est_y:.6f},{est_yaw:.6f},"
               f"{err_loc_xy:.6f},{err_loc_yaw:.6f},"
               f"{sigma_x:.6f},{sigma_y:.6f},{sigma_yaw:.6f},"
               f"{c_xy},{c_yaw}")
        print(csv, flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = ExperimentRunner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
