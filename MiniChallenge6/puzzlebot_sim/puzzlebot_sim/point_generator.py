"""
Nodo que expone los setpoints (waypoints) como parametros y los alimenta al
nodo de control uno por uno, via un handshake por tópicos.

Flujo:
  point_generator  --/current_goal-->    control
  point_generator  <--/goal_reached--    control   (cuando control termina)

Al recibir /goal_reached, avanza al siguiente waypoint (o loopea / termina).

Subscribe:
  /goal_reached (std_msgs/Empty) - confirmacion del control

Publica:
  /current_goal (geometry_msgs/PoseStamped, TRANSIENT_LOCAL) - setpoint actual
  /planned_path (nav_msgs/Path, TRANSIENT_LOCAL)             - trayectoria completa
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Empty
from rcl_interfaces.msg import SetParametersResult


class PointGenerator(Node):

    def __init__(self):
        super().__init__('point_generator')

        # --- Parametros ---
        self.declare_parameter('waypoints_x', [1.0, 1.0, 0.0, 0.0])
        self.declare_parameter('waypoints_y', [0.0, 1.0, 1.0, 0.0])
        self.declare_parameter('loop_trajectory', True)
        self.declare_parameter('startup_delay', 1.5)
        self.declare_parameter('frame_id', 'odom')
        # Modo "evaluacion": el robot se detiene en cada waypoint y espera
        # una senal manual del profesor para avanzar al siguiente. Cumple el
        # requisito del PDF "Detenerse completamente al llegar al objetivo.
        # Esperar la indicacion de los profesores para continuar al siguiente
        # waypoint."
        # Para avanzar manualmente, publicar en el topico `next_waypoint`:
        #     ros2 topic pub --once /next_waypoint std_msgs/Empty '{}'
        self.declare_parameter('manual_advance', False)
        # Modo "interactivo": NO publica ningun waypoint al arrancar (ignora
        # la lista del yaml). Solo se mueve cuando el usuario manda un
        # /goal_pose desde RViz. Ideal para evaluacion con click-y-anda.
        self.declare_parameter('interactive_mode', False)

        self._wx_raw = list(self.get_parameter('waypoints_x').value)
        self._wy_raw = list(self.get_parameter('waypoints_y').value)
        self.loop = bool(self.get_parameter('loop_trajectory').value)
        self.startup_delay = float(self.get_parameter('startup_delay').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.manual_advance = bool(self.get_parameter('manual_advance').value)
        self.interactive_mode = bool(self.get_parameter('interactive_mode').value)

        if len(self._wx_raw) != len(self._wy_raw) or len(self._wx_raw) == 0:
            self.get_logger().error('waypoints_x y waypoints_y deben tener mismo tamano (>0).')
            self.waypoints = []
        else:
            self.waypoints = [(float(a), float(b)) for a, b in zip(self._wx_raw, self._wy_raw)]

        # --- Estado ---
        self.wp_idx = 0
        self.finished = False

        # --- QoS ---
        latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        vol_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # --- Pub / Sub ---
        self.goal_pub = self.create_publisher(PoseStamped, 'current_goal', latched)
        self.path_pub = self.create_publisher(Path, 'planned_path', latched)
        self.reached_sub = self.create_subscription(
            Empty, 'goal_reached', self.reached_cb, vol_qos
        )
        # Senal manual del profesor para avanzar al siguiente waypoint.
        # Solo se usa cuando manual_advance=True.
        self.next_sub = self.create_subscription(
            Empty, 'next_waypoint', self.next_cb, vol_qos
        )
        self._waiting_for_next = False

        # /goal_pose viene del boton "2D Goal Pose" de RViz: permite mandar
        # un nuevo waypoint en vivo con un click en el mapa. Sobrescribe la
        # lista de waypoints del yaml.
        self.goalpose_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goalpose_cb, vol_qos
        )

        # En modo interactivo NO publicamos planned_path desde el yaml ni el
        # primer goal -- el usuario controla todo desde RViz (2D Goal Pose).
        if not self.interactive_mode:
            self.publish_planned_path()
            # Retrasa la publicacion del primer goal para dar tiempo al control a suscribirse
            self.startup_timer = self.create_timer(self.startup_delay, self._send_first_goal)
        else:
            self.get_logger().info(
                'point_generator en modo INTERACTIVO. Esperando goals de RViz '
                '(boton "2D Goal Pose"). La lista de waypoints del yaml se ignora.'
            )

        # Actualizacion en caliente de parametros
        self.add_on_set_parameters_callback(self._on_set_params)

        self.get_logger().info(
            f'PointGenerator iniciado con {len(self.waypoints)} waypoints, loop={self.loop}.'
        )

    # ---------- Primer goal ----------
    def _send_first_goal(self):
        # Timer one-shot
        self.startup_timer.cancel()
        self.publish_current_goal()

    # ---------- Publicacion de goal / path ----------
    def publish_current_goal(self):
        if not self.waypoints or self.finished:
            return
        gx, gy = self.waypoints[self.wp_idx]
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = self.frame_id
        ps.pose.position.x = float(gx)
        ps.pose.position.y = float(gy)
        ps.pose.orientation.w = 1.0
        self.goal_pub.publish(ps)
        self.get_logger().info(f'Publicando WP{self.wp_idx}: ({gx:.2f}, {gy:.2f})')

    def publish_planned_path(self):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.frame_id
        for wx, wy in self.waypoints:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(wx)
            ps.pose.position.y = float(wy)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        if self.loop and self.waypoints:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(self.waypoints[0][0])
            ps.pose.position.y = float(self.waypoints[0][1])
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.path_pub.publish(path)

    # ---------- Callback de confirmacion ----------
    def reached_cb(self, _msg: Empty):
        if self.finished or not self.waypoints:
            return
        self.get_logger().info(f'WP{self.wp_idx} confirmado por el control.')

        if self.manual_advance:
            # Modo evaluacion: detenerse y esperar senal del profesor.
            self._waiting_for_next = True
            self.get_logger().info(
                f'WP{self.wp_idx} alcanzado. Esperando senal manual del profesor.\n'
                f'    Para avanzar al siguiente waypoint, ejecutar:\n'
                f'        ros2 topic pub --once /next_waypoint std_msgs/msg/Empty "{{}}"'
            )
            return

        self._advance_waypoint()

    def goalpose_cb(self, msg: PoseStamped):
        """Recibe un goal del boton '2D Goal Pose' de RViz y lo publica
        como current_goal. Permite que el profesor de waypoints en vivo
        con clicks en el mapa, sin tocar el yaml."""
        gx = float(msg.pose.position.x)
        gy = float(msg.pose.position.y)
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = self.frame_id
        ps.pose.position.x = gx
        ps.pose.position.y = gy
        ps.pose.orientation.w = 1.0
        self.goal_pub.publish(ps)
        self.finished = False
        self._waiting_for_next = False
        self.get_logger().info(
            f'Nuevo goal recibido de RViz (2D Goal Pose): ({gx:.2f}, {gy:.2f})'
        )

    def next_cb(self, _msg: Empty):
        """Senal manual del profesor para avanzar al siguiente waypoint."""
        if not self.manual_advance:
            self.get_logger().warn(
                'Recibida /next_waypoint pero manual_advance=False; ignorando.'
            )
            return
        if not self._waiting_for_next:
            self.get_logger().warn(
                'Recibida /next_waypoint pero aun no llego al waypoint; ignorando.'
            )
            return
        self._waiting_for_next = False
        self.get_logger().info('Senal manual recibida; avanzando al siguiente waypoint.')
        self._advance_waypoint()

    def _advance_waypoint(self):
        """Avanza wp_idx y publica el siguiente goal."""
        self.wp_idx += 1
        if self.wp_idx >= len(self.waypoints):
            if self.loop:
                self.wp_idx = 0
                self.get_logger().info('Reiniciando trayectoria (loop).')
            else:
                self.finished = True
                self.get_logger().info('Trayectoria completada.')
                return
        self.publish_current_goal()

    # ---------- Update en caliente ----------
    def _on_set_params(self, params):
        new_wx = list(self._wx_raw)
        new_wy = list(self._wy_raw)
        new_loop = self.loop
        for p in params:
            if p.name == 'waypoints_x':
                new_wx = list(p.value)
            elif p.name == 'waypoints_y':
                new_wy = list(p.value)
            elif p.name == 'loop_trajectory':
                new_loop = bool(p.value)

        self._wx_raw = new_wx
        self._wy_raw = new_wy
        self.loop = new_loop

        if len(new_wx) == len(new_wy) and len(new_wx) > 0:
            self.waypoints = [(float(a), float(b)) for a, b in zip(new_wx, new_wy)]
            self.wp_idx = 0
            self.finished = False
            self.publish_planned_path()
            self.publish_current_goal()
            self.get_logger().info(
                f'Trayectoria actualizada ({len(self.waypoints)} waypoints, loop={self.loop}).'
            )
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    node = PointGenerator()
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
