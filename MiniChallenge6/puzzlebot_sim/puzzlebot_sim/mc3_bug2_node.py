import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       DurabilityPolicy)

from geometry_msgs.msg import Twist, Pose2D, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty, ColorRGBA
from visualization_msgs.msg import Marker

import math


class Bug2Node(Node):

    def __init__(self):

        super().__init__('bug2_node')

        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.goal_sub = self.create_subscription(Pose2D, '/goal', self.goal_callback, 10)
        # Acepta tambien /current_goal del point_generator (PoseStamped)
        self.current_goal_sub = self.create_subscription(
            PoseStamped, '/current_goal', self.current_goal_callback, latched_qos)
        self.cmd_pub  = self.create_publisher(Twist, '/cmd_vel', 10)
        # Publica /goal_reached para que point_generator avance al siguiente WP
        self.goal_reached_pub = self.create_publisher(Empty, '/goal_reached', reliable_qos)
        # Marker para RViz
        self.goal_marker_pub = self.create_publisher(Marker, '/goal_marker', latched_qos)

        self.goal_received = False
        self.goal_reached  = False

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.raw_x     = 0.0
        self.raw_y     = 0.0
        self.raw_theta = 0.0

        self.odom_received       = False
        self.last_odom_time      = None
        self.prev_odom_time      = None
        self.last_odom_diag_time = None

        self.goal_x  = 0.0
        self.goal_y  = 0.0
        self.start_x = 0.23
        self.start_y = -0.28

        self.state = "GO_TO_GOAL"
        self.hit_distance = 0.0
        # Tracking del tiempo en wall_follow para evitar oscilaciones BUG0
        self._wf_enter_time = None
        # Tolerancia M-line subida (era 0.10): en maze angosto el robot
        # nunca pasaba EXACTO por la linea start-goal por el desvio
        # alrededor del divisor 705/706.
        self.mline_tolerance = 0.25

        self.lidar         = LaserScan()
        self.scan_received = False

        # Control Go To Goal
        self.kv = 0.25
        self.kw = 0.8

        # Control Follow Wall
        self.min_side_dist = 0.28        # antes 0.25 — la rueda sobresale ~8cm
        self.desired_wall_dist = 0.36    # antes 0.32 — wheel queda a 28cm
        self.k_wall = 1.0

        # "auto" = decide segun donde esta el goal (recomendado).
        # "left" o "right" = fija el lado siempre.
        self.wall_follow_side = "auto"
        self.active_wall_side = None

        # Velocidades límite
        self.max_v = 0.08
        self.max_w = 0.45

        self.declare_parameter('goal_tolerance', 0.15)

        # Umbrales de detección — SUBIDOS para detectar paredes delgadas
        # antes y dar mas margen al cuerpo del robot.
        self.d_wall          = 0.42      # antes 0.35
        self.front_stop_dist = 0.36      # antes 0.30
        self.goal_tolerance  = self.get_parameter('goal_tolerance').value

        # Bandera para evitar búsqueda de pared mientras se gira
        self._turning_away = False

        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info("BUG2 NODE STARTED")

    # Callbacks 

    def odom_callback(self, msg):

        now = self.get_clock().now()
        self.odom_received  = True
        self.prev_odom_time = self.last_odom_time
        self.last_odom_time = now

        self.raw_x = msg.pose.pose.position.x
        self.raw_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        sin_yaw = 2.0 * (q.w * q.z + q.x * q.y)
        cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.raw_theta = math.atan2(sin_yaw, cos_yaw)

        self.x = self.raw_x
        self.y = self.raw_y
        self.theta = self.raw_theta

        if self.last_odom_diag_time is None:
            self.last_odom_diag_time = now

        if (now - self.last_odom_diag_time).nanoseconds * 1e-9 >= 1.0:
            #self.get_logger().info(
             #   f"ODOM ({self.x:.2f}, {self.y:.2f}, {math.degrees(self.theta):.1f}°)"
            #)
            self.last_odom_diag_time = now

    def scan_callback(self, msg):
        self.lidar         = msg
        self.scan_received = True

    def goal_callback(self, msg):
        self._set_goal(msg.x, msg.y, source='/goal')

    def current_goal_callback(self, msg):
        # Anti-stale del latched (descartar si > 5s viejo)
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        now = self.get_clock().now().nanoseconds * 1e-9
        if ts > 0 and (now - ts) > 5.0:
            self.get_logger().warn(
                f'/current_goal descartado por stale ({now-ts:.1f}s viejo)')
            return
        # Solo aceptar si cambia
        if (self.goal_received and abs(msg.pose.position.x - self.goal_x) < 0.01
                and abs(msg.pose.position.y - self.goal_y) < 0.01):
            return
        self._set_goal(msg.pose.position.x, msg.pose.position.y,
                       source='/current_goal')

    def _set_goal(self, gx, gy, source='/goal'):
        self.goal_x = float(gx)
        self.goal_y = float(gy)
        self.start_x = self.x
        self.start_y = self.y
        self.goal_received = True
        self.goal_reached = False
        self.state = "GO_TO_GOAL"
        self._turning_away = False
        self.active_wall_side = None
        self._publish_goal_marker()
        self.get_logger().info(
            f"NEW GOAL ({source}): ({self.goal_x:.2f}, {self.goal_y:.2f}) "
            f"from=({self.start_x:.2f}, {self.start_y:.2f}) "
            f"tol={self.goal_tolerance:.2f}"
        )

    def _publish_goal_marker(self):
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'bug2_goal'
        m.id = 0
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose.position.x = float(self.goal_x)
        m.pose.position.y = float(self.goal_y)
        m.pose.position.z = 0.05
        m.pose.orientation.w = 1.0
        m.scale.x = 0.15
        m.scale.y = 0.15
        m.scale.z = 0.10
        m.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)
        self.goal_marker_pub.publish(m)

    #  Ángulos

    def normalize_angle_pi(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def normalize_angle_0_2pi(self, angle):
        return angle % (2.0 * math.pi)

    #  LiDAR helpers 

    def valid_range(self, r):
        return math.isfinite(r) and r > 0.0

    def get_min_range_between(self, angle_min, angle_max):
        if not self.scan_received or len(self.lidar.ranges) == 0:
            return 999.0
        angle_min = self.normalize_angle_0_2pi(angle_min)
        angle_max = self.normalize_angle_0_2pi(angle_max)
        if self.lidar.angle_increment <= 0.0:
            return 999.0

        valid = []
        for i, r in enumerate(self.lidar.ranges):
            if not self.valid_range(r):
                continue

            angle = self.normalize_angle_0_2pi(
                self.lidar.angle_min + i * self.lidar.angle_increment
            )

            if angle_min <= angle_max:
                in_sector = angle_min <= angle <= angle_max
            else:
                in_sector = angle >= angle_min or angle <= angle_max

            if in_sector:
                valid.append(r)

        return min(valid) if valid else 999.0

    def get_front_distance(self):
        return self.get_min_range_between(19.0 * math.pi / 10.0, math.pi / 10.0)

    def get_left_front_distance(self):
        return self.get_min_range_between(math.pi / 10.0, 3.0 * math.pi / 10.0)

    def get_left_distance(self):
        return self.get_min_range_between(3.0 * math.pi / 10.0, 7.0 * math.pi / 10.0)

    def get_right_front_distance(self):
        return self.get_min_range_between(17.0 * math.pi / 10.0, 19.0 * math.pi / 10.0)

    def get_right_distance(self):
        return self.get_min_range_between(13.0 * math.pi / 10.0, 17.0 * math.pi / 10.0)

    #  Bug2 helpers 

    def is_path_blocked(self):
        return self.get_front_distance() < self.d_wall

    def distance_to_mline(self):
        A = self.goal_y - self.start_y
        B = self.start_x - self.goal_x
        C = self.goal_x * self.start_y - self.start_x * self.goal_y
        denom = math.sqrt(A * A + B * B)
        if denom < 1e-6:
            return 999.0
        return abs(A * self.x + B * self.y + C) / denom

    def odom_is_fresh(self):
        if not self.odom_received or self.last_odom_time is None:
            self.get_logger().warn("Sin odometría.", throttle_duration_sec=1.0)
            return False
        age = (self.get_clock().now() - self.last_odom_time).nanoseconds * 1e-9
        if age > 0.25:
            self.cmd_pub.publish(Twist())
            self.get_logger().warn(f"Odometría vieja ({age:.2f}s).", throttle_duration_sec=1.0)
            return False
        return True

    def limit_cmd(self, v, w):
        v = max(min(v, self.max_v), -self.max_v)
        w = max(min(w, self.max_w), -self.max_w)
        return v, w

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def choose_wall_side(self):
        if self.wall_follow_side in ["left", "right"]:
            return self.wall_follow_side

        # AUTO basado en la POSICION RELATIVA del goal en el frame del robot:
        # si el goal esta a la derecha (Y relativa negativa), seguir pared
        # DERECHA (asi la rodea por su lado derecho).
        # Si el goal esta a la izquierda, seguir pared izquierda.
        dx_world = self.goal_x - self.x
        dy_world = self.goal_y - self.y
        # Transformar al frame del robot (rotar -theta)
        cos_t = math.cos(-self.theta)
        sin_t = math.sin(-self.theta)
        dy_robot = sin_t * dx_world + cos_t * dy_world
        side = "right" if dy_robot < 0.0 else "left"
        self.get_logger().info(
            f"choose_wall_side: dy_robot={dy_robot:+.2f} -> {side}"
        )
        return side

    #  Control loop 

    def control_loop(self):

        if not self.goal_received:
            if self.goal_reached:
                self.stop_robot()
            return
        if not self.odom_is_fresh():
            return
        if not self.scan_received or len(self.lidar.ranges) == 0:
            self.stop_robot()
            self.get_logger().warn("Sin scan LiDAR.", throttle_duration_sec=1.0)
            return

        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        distance_to_goal = math.sqrt(dx**2 + dy**2)
        theta_g = math.atan2(dy, dx)
        error_theta = self.normalize_angle_pi(theta_g - self.theta)

        front = self.get_front_distance()
        left = self.get_left_distance()
        left_front = self.get_left_front_distance()
        right = self.get_right_distance()
        right_front = self.get_right_front_distance()

        self.get_logger().info(
            f"STATE={self.state} | front={front:.2f} L={left:.2f} "
            f"R={right:.2f} lf={left_front:.2f} rf={right_front:.2f} | "
            f"pose=({self.x:.2f},{self.y:.2f},{math.degrees(self.theta):+.0f}°) "
            f"dist={distance_to_goal:.2f} et={math.degrees(error_theta):+.0f}°",
            throttle_duration_sec=1.0
        )

        #  Goal alcanzado
        if distance_to_goal <= self.goal_tolerance:
            self.stop_robot()
            self.goal_reached  = True
            self.goal_received = False
            # Notifica al point_generator para que pase al siguiente WP
            self.goal_reached_pub.publish(Empty())
            self.get_logger().info(
                f"GOAL REACHED | dist={distance_to_goal:.2f} "
                f"pose=({self.x:.2f}, {self.y:.2f})"
            )
            return

        #  GO TO GOAL 
        if self.state == "GO_TO_GOAL":

            if right < self.min_side_dist or right_front < self.min_side_dist:
                self.state = "FOLLOW_WALL"
                self.hit_distance = distance_to_goal
                self._turning_away = False
                self.active_wall_side = self.choose_wall_side()
                self._wf_enter_time = self.get_clock().now()  # marca entrada
                self.stop_robot()
                self.get_logger().info(
                    f"→ FOLLOW_WALL (pared derecha) | side={self.active_wall_side} "
                    f"right={right:.2f} rf={right_front:.2f}"
                )
                return

            if left < self.min_side_dist or left_front < self.min_side_dist:
                self.state = "FOLLOW_WALL"
                self.hit_distance = distance_to_goal
                self._turning_away = False
                self.active_wall_side = self.choose_wall_side()
                self._wf_enter_time = self.get_clock().now()  # marca entrada
                self.stop_robot()
                self.get_logger().info(
                    f"→ FOLLOW_WALL (pared izquierda) | side={self.active_wall_side} "
                    f"left={left:.2f} lf={left_front:.2f}"
                )
                return

            if self.is_path_blocked():
                self.state = "FOLLOW_WALL"
                self.hit_distance = distance_to_goal
                self._turning_away = False
                self.active_wall_side = self.choose_wall_side()
                self._wf_enter_time = self.get_clock().now()  # marca entrada
                self.stop_robot()
                return

            if abs(error_theta) > 0.25:
                v = 0.0
                w = self.kw * error_theta
            else:
                v = self.kv * distance_to_goal
                w = self.kw * error_theta

            v, w = self.limit_cmd(v, w)

        # FOLLOW WALL 
        elif self.state == "FOLLOW_WALL":

            distance_mline = self.distance_to_mline()

            # Condición de salida — BUG2 clasica
            bug2_leave = (
                distance_mline < self.mline_tolerance
                and distance_to_goal < self.hit_distance - 0.15
                and not self.is_path_blocked()
            )

            # Anti-oscilacion: lleva cuenta de cuanto tiempo en wall_follow.
            if self._wf_enter_time is None:
                self._wf_enter_time = self.get_clock().now()
            wf_elapsed = (self.get_clock().now() -
                          self._wf_enter_time).nanoseconds * 1e-9

            # DETECTOR DE LADO EQUIVOCADO — AGRESIVO.
            # Dispara si > 5 seg en wall_follow Y dist subio > 0.20 desde hit.
            # Tambien dispara si la distancia subio > 0.60 sin importar el tiempo
            # (estamos vagando muy lejos del goal).
            wandering_far = distance_to_goal > self.hit_distance + 0.60
            wandering_long = (wf_elapsed > 5.0 and
                              distance_to_goal > self.hit_distance + 0.20)
            if wandering_far or wandering_long:
                new_side = ('right' if self.active_wall_side == 'left'
                            else 'left')
                self.get_logger().warn(
                    f"WRONG WAY ({wf_elapsed:.1f}s): dist {self.hit_distance:.2f} "
                    f"-> {distance_to_goal:.2f}. Cambio side "
                    f"{self.active_wall_side} -> {new_side}"
                )
                self.active_wall_side = new_side
                self.hit_distance = distance_to_goal
                self._wf_enter_time = self.get_clock().now()
                self._turning_away = False
                # Backup forzado: retrocede un poco para escapar del corner
                # actual antes de iniciar el otro side
                self.stop_robot()
                back = Twist()
                back.linear.x = -0.05
                self.cmd_pub.publish(back)
                return

            # BUG0 fallback (anti-oscilacion). Dispara si:
            #   1. >= 1.5s en wall_follow
            #   2. Goal cerca (< 1.5m)
            #   3. Hicimos progreso (dist < hit_dist - 10cm) - clave!
            #   4. Goal en cono frontal +/- 60 deg
            #   5. Clearance en direccion del goal > 0.45m
            bug0_leave = False
            if (wf_elapsed >= 1.5
                    and distance_to_goal < 1.5
                    and distance_to_goal < self.hit_distance - 0.10
                    and not self.is_path_blocked()
                    and abs(error_theta) < math.pi / 3):
                goal_clearance = front
                if error_theta > math.pi / 6:
                    goal_clearance = min(front, left_front)
                elif error_theta < -math.pi / 6:
                    goal_clearance = min(front, right_front)
                if goal_clearance > 0.45:
                    bug0_leave = True

            if bug2_leave or bug0_leave:
                self.state         = "GO_TO_GOAL"
                self._turning_away = False
                self._wf_enter_time = None   # reset al salir
                self.stop_robot()
                reason = "BUG2-mline" if bug2_leave else "BUG0-clearshot"
                self.get_logger().info(
                    f"→ GO_TO_GOAL ({reason}) | mline={distance_mline:.3f} "
                    f"hit_d={self.hit_distance:.2f} dist={distance_to_goal:.2f}"
                )
                return

            # Pared enfrente: girar a la derecha o izquierda
            side = self.active_wall_side or "left"

            if side == "left":
                wall_dist = left
                wall_front = left_front
                turn_away_w = -0.45
                corner_w = -0.40
                search_w = 0.25

                if front < self.front_stop_dist:
                    v = 0.02
                    w = turn_away_w
                    self._turning_away = True

                else:
                    if self._turning_away and front >= self.d_wall:
                        self._turning_away = False

                    #if wall_dist < self.min_side_dist or wall_front < self.min_side_dist:
                     #   v = 0.02
                      #  w = -0.45

                    if wall_front < 0.28:
                        v = 0.03
                        w = corner_w

                    elif not self._turning_away and wall_dist > 0.30 and wall_front > 0.30:
                        v = 0.05
                        w = search_w

                    else:
                        error_wall = self.desired_wall_dist - wall_dist
                        v = 0.07
                        w = -self.k_wall * error_wall

            else:  # side == "right"
                wall_dist = right
                wall_front = right_front
                turn_away_w = 0.45
                corner_w = 0.40
                search_w = -0.25

                if front < self.front_stop_dist:
                    v = 0.02
                    w = turn_away_w
                    self._turning_away = True

                else:
                    if self._turning_away and front >= self.d_wall:
                        self._turning_away = False

                    #if wall_dist < self.min_side_dist or wall_front < self.min_side_dist:
                     #   v = 0.02
                      #  w = 0.45

                    if wall_front < 0.28:
                        v = 0.03
                        w = corner_w

                    elif not self._turning_away and wall_dist > 0.30 and wall_front > 0.30:
                        v = 0.05
                        w = search_w

                    else:
                        error_wall = self.desired_wall_dist - wall_dist
                        v = 0.07
                        w = self.k_wall * error_wall

            v, w = self.limit_cmd(v, w)

        else:
            self.get_logger().warn(f"Estado desconocido: {self.state}")
            self.stop_robot()
            return

        msg = Twist()
        msg.linear.x  = v
        msg.angular.z = w
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Bug2Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
