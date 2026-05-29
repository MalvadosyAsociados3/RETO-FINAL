import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time as RclpyTime
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState


def yaw_to_quat(yaw: float):
    return (math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0))


class PuzzlebotPublisher(Node):
    """
    Nodo que publica:
      - TF estatico:  map -> odom
      - TF dinamico:  odom -> base_footprint  (movimiento circular)
      - JointState:   wheel_r_joint, wheel_l_joint  (ruedas girando)

    
    """

    def __init__(self):
        super().__init__('puzzlebot_publisher')

        # --- Broadcasters ---
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        # --- Publisher de estados de articulaciones ---
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        # --- TF estatico: ---
        self.publish_static_map_odom()

        # --- Estado del robot ---
        self.t = 0.0          # tiempo acumulado
        self.wheel_angle = 0.0  # angulo acumulado de las ruedas

        # Parametros del movimiento circular
        self.radius = 0.5   # metros
        self.omega = 0.5    # rad/s (velocidad angular)

        # Timer a 20 Hz
        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.timer_cb)

        self.get_logger().info('Puzzlebot Publisher iniciado.')

    
    # TF ESTATICO: map -> odom  
    
    def publish_static_map_odom(self):
        tf_msg = TransformStamped()
    
        tf_msg.header.stamp = RclpyTime(seconds=0).to_msg()
        tf_msg.header.frame_id = 'map'
        tf_msg.child_frame_id = 'odom'
        tf_msg.transform.translation.x = 0.0
        tf_msg.transform.translation.y = 0.0
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.x = 0.0
        tf_msg.transform.rotation.y = 0.0
        tf_msg.transform.rotation.z = 0.0
        tf_msg.transform.rotation.w = 1.0
        self.static_broadcaster.sendTransform(tf_msg)
        self.get_logger().info('TF estatico map->odom publicado.')

    
    # CALLBACK del timer (20 Hz)
    
    def timer_cb(self):
        self.t += self.dt

        # Posicion en el circulo
        x = self.radius * math.cos(self.omega * self.t)
        y = self.radius * math.sin(self.omega * self.t)

        
        yaw = self.omega * self.t + math.pi / 2.0

        # Velocidad lineal del robot = radio * omega (m/s)
        v = self.radius * self.omega
        wheel_radius = 0.05  # metros (radio de la rueda)
        self.wheel_angle += (v / wheel_radius) * self.dt

      
        self.publish_odom_to_base_footprint(x, y, yaw)

        # Publicar JointState para las ruedas
        self.publish_joint_states()

    
    # TF DINAMICO: odom -> base_footprint
    
    def publish_odom_to_base_footprint(self, x, y, yaw):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_footprint'

        tf_msg.transform.translation.x = x
        tf_msg.transform.translation.y = y
        tf_msg.transform.translation.z = 0.0

       
        # (w, x, y, z) para una rotacion pura alrededor de Z
        q = yaw_to_quat(yaw)
        tf_msg.transform.rotation.w = q[0]
        tf_msg.transform.rotation.x = q[1]
        tf_msg.transform.rotation.y = q[2]
        tf_msg.transform.rotation.z = q[3]

        self.tf_broadcaster.sendTransform(tf_msg)

    
    # JOINT STATES: ruedas girando
    
    def publish_joint_states(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ['wheel_r_joint', 'wheel_l_joint']
        js.position = [self.wheel_angle, self.wheel_angle]
        js.velocity = []
        js.effort = []
        self.joint_pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = PuzzlebotPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()
