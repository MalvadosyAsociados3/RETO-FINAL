import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState                                              # Tipo de mensaje para las articulaciones.
from rclpy.qos import QoSProfile, ReliabilityPolicy

class JointStateNode(Node):

    def __init__(self):
        super().__init__('joint_state_node')

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Suscriptores a los tópicos de velocidad de la rueda derecha e izquierda.
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

        # Publicador de joint states.
        self.pub_joint = self.create_publisher(JointState, '/joint_states', 10)

        self.vel_der = 0.0                                                          # Velocidad angular rueda derecha.
        self.vel_izq = 0.0                                                          # Velocidad angular rueda izquierda.

        self.pos_der = 0.0                                                          # Posición angular acumulada rueda derecha.
        self.pos_izq = 0.0                                                          # Posición angular acumulada rueda izquierda.

        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(0.05, self.update)

    def der_callback(self, msg):
        self.vel_der = msg.data

    def izq_callback(self, msg):
        self.vel_izq = msg.data

    def update(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time
        if dt <= 0.0:
            return

        # Integración de velocidad / posición angular.
        self.pos_der += self.vel_der * dt
        self.pos_izq += self.vel_izq * dt

        msg = JointState()
        msg.header.stamp = current_time.to_msg()

        msg.name = ['wheel_r_joint', 'wheel_l_joint']
        msg.position = [self.pos_der, self.pos_izq]                                 # Posiciones angulares. 
        msg.velocity = [self.vel_der, self.vel_izq]

        self.pub_joint.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
