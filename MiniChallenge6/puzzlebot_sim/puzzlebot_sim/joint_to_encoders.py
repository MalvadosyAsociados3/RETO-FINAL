"""
joint_to_encoders
=================

Republica las velocidades angulares de las ruedas izquierda y derecha publicadas
en /joint_states (sensor_msgs/JointState, por el JointStatePublisher de Gazebo
Fortress) como std_msgs/Float32 en /VelocityEncL y /VelocityEncR — el formato
que espera el nodo `localisation` (y el simulador real del Puzzlebot).

Existe porque el plugin custom de MCR2 (libDiffDynamicPlugin.so) se construyo
contra gz-sim7 (Garden) y no carga en ign-gazebo-6 (Fortress, el que usa Humble).
Usamos el DiffDrive built-in de Fortress para mover el robot y este shim para
emitir los encoders en el topico esperado.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32


class JointToEncoders(Node):

    def __init__(self):
        super().__init__('joint_to_encoders')

        self.declare_parameter('left_joint', 'wheel_left_joint')
        self.declare_parameter('right_joint', 'wheel_right_joint')
        self.left_joint = str(self.get_parameter('left_joint').value)
        self.right_joint = str(self.get_parameter('right_joint').value)

        self.left_pub = self.create_publisher(Float32, 'VelocityEncL', 10)
        self.right_pub = self.create_publisher(Float32, 'VelocityEncR', 10)

        self.create_subscription(
            JointState, 'joint_states', self.joint_states_cb,
            qos_profile_sensor_data,
        )

        self._warned_missing = False
        self.get_logger().info(
            f'joint_to_encoders: {self.left_joint} -> /VelocityEncL, '
            f'{self.right_joint} -> /VelocityEncR'
        )

    def joint_states_cb(self, msg: JointState):
        if not msg.velocity or not msg.name:
            return
        try:
            il = msg.name.index(self.left_joint)
            ir = msg.name.index(self.right_joint)
        except ValueError:
            if not self._warned_missing:
                self.get_logger().warn(
                    f'/joint_states no contiene {self.left_joint} o '
                    f'{self.right_joint}. Nombres recibidos: {list(msg.name)}'
                )
                self._warned_missing = True
            return

        if il >= len(msg.velocity) or ir >= len(msg.velocity):
            return

        self.left_pub.publish(Float32(data=float(msg.velocity[il])))
        self.right_pub.publish(Float32(data=float(msg.velocity[ir])))


def main(args=None):
    rclpy.init(args=args)
    node = JointToEncoders()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
