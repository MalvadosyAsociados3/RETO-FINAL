import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster    # Se usa para publicar las transformaciones dinámicas y estáticas.
from geometry_msgs.msg import TransformStamped                          # Mensaje de transformación.
import math
import numpy as np


def euler_to_quat(roll, pitch, yaw):
    """Convert Euler -> quaternion (w, x, y, z). Reemplaza transforms3d."""
    cy = math.cos(yaw * 0.5); sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5); sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5); sr = math.sin(roll * 0.5)
    return (cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy)

class FramePublisher(Node):

    def __init__(self):
        super().__init__('frame_publisher')

        # Estáticos.
        self.static_br1 = StaticTransformBroadcaster(self)


        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'                                       # Frame padre.
        t.child_frame_id = 'odom'                                       # Frame hijo.

        # Posición inicial.
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        # Sin rotación.
        q = euler_to_quat(0, 0, 0)
        t.transform.rotation.w = q[0]
        t.transform.rotation.x = q[1]
        t.transform.rotation.y = q[2]
        t.transform.rotation.z = q[3]

        self.static_br1.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)

    node = FramePublisher()

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