import math
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from aruco_msgs.msg import MarkerArray


class ArucoNode(Node):

    def __init__(self):
        super().__init__('aruco_node')

        # Mapa global ArUcos
        # Formato:
        # ID: (x_global, y_global, theta_global)
        #
        # x_global, y_global vienen de tu tabla.
        # theta_global es la orientación aproximada del ArUco.

        self.aruco_map = {
            70:  (1.84, -0.30, math.pi),  # A
            705: (0.90, -1.20,  math.pi / 2),      # B
            706: (2.39, -1.26, math.pi),  # C
            708: (1.19, -1.21,  -math.pi / 2),          # D
            703: (1.23, -2.07,  0.0),  # E
            702: (0.28, -1.82,  -math.pi / 2),          # F
            75:  (2.74, -2.40,  math.pi / 2),      # G
            701: (2.84,  0.00,  -math.pi / 2),          # H
        }

        # Suscriptor a las detecciones ArUco en físico
        self.aruco_sub = self.create_subscription(MarkerArray,'/marker_publisher/markers',self.aruco_callback,10)

        # Publicador para el nodo de localización / EKF
        #
        # Mensaje:
        # data = [
        #   marker_id,
        #   marker_x_global,
        #   marker_y_global,
        #   marker_theta_global,
        #   range,
        #   bearing
        # ]
        self.measurement_pub = self.create_publisher(Float64MultiArray,'/aruco_measurement',10)
        

        self.get_logger().info('Aruco node started')

    def aruco_callback(self, msg):

        if len(msg.markers) == 0:
            return

        best_marker = None
        best_distance = float('inf')

        # Si detecta varios ArUcos, usamos el más cercano
        for marker in msg.markers:
            marker_id = int(marker.id)

            if marker_id not in self.aruco_map:
                # IDs no validos = falsa deteccion del aruco_ros. Filtrar
                # SILENCIOSAMENTE: cualquier marker que no este en nuestro
                # mapa es ruido (lighting, fragmentos, dictionary mismatch).
                continue

            # pose viene como marker.pose.pose.position
            # z = distancia hacia enfrente
            # x = desplazamiento lateral
            px = marker.pose.pose.position.x
            pz = marker.pose.pose.position.z

            distance = math.sqrt(px**2 + pz**2)

            if distance < best_distance:
                best_distance = distance
                best_marker = marker

        if best_marker is None:
            return

        marker_id = int(best_marker.id)

        # Posición del marcador respecto a la cámara
        cam_x = best_marker.pose.pose.position.x
        cam_z = best_marker.pose.pose.position.z

        # Medición para EKF
        measured_range = math.sqrt(cam_x**2 + cam_z**2)
        measured_bearing = -math.atan2(cam_x, cam_z)

        # Posición conocida del ArUco en el mapa
        marker_x, marker_y, marker_theta = self.aruco_map[marker_id]

        out = Float64MultiArray()
        out.data = [
            float(marker_id),
            marker_x,
            marker_y,
            marker_theta,
            measured_range,
            measured_bearing
        ]

        self.measurement_pub.publish(out)

        self.get_logger().info(
            f'ID: {marker_id} | '
            f'range: {measured_range:.3f} m | '
            f'bearing: {measured_bearing:.3f} rad | '
            f'global: ({marker_x:.2f}, {marker_y:.2f})'
        )


def main(args=None):
    rclpy.init(args=args)
    node = ArucoNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()