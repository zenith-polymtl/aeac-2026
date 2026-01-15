import rclpy
from rclpy.node import Node
import math

from custom_interfaces.srv import GpsToLocal


class GpsToLocalServer(Node):

    def __init__(self):
        super().__init__('gps_to_local')

        # Origine locale (doit correspondre à ton JSON)
        self.declare_parameter("lat_ref", 0.0)
        self.declare_parameter("lon_ref", 0.0)
        self.declare_parameter("alt_ref", 0.0)

        self.lat_ref = self.get_parameter("lat_ref").value
        self.lon_ref = self.get_parameter("lon_ref").value
        self.alt_ref = self.get_parameter("alt_ref").value

        self.srv = self.create_service(
            GpsToLocal,
            'gps_to_local',
            self.handle_request
        )

        self.get_logger().info(
            f"Service gps_to_local prêt "
            f"(ref lat={self.lat_ref}, lon={self.lon_ref}, alt={self.alt_ref})"
        )

    # --------------------------------------------------
    # Callback du service
    # --------------------------------------------------
    def handle_request(self, request, response):
        R = 6378137.0  # rayon Terre (m)

        d_lat = math.radians(request.latitude - self.lat_ref)
        d_lon = math.radians(request.longitude - self.lon_ref)

        north = d_lat * R
        east = d_lon * R * math.cos(math.radians(self.lat_ref))
        down = self.alt_ref - request.altitude

        response.north = north
        response.east = east
        response.down = down

        self.get_logger().info(
            f"GPS({request.latitude:.6f}, {request.longitude:.6f}, {request.altitude:.1f}) "
            f"→ NED({north:.2f}, {east:.2f}, {down:.2f})"
        )

        return response


def main(args=None):
    rclpy.init(args=args)
    node = GpsToLocalServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
