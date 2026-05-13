#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from ament_index_python.packages import get_package_share_directory
import os

class SendPhotoMock(Node):
    def __init__(self):
        super().__init__("send_photo_mock")

        # ── Parameters ─────────────────────────────────────────────────────
        self.declare_parameter("image_filename", "chat.jpg")
        self.declare_parameter("image_format", "jpeg")

        image_filename = self.get_parameter("image_filename").get_parameter_value().string_value
        self._image_format = self.get_parameter("image_format").get_parameter_value().string_value

        pkg_share = get_package_share_directory("upload_controller")
        self._image_path = os.path.join(pkg_share, "images", image_filename)

        self._image_data = self._load_image(self._image_path)

        self._photo_pub = self.create_publisher(CompressedImage, "/aeac/external/target_picture", 10)

        self._timer = self.create_timer(5.0, self.timer_callback)

    def _load_image(self, path: str) -> bytes:
        abs_path = os.path.abspath(path)
        if not os.path.isfile(abs_path):
            self.get_logger().error(f"Image file not found: {abs_path}")
            raise FileNotFoundError(f"Image file not found: {abs_path}")

        with open(abs_path, "rb") as f:
            data = f.read()

        self.get_logger().info(f"Loaded image '{abs_path}' ({len(data)} bytes)")
        return data

    def timer_callback(self):
        if self._image_data is None:
            self.get_logger().warn("No image data available, skipping publish.")
            return

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"
        msg.format = self._image_format
        msg.data = list(self._image_data)
        self._photo_pub.publish(msg)
        self.get_logger().info(
            f"Published '{self._image_path}' ({len(self._image_data)} bytes) as '{self._image_format}'."
        )

def main(args=None):
    rclpy.init(args=args)
    node = SendPhotoMock()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()