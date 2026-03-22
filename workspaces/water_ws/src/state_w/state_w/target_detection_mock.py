#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from custom_interfaces.srv import LocateTarget
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped


class TargetDetectionMock(Node):
    def __init__(self):
        super().__init__('target_detection_mock')
        self.target_pose_srv = self.create_service(
            LocateTarget,
            '/aeac/auto_approach/locate_target',
            self.locate_target_callback
        )
        
        self.circle_pos = [5.4, 6.01, 1.0]
        
    def locate_target_callback(self, request, response):
        position = PoseStamped()        

        if not request.activate:
            self.get_logger().info("Locate target called but activate=False → ignoring")
            return position
        
        position.header.stamp = self.get_clock().now().to_msg()
        position.header.frame_id = 'map'
        
        position.pose.position.x = self.circle_pos[0]
        position.pose.position.y = self.circle_pos[1]
        position.pose.position.z = self.circle_pos[2]

        position.pose.orientation.w = 0.0
        response.target = position
        return response
    
    
def main(args=None):
    rclpy.init(args=args)
    node = TargetDetectionMock()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down.\n")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()