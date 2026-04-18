#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty
from zed_msgs.msg import ObjectsStamped, Object
import time


class TargetDetectionMock(Node):
    @staticmethod
    def _create_qos_profile(reliability_policy):
        return QoSProfile(
            reliability=reliability_policy,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
    
    def __init__(self):
        super().__init__('target_detection_mock')
        qos_reliable = self._create_qos_profile(QoSReliabilityPolicy.RELIABLE)

        self.target_pose_pub = self.create_publisher(
            ObjectsStamped,
            '/aeac/internal/auto_approach/target_detected',
            qos_reliable
        )
        
        self.trigger_target_detection_sub = self.create_subscription(
            Empty,
            '/aeac/internal/auto_approach/detect_target',
            self.locate_target_callback,
            qos_reliable
        )
        
        self.circle_pos = [5.4, 6.01, 1.0]
                
    def locate_target_callback(self, _: Empty):
        obects_stamped = ObjectsStamped()            
        object = Object()
        
        time.sleep(2)
        
        object.label = "Red"
        object.confidence = float(0.8)
        object.position = [float(self.circle_pos[0]), float(self.circle_pos[1]), float(self.circle_pos[2])]
        
        obects_stamped.objects.append(object)

        self.target_pose_pub.publish(obects_stamped)
    
    
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