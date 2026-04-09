#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import cv2
import os
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Empty

class ImageCapture(Node):
    def __init__(self):
        super().__init__('image_capture')
                
        self.declare_default_parameters()
        self.declare_topics()
        
        self.target_count = 1
        self.bridge = CvBridge()
        self.latest_image_msg = None
        os.makedirs(self.save_dir, exist_ok=True)
        self.get_logger().info('ImageCapture Node Started!')

    def declare_default_parameters(self):
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('trigger_topic', '/aeac/internal/take_picture')
        self.declare_parameter('snapshot_topic', '/aeac/external/target_picture')
        self.declare_parameter('save_dir', '/water_ws/snapshots')
        
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.trigger_topic = self.get_parameter('trigger_topic').get_parameter_value().string_value
        self.snapshot_topic = self.get_parameter('snapshot_topic').get_parameter_value().string_value
        self.save_dir = self.get_parameter('save_dir').get_parameter_value().string_value
        
    def declare_topics(self):
        qos_re = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST, depth=10)

        self.image_sub = self.create_subscription(
            CompressedImage,
            self.image_topic,
            self.image_callback,
            qos_re
        )

        self.trigger_sub = self.create_subscription(
            Empty,
            self.trigger_topic,
            self.trigger_callback,
            10
        )

        self.snapshot_pub = self.create_publisher(
            CompressedImage,
            self.snapshot_topic,
            qos_re
        )
        
    def image_callback(self, msg: CompressedImage):
        self.latest_image_msg = msg

    def trigger_callback(self, msg: Empty):
        self.get_logger().info('Take picture command received')

        if self.latest_image_msg is None:
            self.get_logger().warn('Trigger received but no image has arrived yet!')
            return

        self.snapshot_pub.publish(self.latest_image_msg)   
        
        try:
            pass
            # cv_image = self.bridge.imgmsg_to_cv2(self.latest_image_msg, desired_encoding='bgr8')

            # filename = os.path.join(self.save_dir, f'Task_2_Zenith_target_{self.target_count}.png')
            # self.get_logger().info(f'File name: {filename}')

            # cv2.imwrite(filename, cv_image)
            # self.target_count+=1

        except Exception as e:
            self.get_logger().error(f'Failed to save snapshot: {e}')     

def main(args=None):
    rclpy.init(args=args)
    node = ImageCapture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()