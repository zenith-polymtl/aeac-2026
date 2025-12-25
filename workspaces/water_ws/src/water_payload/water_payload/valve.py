#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from custom_interfaces.msg import TargetPosePolar
from std_msgs.msg import Bool, String

class AutonomousApproach(Node):
    def __init__(self):
        super().__init__('autonomous_approach')

        self.define_initial_state()
        self.set_up_parameters()
        self.set_up_topics()

    @staticmethod
    def _create_qos_profile(reliability_policy):
        return QoSProfile(
            reliability=reliability_policy,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

    def define_initial_state(self):
        self.close = True

    def set_up_parameters(self):
        self.declare_parameter('burst_time', 3.0)
        self.burst_time = self.get_parameter('burst_time').value

    def shoot_callback(self, msg):
        self.create_timer(self.burst_time, self.close_valve)
        self.open_valve()

    def open_valve(self):
        if not self.open:
            self.get_logger().info("Valve opened for shooting.")
            self.open = True
            self.valve_state_pub.publish(Bool(data=self.open))
            #TODO: Implement valve opening logic here
            #Probably a servo mavlink command

    def close_valve(self):
        
        if self.open:
            self.destroy_timer(self.close_valve)

            self.get_logger().info("Valve closed after shooting.")
            self.open = False  
            self.valve_state_pub.publish(Bool(data=self.open))
            self.request_picture_pub.publish(Bool(data=True))

            #TODO: Implement valve closing logic here


    def set_up_topics(self):
        qos_reliable = self._create_qos_profile(QoSReliabilityPolicy.RELIABLE)
  

        self.shoot_pub = self.create_subscription(
            Bool,
            '/shoot_topic',
            self.shoot_callback,
            qos_reliable
        )

        self.valve_state_pub = self.create_publisher(
            Bool,
            '/valve_state',
            qos_reliable
        )

        self.request_picture_pub = self.create_publisher(
            Bool,
            '/request_picture',
            qos_reliable
        )




def main(args=None):
    rclpy.init(args=args)
    node = AutonomousApproach()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()