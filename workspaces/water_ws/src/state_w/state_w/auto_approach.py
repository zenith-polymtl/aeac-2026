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
        self.active = False
        self.target_acquired = False
        self.waiting_for_approach = False
        self.waiting_for_aim = False

    def set_up_parameters(self):
        self.declare_parameter('approach_radius', 2.0)
        self.declare_parameter('approach_height', 1.0)

        self.approach_radius = self.get_parameter('approach_radius').value
        self.approach_height = self.get_parameter('approach_height').value

    def set_up_topics(self):
        qos_reliable = self._create_qos_profile(QoSReliabilityPolicy.RELIABLE)
        self.auto_approach_sub = self.create_subscription(
            Bool,
            '/auto_approach',
            self.auto_approach_activation_callback,
            10
        )

        self.target_acquired_sub = self.create_subscription(
            Bool,
            '/target_acquired',
            self.target_acquired_callback,
            10
        )

        self.in_position_sub = self.create_subscription(
            Bool,
            '/in_position',
            self.in_position_callback,
            10
        )

        self.aim_pub = self.create_publisher(
            Bool,
            '/aim_topic',
            qos_reliable
        )
        #meow
        self.aimed_sub = self.create_subscription(
            Bool,
            '/aimed_topic',
            self.aimed_callback,
            qos_reliable
        )

        self.shoot_pub = self.create_publisher(
            Bool,
            '/shoot_topic',
            qos_reliable
        )

        self.target_pub = self.create_publisher(
            TargetPosePolar,
            '/goal_pose_polar',
            qos_reliable
        )

        # Topics / Nodes needed
        # /auto_approach (Bool) - to activate/deactivate autonomous approach
        # /target_acquired (Bool) - to indicate if a target is acquired
        # /in_position (Bool) - to indicate if in position for payload deployment
        # /aim_topic (Bool) - to command aiming at the target
        # /aimed_topic (Bool) - to indicate if aiming is successful
        # /shoot_topic (Bool) - to command payload deployment

    def auto_approach_activation_callback(self, msg):
        if msg.data and not self.active:
            self.get_logger().info("Autonomous approach activated.")
            self.active = True
            if self.target_acquired:
                self.get_logger().info("Target acquired, commencing approach.")
                self.send_physical_approach_command()
            else:
                self.get_logger().info("No targets acquired, approach aborted.")

        elif not msg.data and self.active:
            self.get_logger().info("Autonomous approach deactivated.")
            self.define_initial_state()

    def target_acquired_callback(self, msg):
        self.target_acquired = msg.data
        if self.target_acquired:
            self.get_logger().info("Target acquired.")
            if self.active and not self.waiting_for_approach:
                self.get_logger().info("Commencing approach to acquired target.")
                self.send_physical_approach_command()
        else:
            self.get_logger().info("Target lost.")
            self.waiting_for_approach = False
            self.waiting_for_aim = False

    def in_position_callback(self, msg):
        if self.waiting_for_approach and msg.data:
            self.get_logger().info("In position for payload deployment.")
            self.waiting_for_approach = False
            self.aim_target()

    def aimed_callback(self, msg):
        if self.waiting_for_aim and msg.data:
            self.get_logger().info("Target aimed successfully. Payload can be deployed.")
            self.waiting_for_aim = False
            self.shoot_target()

    def send_physical_approach_command(self):
        target_msg = TargetPosePolar()
        target_msg.radius = self.approach_radius
        target_msg.z = self.approach_height
        target_msg.theta = None
        target_msg.relative = False
        self.target_pub.publish(target_msg)
        self.get_logger().info("Published physical approach command.")
        self.waiting_for_approach = True

    def aim_target(self):
        self.get_logger().info("Aiming at target for payload deployment.")
        self.waiting_for_aim = True
        aim_msg = Bool()
        aim_msg.data = True
        self.aim_pub.publish(aim_msg)

    def shoot_target(self):
        self.get_logger().info("Shooting at target.")
        shoot_msg = Bool()
        shoot_msg.data = True
        self.shoot_pub.publish(shoot_msg)


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