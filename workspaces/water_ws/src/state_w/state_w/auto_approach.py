#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from custom_interfaces.msg import TargetPosePolar
from custom_interfaces.srv import LocateTarget, ConvertCamDistToLocalDist
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
        self.auto_approach_active = False
        self.target_acquired = False
        self.in_movement = False
        self.in_position = False
        self.target_aimed = False

    def set_up_parameters(self):
        self.declare_parameter('approach_radius', 2.0)
        self.declare_parameter('approach_height', 1.0)

        self.approach_radius = self.get_parameter('approach_radius').value
        self.approach_height = self.get_parameter('approach_height').value

    def set_up_topics(self):
        qos_reliable = self._create_qos_profile(QoSReliabilityPolicy.RELIABLE)
        # Autonomous Approach topics
        # Activate auto_approach
        self.auto_approach_sub = self.create_subscription(
            Bool,
            '/aeac/external/auto_approach/start',
            self.auto_approach_activation_callback,
            10
        )
        # Triggered when polar finished
        self.in_position_to_shoot_sub = self.create_subscription(
            Bool,
            '/aeac/internal/auto_approach/in_position',
            self.in_position_callback,
            10
        )
        # Activate polar for auto approach
        self.activate_polar_pub = self.create_publisher(
            Bool,
            '/aeac/internal/auto_approach/activate_polar',
            qos_reliable
        )
        # Send polar position for auto approach
        self.polar_target_pub = self.create_publisher(
            TargetPosePolar,
            '/aeac/internal/auto_approach/target_position',
            qos_reliable
        )
        # Autonomous Shoot topcis
        # Start the vision module for auto aim
        self.start_HR_aiming_pub = self.create_publisher(
            Bool,
            '/aeac/internal/auto_shoot/start_hr_aiming',
            qos_reliable
        )
        self.target_in_aim_sub = self.create_subscription(
            Bool,
            '/aeac/internal/auto_shoot/target_in_aim',
            self.target_aimed_callback,
            10
        )
        # Shoot procedure
        # Triggered by outside operator
        self.shoot_sub = self.create_subscription(
            Bool,
            '/aeac/external/shoot',
            self.shoot_target_callback,
            10
        )
        self.shoot_sub = self.create_publisher(
            Bool,
            '/aeac/internal/shoot',
            qos_reliable
        )
        # Service that ask for target detection
        self.locate_target_client = self.create_client(LocateTarget, 'locate_target')
        # Service that ask that tansform the target position
        self.transfrom_target_pos_client = self.create_client(ConvertCamDistToLocalDist, 'target_distance')

        while not self.locate_target_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('locate target service not available, waiting...')
        
        while not self.transfrom_target_pos_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('convert target location not available, waiting...')
    
    def auto_approach_activation_callback(self, msg: Bool):
        """
        This function is used to activate or deactivate the autonomus approach
        
        Parameters:
            msg: activate or deactivate the autonomus approach
        """
        # Check for activation
        if msg.data and not self.auto_approach_active:
            self.get_logger().info("Autonomous approach activated.")
            request = Bool()
            request.data = True
            auto_approach_active = True
            self.detect_target_pub.publish(request)
        # Check for deactivation
        elif not msg.data and self.auto_approach_active:
            self.get_logger().info("Autonomous approach deactivated.")
            self.define_initial_state()

    def auto_shoot_activation_callback(self, msg):
        pass
    

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