#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from custom_interfaces.msg import TargetPosePolar
from custom_interfaces.srv import LocateTarget, ConvertCamDistToLocalDist
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped
import numpy as np

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
        # self.target_aimed = False

    def set_up_parameters(self):
        self.declare_parameter('approach_radius', 2.0)
        self.declare_parameter('approach_height', 1.0)

        self.approach_radius = self.get_parameter('approach_radius').value
        self.approach_height = self.get_parameter('approach_height').value
        
    def set_up_topics(self):
        qos_reliable = self._create_qos_profile(QoSReliabilityPolicy.RELIABLE)
        
        # Autonomous Approach topics
        self.auto_approach_sub = self.create_subscription(
            Bool,
            '/aeac/external/auto_approach/start',
            self.auto_approach_activation_callback,
            qos_reliable
        )
        self.in_position_to_shoot_sub = self.create_subscription(
            Bool,
            '/aeac/internal/auto_approach/in_position',
            self.in_position_callback,
            qos_reliable
        )
        self.activate_polar_pub = self.create_publisher(
            String,
            '/aeac/internal/auto_approach/activate_polar',
            qos_reliable
        )
        self.polar_target_pub = self.create_publisher(
            TargetPosePolar,
            '/aeac/internal/auto_approach/target_position',
            qos_reliable
        )
        self.polar_center_location_pub = self.create_publisher(
            PoseStamped,
            '/aeac/internal/auto_shoot/center_location',
            qos_reliable
        )
        # Autonomous Shoot topics
        self.start_hr_aiming_pub = self.create_publisher(
            Bool,
            '/aeac/internal/auto_shoot/start_hr_aiming',
            qos_reliable
        )
        self.start_auto_shoot = self.create_subscription(
            Bool,
            '/aeac/external/auto_shoot/start',
            self.auto_shoot_callback,
            qos_reliable
        )
        self.target_in_aim_sub = self.create_subscription(
            Bool,
            '/aeac/internal/auto_shoot/target_in_aim',
            self.target_aimed_callback,
            qos_reliable
        )
        # Shoot procedure
        self.shoot_sub = self.create_subscription(
            Bool,
            '/aeac/external/shoot',
            self.shoot_target_callback,
            qos_reliable
        )
        self.take_picture_sub = self.create_subscription(
            Bool,
            '/aeac/external/take_picture',
            self.take_picture_callback,
            qos_reliable
        )
        self.shoot_pub = self.create_publisher(
            Bool,
            '/aeac/internal/shoot',
            qos_reliable
        )
        self.take_picture_pub = self.create_publisher(
            Bool,
            '/aeac/internal/take_picture',
            qos_reliable
        )
        
        # Service Clients
        self.locate_target_client = self.create_client(LocateTarget, '/aeac/auto_approach/locate_target')
        # self.transform_target_pos_client = self.create_client(ConvertCamDistToLocalDist, 'target_distance')

        while not self.locate_target_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Locate target service not available, waiting...')
        
        # while not self.transform_target_pos_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Convert target location service not available, waiting...')
            
        self.get_logger().info('Autonomous approach node started')
    
    def auto_approach_activation_callback(self, msg: Bool):
        """
        Activates or deactivates the autonomous approach.
        """
        self.get_logger().info('Auto approach command received')
        if msg.data and not self.auto_approach_active:
            self.get_logger().info("Autonomous approach activated.")
            self.auto_approach_active = True

            # Publish activation
            # activate_req = Bool()
            # activate_req.data = True
            # self.activate_polar_pub.publish(activate_req)
            
            # Call locate target service
            srv_request = LocateTarget.Request()
            srv_request.activate = True
            # TODO: Add any necessary fields to srv_request here depending on your .srv file
            
            future = self.locate_target_client.call_async(srv_request)
            future.add_done_callback(self.target_position_received_callback)
            
        elif not msg.data and self.auto_approach_active:
            self.get_logger().info("Autonomous approach deactivated.")
            self.define_initial_state()

    def target_position_received_callback(self, future: PoseStamped):
        try:
            # response = future.result()
            
            # srv_request = response         
            # transform_future = self.transform_target_pos_client.call_async(srv_request)
            # transform_future.add_done_callback(self.target_position_transformed_callback)
            self.target_acquired = True
            self.target_position_transformed_callback(future)

            
        except Exception as e:
            self.get_logger().error(f"Transform service call failed: {e}")
    
    def target_position_transformed_callback(self, future: PoseStamped):
        try:                        
            if not self.auto_approach_active:
                return
            
            response = future.result()
            self.polar_center_location_pub.publish(response.target)
            self.activate_polar_timer = self.create_timer(0.1, self.activate_polar_callback)

                    
        except Exception as e:
            self.get_logger().error(f"Failed to transform target position: {e}")
    
    def activate_polar_callback(self):
        self.activate_polar_timer.cancel()
        
        request = String()
        request.data = "start"
        self.activate_polar_pub.publish(request)
        
        start_aiming_req = Bool()
        start_aiming_req.data = True
        self.start_hr_aiming_pub.publish(start_aiming_req)
        
        self.send_polar_target_timer = self.create_timer(0.1, self.send_polar_target_pose)


    def send_polar_target_pose(self):
        self.send_polar_target_timer.cancel()

        request = TargetPosePolar()
        request.relative = False
        request.z = np.nan
        request.theta = np.nan
        request.r = 3.0
        self.polar_target_pub.publish(request)
        
        self.in_movement = True
          
    def auto_shoot_callback(self, msg: Bool):
        if msg.data:
            self.in_position = True
            start_aiming_req = Bool()
            start_aiming_req.data = True
            self.start_hr_aiming_pub.publish(start_aiming_req)

    def in_position_callback(self, msg: Bool):
        self.in_position = msg.data
        self.in_movement = msg.data
        if self.target_aimed and self.in_position:
            self.shoot()
    
    def shoot_target_callback(self, msg: Bool):
        if msg.data:
            self.shoot()
            
    def take_picture_callback(self, msg: Bool):
        if msg.data:
            self.take_picture()
    
    def target_aimed_callback(self, msg: Bool):
        # Note perso: Il faudrait peut etre regarder le temps avant la dernier reponse au cas ou le auto aim fail
        self.target_aimed = msg.data
        if self.target_aimed and self.in_position:
            self.shoot()
    
    def shoot(self):
        request = Bool()
        request.data = True
        self.shoot_pub.publish(request)
        self.take_picture()
        self.get_logger().info("Target shot and picture taken.")
    
    def take_picture(self):
        request = Bool()
        request.data = True
        self.take_picture_pub.publish(request)



def main(args=None):
    rclpy.init(args=args)
    node = AutonomousApproach()
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