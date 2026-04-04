#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from custom_interfaces.msg import TargetPosePolar, UiMessage
from std_msgs.msg import Bool, String, UInt8, Empty
from geometry_msgs.msg import PoseStamped, Point, Pose
from zed_msgs.msg import ObjectsStamped
import numpy as np
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose

class GimbalMode:
    LOCK = 0
    FOLLOW = 1
    AUTO_AIM = 2

class AutonomousApproach(Node):
    def __init__(self):
        super().__init__('autonomous_approach')

        self.define_initial_state()
        self.set_up_parameters()
        self.set_up_topics()
        
        self.get_logger().info(f"Autonomous Approach node launched")

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
        self.auto_aim_enable = False
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
        
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.message_to_ui_pub = self.create_publisher(
            UiMessage,
            'aeac/external/send_to_ui',
            qos_reliable
        )
        
        self.abort_publisher = self.create_subscription(
            Bool,
            '/aeac/external/mission/abort_all',
            self.abort_callback,
            qos_reliable
            
        )
        
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
        self.target_position_sub = self.create_subscription(
            ObjectsStamped,
            '/aeac/internal/auto_approach/target_detected',
            self.target_position_received_callback,
            qos_reliable
        )
        self.trigger_target_detection_pub = self.create_publisher(
            Empty,
            '/aeac/internal/auto_approach/detect_target',
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
        self.start_target_detection_pub = self.create_publisher(
            Bool,
            '/aeac/internal/auto_shoot/start_hr_aiming',
            qos_reliable
        )
        self.start_auto_shoot_pub = self.create_publisher(
            UInt8,
            '/aeac/external/gimbal/set_mode',
            qos_reliable
        )
        self.target_in_aim_sub = self.create_subscription(
            Bool,
            '/aeac/internal/auto_shoot/target_in_aim',
            self.target_aimed_callback,
            qos_reliable
        )
        self.auto_shoot_sub = self.create_subscription (
            Bool,
            '/aeac/external/auto_shoot/start',
            self.auto_shoot_callback,
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
            Empty,
            '/aeac/internal/take_picture',
            qos_reliable
        )
        
        # # Service Clients
        # self.locate_target_client = self.create_client(LocateTarget, '/aeac/auto_approach/locate_target')
        # # self.transform_target_pos_client = self.create_client(ConvertCamDistToLocalDist, 'target_distance')

        # while not self.locate_target_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Locate target service not available, waiting...')
        
        # while not self.transform_target_pos_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Convert target location service not available, waiting...')
                
    def auto_approach_activation_callback(self, msg: Bool):
        """
        Activates or deactivates the autonomous approach.
        """
        self.get_logger().info('Auto approach command received')
        if msg.data and not self.auto_approach_active:
            self.get_logger().info("Autonomous approach activated.")
            self.auto_approach_active = True
                        
            # Call locate target service
            # srv_request = LocateTarget.Request()
            # srv_request.activate = True
            # TODO: Add any necessary fields to srv_request here depending on your .srv file
            req = Empty()
            self.trigger_target_detection_pub.publish(req)
            
            # future = self.locate_target_client.call_async(srv_request)
            # future.add_done_callback(self.target_position_received_callback)
            
        elif not msg.data and self.auto_approach_active:
            self.get_logger().info("Autonomous approach deactivated.")
            self.define_initial_state()

    def target_position_received_callback(self, msg: ObjectsStamped):
        try:
            if len(msg.objects) == 0:
                self.send_message_to_ui("No object detected", Falses)
            
            best_confidence = 0
            best_found_object = None
            for obj in msg.objects:
                if best_confidence < obj.confidence:
                    best_confidence = obj.confidence
                    best_found_object = obj

            self.send_message_to_ui(f"Object {best_found_object.label} detected with {best_confidence*100}% confidence at position: {best_found_object.position}")

            self.target_acquired = True
        
            pose_stamped = PoseStamped()
            pose_stamped.header = msg.header

            target_position = Pose()
            target_position.position.x = float(best_found_object.position[0])
            target_position.position.y = float(best_found_object.position[1])
            target_position.position.z = float(best_found_object.position[2])
            
            transform = self.tf_buffer.lookup_transform(
                'zed_camera_center',
                # 'map',
                'base_link',
                rclpy.time.Time()
            )
            pose_stamped.pose = do_transform_pose(target_position, transform)
            self.polar_center_location_pub.publish(pose_stamped)
            
            # self.activate_polar_timer = self.create_timer(1.0, self.activate_polar_callback)
            self.activate_polar_callback()

            
        except Exception as e:
            self.get_logger().error(f"Transform service call failed: {e}")
    
    def send_message_to_ui(self, msg: String, is_success: Bool = True):
        req = UiMessage()
        req.message = msg
        req.is_success = True
        
        self.message_to_ui_pub.publish(req)
    
    def abort_callback(self, msg: Bool):
        if msg.data:
            self.define_initial_state()
            self.send_message_to_ui("Aborted command processed!")

            
    def activate_polar_callback(self):        
        request = String()
        request.data = "start"
        self.activate_polar_pub.publish(request)
        
        # self.toggle_auto_aim()
        
        self.send_polar_target_pose()
        # self.send_polar_target_timer = self.create_timer(0.1, self.send_polar_target_pose)
        # self.activate_polar_timer.destroy()

    def send_polar_target_pose(self):
        # self.send_polar_target_timer.cancel()

        request = TargetPosePolar()
        request.relative = False
        request.z = np.nan
        request.theta = np.nan
        request.r = 3.0
        self.polar_target_pub.publish(request)
        self.send_message_to_ui(f"Starting auto approach with distance {request.r}")
        self.in_movement = True
          
    def auto_shoot_callback(self, msg: Bool):
        self.get_logger().info(f"auto shoot callback. msg.data: {msg.data}, self.auto_aim_enable: {self.auto_aim_enable}")
        if msg.data != self.auto_aim_enable:
            self.get_logger().info("Toggling auto aim.")
            self.toggle_auto_aim()

    def toggle_auto_aim(self):
        self.auto_aim_enable = not self.auto_aim_enable
        
        start_target_detection_req = Bool()
        start_target_detection_req.data = self.auto_aim_enable
        self.start_target_detection_pub.publish(start_target_detection_req)
        
        start_auto_aim_req = UInt8()
        start_auto_aim_req.data = GimbalMode.AUTO_AIM if self.auto_aim_enable else GimbalMode.FOLLOW
        self.start_auto_shoot_pub.publish(start_auto_aim_req)
        
    
    def in_position_callback(self, msg: Bool):
        self.in_position = msg.data
        self.in_movement = msg.data
        self.send_message_to_ui(f"Target reached. Ready to shoot")
        if self.target_aimed and self.in_position:
            pass
            # self.shoot()
    
    def shoot_target_callback(self, msg: Bool):
        if msg.data:
            self.shoot()
            
    def take_picture_callback(self, msg: Bool):
        if msg.data:
            self.take_picture()
    
    def target_aimed_callback(self, msg: Bool):
        # Note perso: Il faudrait peut etre regarder le temps avant la dernier reponse au cas ou le auto aim fail
        self.target_aimed = msg.data
        self.get_logger().info(f"Target in aim: {self.target_aimed}")
        if self.target_aimed and self.in_position:
            self.shoot()
    
    def shoot(self):
        return
        request = Bool()
        request.data = True
        self.shoot_pub.publish(request)
        self.take_picture()
        self.get_logger().info("Target shot and picture taken.")
    
    def take_picture(self):
        request = Empty()
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