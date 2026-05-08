#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from custom_interfaces.srv import ServoState

from std_msgs.msg import Bool
from mavros_msgs.msg import RCIn, State
from mavros_msgs.srv import MessageInterval, SetMode

class RemoteControlInterface(Node):
    def __init__(self):
        super().__init__('remote_control_interface')
        
        self._declare_parameters()

        self.controls = {
            'camera':    {'ch': 5, 'last_state': None},
            'mission_action_state':   {'ch': 7, 'last_state': None},
            'mode_triggre' : {'ch': 8, 'last_state': None},
            'flight_mode_switch' : {'ch': 9, 'last_state': None},
        }
        
        self.inital_state()

        qos_be = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        qos_re = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        
        self.rc_sub = self.create_subscription(RCIn, '/mavros/rc/in', self.rc_callback, qos_be)
        # self.state_sub = self.create_subscription(State, '/mavros/state', self.state_callback, qos_re)
        
        
        self.auto_approach_pub = self.create_publisher(Bool, '/aeac/external/auto_approach/start', qos_re)
        self.auto_shoot_pub = self.create_publisher(Bool, '/aeac/external/auto_shoot/start', qos_re)
        self.shoot_pub = self.create_publisher(Bool, '/aeac/external/shoot', qos_re)
        self.take_picture_pub = self.create_publisher(Bool, '/aeac/external/take_picture', qos_re)

        self.get_logger().info("Remote Controller Interface Initialized")
        
        # self.servo_cli = self.create_client(ServoState, '/aeac/external/payload/set_state')
        
        # while not self.servo_cli.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Service not available, waiting...')

    def inital_state(self):
        self.mode_triggered = False
    
    def _declare_parameters(self):
        self.declare_parameter('threshold_high', 1700)
        self.declare_parameter('threshold_low', 1300)
        
        self.high_threshold = self.get_parameter('threshold_high').value
        self.low_threshold = self.get_parameter('threshold_low').value

    def rc_callback(self, msg):
        if len(msg.channels) < 10:
            return

        for name, data in self.controls.items():
            val = msg.channels[data['ch']]
            current_state = self.get_switch_state(val)

            if current_state != data['last_state']:
                self.handle_switch_change(name, current_state)
                data['last_state'] = current_state
                
    def get_switch_state(self, pwm):
        """Maps PWM values to discrete states."""
        if pwm < 1300:
            return "LOW"
        elif 1300 <= pwm <= 1700:
            return "MIDDLE"
        else:
            return "HIGH"
        
    def handle_switch_change(self, name, state):
        """Centralized command center for all RC events."""
        match name :
            case "camera":
                self.camera_change(state)

            case 'mission_action_state':
                # No action are needed because we simply store the mode requested
                pass
                
            case 'mode_triggre':
                if state == "HIGH":
                    self.tigger_pipline_action()
            case 'flight_mode_switch':
                if state == "MIDDLE":
                    self.trigger_auto_approach()

    def camera_change(self, state):
        match state:
            case "LOW":
                self.get_logger().info("Camera: Setting LOW angle (e.g., 0°)")
            case "MIDDLE":
                self.get_logger().info("Camera: Setting MIDDLE angle (e.g., 45°)")
            case "HIGH":
                self.get_logger().info("Camera: Setting HIGH angle (e.g., 90°)")        

    def tigger_pipline_action(self):
        req = Bool()
        req.data = True
        match self.controls['mission_action_state']['last_state']:
            case "LOW":
                # Auto Aim
                self.get_logger().info("Auto Aim")
                self.auto_shoot_pub.publish(req)
            case "MIDDLE":
                # Shoot
                self.get_logger().info("Shoot")
                self.shoot_pub.publish(req)
            case "HIGH":
                # Take Picture
                self.get_logger().info("Picture")
                self.take_picture_pub.publish(req)

    def trigger_auto_approach(self):
        req = Bool()
        req.data = True
        self.auto_approach_pub.publish(req)

def main(args=None):
    rclpy.init(args=args)
    node = RemoteControlInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()