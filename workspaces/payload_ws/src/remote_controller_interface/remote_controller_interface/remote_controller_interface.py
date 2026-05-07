#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from custom_interfaces.srv import ServoState

from mavros_msgs.msg import RCIn, State
from mavros_msgs.srv import MessageInterval, SetMode

class RemoteControlInterface(Node):
    def __init__(self):
        super().__init__('remote_control_interface')
        
        self._declare_parameters()

        self.controls = {
            'camera':    {'rc_ch': 13, 'servo_ch' : 13, 'last_state': None,  "LOW" : 1850, "MIDDLE" : 1000 ,"HIGH" : 700},
            'servo_1':   {'rc_ch': 7, 'servo_ch' : 9, 'last_state': None, "LOW" : 2050, "MIDDLE" : 1700 ,"HIGH" : 1600},
            'lap_controle' : {'rc_ch': 9, 'last_state': None},
            'servo_2':   {'rc_ch': 8,'servo_ch' : 10, 'last_state': None, "LOW" : 1000, "MIDDLE" : 1500, "HIGH" : 2000},
            # 'polar_lock':{'ch': 9, 'last_state': None},
        }
        
        self.inital_state()

        qos_be = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        qos_re = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.rc_sub = self.create_subscription(RCIn, '/mavros/rc/in', self.rc_callback, qos_be)
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_callback, qos_re)

        self.get_logger().info("Remote Controller Interface Initialized")
        
        self.servo_cli = self.create_client(ServoState, '/aeac/external/payload/set_state')
        
        while not self.servo_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

    def inital_state(self):
        self.requested_polar_lock = False
    
    def _declare_parameters(self):
        self.declare_parameter('threshold_high', 1700)
        self.declare_parameter('threshold_low', 1300)
        
        self.high_threshold = self.get_parameter('threshold_high').value
        self.low_threshold = self.get_parameter('threshold_low').value

    def rc_callback(self, msg):
        if len(msg.channels) < 10:
            return

        for name, data in self.controls.items():
            val = msg.channels[data['rc_ch']]
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

            case 'servo_1'| 'servo_2':
                if state == "HIGH":
                    self.get_logger().info(f"Opening {name}")
                    
                elif state == "LOW":
                    self.get_logger().info(f"Closing {name}")
                self.send_servo(name, state)

            case 'polar_lock':
                self.polar_change(state)
                
            case 'lap_control':
                self.lap_controle_change(state)

    def send_servo(self, name, state):
        request = ServoState.Request()
        request.servo_num = self.controls[name]["servo_ch"]

        request.pwm = self.controls[name][state]
        self.get_logger().info(f"Sending {name} command: servo_num={request.servo_num}, pwm={request.pwm}") 

        future = self.servo_cli.call_async(request)
        future.add_done_callback(lambda fut: self.servo_response_callback(fut, name))

    def servo_response_callback(self, future, name):
        try:
            response = future.result()
            self.get_logger().info(
                f"{name} response: success={response.success}, message='{response.message}'"
            )
        except Exception as e:
            self.get_logger().error(f"Servo service call failed for {name}: {e}")

    def camera_change(self, state):
        match state:
            case "LOW":
                self.get_logger().info("Camera: Setting LOW angle (e.g., 0°)")
            case "MIDDLE":
                self.get_logger().info("Camera: Setting MIDDLE angle (e.g., 45°)")
            case "HIGH":
                self.get_logger().info("Camera: Setting HIGH angle (e.g., 90°)")

        self.send_servo("camera", state)
    
    def polar_change(self, state):
        if state == "HIGH":
            self.get_logger().info("Requesting Polar Lock...")
            self.set_mavros_mode("GUIDED")
            self.requested_polar_lock = True
        else:
            self.requested_polar_lock = False
    
    def lap_controle_change(self, state):
        if state == "HIGH":
            self.get_logger().info("Finishing current lap and going to site")
        else:
            self.get_logger().info("Stopping lap now and going to site")
        
    def set_mavros_mode(self, mode_str):
        req = SetMode.Request()
        req.custom_mode = mode_str
        self.set_mode_client.call_async(req)

    def state_callback(self, msg):  
        if msg.mode == "GUIDED" and self.requested_polar_lock:
            self.get_logger().info('Vehicle in GUIDED. Activating Polar Lock logic...')
            self.requested_polar_lock = False

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