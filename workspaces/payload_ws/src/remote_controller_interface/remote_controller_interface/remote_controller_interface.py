#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from mavros_msgs.msg import RCIn
from std_msgs.msg import String
from mavros_msgs.srv import MessageInterval 


class RemoteControlInterface(Node):
    """ROS2 node for reading RC channel inputs and triggering events."""

    def __init__(self):
        super().__init__('remote_control_interface')
        
        # Create QoS profiles
        qos_best_effort = self._create_qos_profile(QoSReliabilityPolicy.BEST_EFFORT)
        qos_reliable = self._create_qos_profile(QoSReliabilityPolicy.RELIABLE)

        # Using MAVROS Python client  
        self.msg_interval_client = self.create_client(MessageInterval, '/mavros/set_message_interval')  


        # TODO Verify if this initalisation is really necessary
        # Set up message intervals after a short delay  
        # self.setup_timer = self.create_timer(1.0, self.setup_message_intervals)  
        
        # Initialize state variables
        self.active_servo_1 = False
        self.active_servo_2 = False
        self.last_active_servo_1 = self.active_servo_1
        self.last_active_servo_2 = self.active_servo_2
        
        # Declare parameters
        self._declare_parameters()
        self._load_parameters()

        # Create subscriptions
        self.rc_sub = self.create_subscription(
            RCIn,
            '/mavros/rc/in',
            self.rc_callback,
            qos_best_effort
        )

        # Create publishers
        self.servo_pub = self.create_publisher(
            String,
            '/servo_topic',
            qos_reliable
        )
        
        self.get_logger().info("Remote Controller Interface started")

    @staticmethod
    def _create_qos_profile(reliability_policy):
        """Create a QoS profile with specified reliability policy."""
        return QoSProfile(
            reliability=reliability_policy,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

    def _declare_parameters(self):
        """Declare all ROS2 parameters."""
        self.declare_parameter('talk', True)
        self.declare_parameter('servo_1_channel_controller', 8)
        self.declare_parameter('servo_2_channel_controller', 9)

    def _load_parameters(self):
        """Load parameter values from ROS2 parameter server."""
        self.talk = self.get_parameter('talk').value
        self.servo_1_channel = self.get_parameter('servo_1_channel_controller').value
        self.servo_2_channel = self.get_parameter('servo_2_channel_controller').value  

    def handle_activation(self, msg):
        """Process activation input from RC channel."""
        if len(msg.channels) < 8:
            self.get_logger().warn(f"Insufficient RC channels for activation: {len(msg.channels)}")
            return

        servo_1_input = msg.channels[7] # Channel 8
        servo_2_input = msg.channels[8] # Channel 9
        
        if servo_1_input > 1700:
            self.active_servo_1 = True
        elif servo_1_input < 1300:
            self.active_servo_1 = False
        
        if servo_2_input > 1700:
            self.active_servo_2 = True
        elif servo_2_input < 1300:
            self.active_servo_2 = False
        
    def handle_servo_controls(self):
        if self.active_servo_1 and not self.last_active_servo_1:
            self.get_logger().info(f"Published open command for servo_1")
        elif not self.active_servo_1 and self.last_active_servo_1:
            self.get_logger().info(f"Published close command for servo 1")
            
        if self.active_servo_2 and not self.last_active_servo_2:
            self.get_logger().info(f"Published open command for servo_2")
            # self.start_position()
            pass
        elif not self.active_servo_2 and self.last_active_servo_2:
            self.get_logger().info(f"Published close command for servo 2")
            pass
        
        self.last_active_servo_1 = self.active_servo_1
        self.last_active_servo_2 = self.active_servo_2
      
    def rc_callback(self, msg):
        """
        Process incoming RC channel data.

        Ignores channels 1-4.
        - Channel 8: Servo 1 control (temp)
        - Channel 9: Servo 2 control (temp)
        """
        self.handle_activation(msg)
        self.handle_servo_controls()
        
    def setup_message_intervals(self):
        if True:
            """Set up message intervals after node initialization"""  
            if not self.msg_interval_client.wait_for_service(timeout_sec=5.0):  
                self.get_logger().warn('Message interval service not available, aborting request...')  
                self.destroy_timer(self.setup_timer) 
                return  
            
            request = MessageInterval.Request()  
            request.message_id = 65  
            request.message_rate = 25.0  
            
            future = self.msg_interval_client.call_async(request)  
            future.add_done_callback(self.message_interval_callback)  
            
        # Destroy the timer since we only need to run this once  
        self.destroy_timer(self.setup_timer) 

    def message_interval_callback(self, future):  
        try:  
            response = future.result()  
            if response.success:  
                self.get_logger().info("Message interval set successfully")  
            else:  
                self.get_logger().error("Failed to set message interval")  
        except Exception as e:  
            self.get_logger().error(f"Service call failed: {e}") 
        

def main(args=None):
    """Main entry point for the RC Event Trigger node."""
    rclpy.init(args=args)
    node = RemoteControlInterface()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()