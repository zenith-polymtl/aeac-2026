import rclpy  
from rclpy.node import Node  
from mavros_msgs.msg import RCIn  
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy  


class RemoteControllerInterface(Node):  
    def __init__(self):  
        super().__init__('remote_controller_interface')  
          
        # Use best effort QoS to match MAVROS sensor data  
        qos_profile = QoSProfile(  
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  
            history=QoSHistoryPolicy.KEEP_LAST,  
            depth=10  
        )  
          
        # Subscribe to RC input channels  
        self.rc_sub = self.create_subscription(  
            RCIn,  
            '/mavros/rc/in',  
            self.rc_callback,  
            qos_profile  
        )
    
    def rc_callback(self, msg):  
        if len(msg.channels) >= 4:  
            roll_raw = msg.channels[0]      # Channel 1  
            pitch_raw = msg.channels[1]     # Channel 2    
            throttle_raw = msg.channels[2]  # Channel 3  
            yaw_raw = msg.channels[3]       # Channel 4  
            activation = msg.channels[6]
            if activation > 1700:
                self.active = True
            elif activation < 1300:
                self.active = False
              
            # Convert PWM values (typically 1000-2000) to normalized values (-1 to 1 for roll/pitch/throttle)  
            self.roll = self.pwm_to_normalized(roll_raw)  
            self.pitch = self.pwm_to_normalized(pitch_raw)  
            self.throttle = self.pwm_to_normalized(throttle_raw)  
            self.yaw = self.pwm_to_normalized(yaw_raw)

        else:  
            self.get_logger().warn(f"Insufficient RC channels: {len(msg.channels)}")

    def close_system(self):
        msg = String()
        msg.data = 'stop'
        self.activation_pub.publish(msg)

    def start_position(self):
        msg = String()
        msg.data = 'start'
        self.activation_pub.publish(msg)
      
    def pwm_to_normalized(self, pwm_value, center=1500, deadband=50):  
        """Convert PWM value to normalized range [-1, 1] with center at 1500"""  
        if abs(pwm_value - center) < deadband:  
            return 0.0  
        return (pwm_value - center) / 500.0  
  
def main(args=None):  
    rclpy.init(args=args)  
    node = RemoteControllerInterface()  
      
    try:  
        rclpy.spin(node)  
    except KeyboardInterrupt:  
        pass  
    finally:  
        node.destroy_node()  
        rclpy.shutdown()  
  
if __name__ == '__main__':  
    main()