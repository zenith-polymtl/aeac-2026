import enum
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from mavros_msgs.srv import CommandLong

class PayloadState(enum):
    OPENED = 1
    CLOSED = 2

SERVO_NUM = 9  # SERVO9 = AUX1 typiquement
PWM_MIN = 1100
PWM_MID = 1500
PWM_MAX = 1900

class PayloadControler(Node):
    def __init__(self):
        super().__init__('PayloadControler')
        self.sub_open = self.create_subscription(String, '/open_payload', self.open_callback, 2)
        self.sub_close = self.create_publisher(String, '/close_payload', self.close_callback, 2)

        self.client = self.create_client(CommandLong, '/mavros/cmd/command')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for /mavros/cmd/command...')

        self.state: PayloadState = PayloadState.CLOSED
        self.is_active = False
    
    def send_servo(self, pwm):
        req = CommandLong.Request()
        req.broadcast = False
        # pt seulement DO_SET_SERVO ou MAV_CMD_DO_SET_SERVO
        # voir https://ardupilot.org/copter/docs/common-mavlink-mission-command-messages-mav_cmd.html
        req.command = CommandLong.Request.CMD_DO_SET_SERVO 
        req.confirmation = 0
        req.param1 = float(SERVO_NUM)
        req.param2 = float(pwm)
        req.param3 = req.param4 = req.param5 = req.param6 = req.param7 = 0.0
        return self.client.call_async(req)

    def timer_callback(self):
        self.send_servo(PWM_MIN)

    def open_callback(self):
        if self.state == PayloadState.OPENED:
            self.get_logger().info('payload already open')
            return
        if self.is_active:
            self.get_logger().error('payload is already performing task')
            return
        
        self.is_active = True
        try:
            future = self.send_servo(PWM_MID)
            rclpy.spin_until_future_complete(self, future)
            future.result()
            self.create_timer(5, self.timer_callback)
            self.state = PayloadState.OPENED
        except Exception as e:
            self.get_logger().error("Error occured while sending PWM request ", e)
        finally:
            self.is_active = False
        
        
    def close_callback(self):
        if self.state == PayloadState.CLOSED:
            self.get_logger().info('payload already closed')
            return
        if self.is_active:
            self.get_logger().error('payload is already performing task')
            return
        
        self.is_active = True
        try:
            future = self.send_servo(PWM_MID)
            rclpy.spin_until_future_complete(self, future)
            future.result()
            self.create_timer(5, self.timer_callback)
            self.state = PayloadState.CLOSED
        except Exception as e:
            self.get_logger().error("Error occured while sending PWM request ", e)

        finally:
            self.is_active = False


def main():
    rclpy.init()
    node = PayloadControler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()