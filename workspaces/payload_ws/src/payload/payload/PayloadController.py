import enum
import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandLong

# could make it a parameter to support multiple motor
SERVO_NUM = 9

PWM_MIN = 1100
PWM_MID = 1500
PWM_MAX = 1900

class PayloadState(enum):
    OPENED = PWM_MID
    CLOSED = PWM_MIN

class PayloadController(Node):
    def __init__(self):
        super().__init__('PayloadController')
        self.sub_open = self.create_subscription(int, '/payload/set_state', self.send_servo, 2)

        self.client = self.create_client(CommandLong, '/mavros/cmd/command')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for /mavros/cmd/command...')

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

def main():
    rclpy.init()
    node = PayloadController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()