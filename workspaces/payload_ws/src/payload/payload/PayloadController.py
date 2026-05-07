import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandLong
from custom_interfaces.srv import ServoState
from custom_interfaces.msg import UiMessage, ServoControl
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from dataclasses import dataclass

@dataclass
class mavros_servo_response:
    success: bool
    message: str 

class PayloadController(Node):
    def __init__(self):
        super().__init__('PayloadController')
        
        reliable_qos = QoSProfile(depth=10)
        reliable_qos.reliability = ReliabilityPolicy.RELIABLE

        self.callback_group = ReentrantCallbackGroup()

        self.srv = self.create_service(ServoState, '/aeac/external/payload/set_state', self.handle_servo_request, callback_group=self.callback_group)

        self.ui_notification_pub = self.create_publisher(UiMessage, '/aeac/external/UI/display', reliable_qos) 
        self.mavros_client = self.create_client(CommandLong, '/mavros/cmd/command', callback_group=self.callback_group)
        while not self.mavros_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for /mavros/cmd/command...')

        self.servo_external_sub = self.create_subscription(ServoControl, '/aeac/external/payload/toggle_servo', self.external_servo_callback, reliable_qos)

        self.get_logger().info('PayloadController Service Ready')

    async def external_servo_callback(self, msg: ServoControl):        
        await self.trigger_servo(msg.servo_num, msg.pwm)

    async def trigger_servo(self, servo_num, pwm):
        self.get_logger().info(f'Setting payload servo {servo_num} to PWM {pwm}')

        mavros_req = CommandLong.Request()
        mavros_req.broadcast = False
        # see https://ardupilot.org/copter/docs/common-mavlink-mission-command-messages-mav_cmd.html
        mavros_req.command = 183 # MAV_CMD_DO_SET_SERVO
        mavros_req.confirmation = 0
        mavros_req.param1 = float(servo_num)
        mavros_req.param2 = float(pwm)
        mavros_req.param3 = mavros_req.param4 = mavros_req.param5 = mavros_req.param6 = mavros_req.param7 = 0.0

        response = mavros_servo_response(success=False, message="")
        
        try:
            future = self.mavros_client.call_async(mavros_req)
            mavros_result = await future

            if mavros_result.success:
                response.success = True
                response.message = f"The servo {servo_num} was succesfully " + ("opened" if pwm >= 2000 else "closed")
            else:
                response.success = False
                response.message = f"MAVROS has rejected request (Result: {mavros_result.result})"
                
        except Exception as e:
            response.success = False
            response.message = f"Error while processing request: {str(e)}"
            self.get_logger().error(response.message)

        notification = UiMessage()
        notification.message = response.message
        notification.is_success = response.success
        self.ui_notification_pub.publish(notification)
        
        return response
    
    async def handle_servo_request(self, request, response):
        res = await self.trigger_servo(request.servo_num, request.pwm)
        response.success = res.success
        response.message = res.message
        return response

def main():
    rclpy.init()
    node = PayloadController()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()