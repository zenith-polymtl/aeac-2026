import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandLong
from custom_interfaces.srv import ServoState 

class PayloadController(Node):
    def __init__(self):
        super().__init__('PayloadController')

        self.srv = self.create_service(ServoState, '/payload/set_state', self.handle_servo_request)

        self.mavros_client = self.create_client(CommandLong, '/mavros/cmd/command')
        while not self.mavros_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for /mavros/cmd/command...')

        self.get_logger().info('PayloadController Service Ready')

    async def handle_servo_request(self, request, response):
        self.get_logger().info(f'Setting payload servo {request.servo_num} to PWM {request.pwm}')

        mavros_req = CommandLong.Request()
        mavros_req.broadcast = False
        # see https://ardupilot.org/copter/docs/common-mavlink-mission-command-messages-mav_cmd.html
        mavros_req.command = 183 # MAV_CMD_DO_SET_SERVO
        mavros_req.confirmation = 0
        mavros_req.param1 = float(request.servo_num)
        mavros_req.param2 = float(request.pwm)
        mavros_req.param3 = mavros_req.param4 = mavros_req.param5 = mavros_req.param6 = mavros_req.param7 = 0.0

        try:
            future = self.mavros_client.call_async(mavros_req)
            mavros_result = await future

            if mavros_result.success:
                response.success = True
                response.message = ""
            else:
                response.success = False
                response.message = f"MAVROS has rejected request (Result: {mavros_result.result})"
                
        except Exception as e:
            response.success = False
            response.message = f"Error while processing request: {str(e)}"
            self.get_logger().error(response.message)

        return response

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