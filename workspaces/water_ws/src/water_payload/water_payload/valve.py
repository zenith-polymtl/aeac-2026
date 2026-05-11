#!/usr/bin/env python3

from time import monotonic

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from custom_interfaces.msg import TargetPosePolar
from std_msgs.msg import Bool, String
from custom_interfaces.srv import ServoState

class AutonomousApproach(Node):
    def __init__(self):
        super().__init__('autonomous_approach')

        self.define_initial_state()
        self.set_up_parameters()
        self.set_up_topics()

        self.closed_pwm = 1000
        self.open_pwm = 2000
        self.servo_channel = 10
        self.close_timer = None


    @staticmethod
    def _create_qos_profile(reliability_policy):
        return QoSProfile(
            reliability=reliability_policy,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

    def define_initial_state(self):
        self.open = False
        self.valve_opened_at = None

    def set_up_parameters(self):
        self.declare_parameter('burst_time', 3.0)
        self.burst_time = self.get_parameter('burst_time').value

    def shoot_callback(self, msg):
        if self.close_timer is not None:
            self.close_timer.cancel()
        self.close_timer = self.create_timer(self.burst_time, self.close_valve)
        self.open_valve()

    def open_valve(self):
        if not self.open:
            self.send_servo(self.open_pwm)
            self.valve_opened_at = monotonic()

            self.get_logger().info("Valve opened for shooting.")
            self.open = True
            self.valve_state_pub.publish(Bool(data=self.open))

            

    def close_valve(self):
        
        if self.open:
            self.send_servo(self.closed_pwm)
            if self.close_timer is not None:
                self.close_timer.cancel()
                self.close_timer = None

            open_time = None
            if self.valve_opened_at is not None:
                open_time = monotonic() - self.valve_opened_at

            if open_time is not None:
                self.get_logger().info(
                    f"Valve closed after shooting. Measured open time: {open_time:.3f} s"
                )
            else:
                self.get_logger().info("Valve closed after shooting.")

            self.valve_opened_at = None
            self.open = False  
            self.valve_state_pub.publish(Bool(data=self.open))
            self.request_picture_pub.publish(Bool(data=True))

            


    def set_up_topics(self):
        qos_reliable = self._create_qos_profile(QoSReliabilityPolicy.RELIABLE)
  

        self.shoot_pub = self.create_subscription(
            Bool,
            '/shoot_topic',
            self.shoot_callback,
            qos_reliable
        )

        self.valve_state_pub = self.create_publisher(
            Bool,
            '/valve_state',
            qos_reliable
        )

        self.request_picture_pub = self.create_publisher(
            Bool,
            '/request_picture',
            qos_reliable
        )

        self.servo_cli = self.create_client(ServoState, '/aeac/external/payload/set_state')

    def send_servo(self, pwm, name="valve"):
        request = ServoState.Request()
        request.servo_num = self.servo_channel
        request.pwm = pwm

        self.get_logger().info(
            f"Sending {name} command: servo_num={request.servo_num}, pwm={request.pwm}"
        )

        future = self.servo_cli.call_async(request)
        future.add_done_callback(
            lambda fut: self.servo_response_callback(fut, name)
        )

    def servo_response_callback(self, future, name):
        try:
            response = future.result()
            self.get_logger().info(
                f"{name} response: success={response.success}, message='{response.message}'"
            )
        except Exception as e:
            self.get_logger().error(
                f"Servo service call failed for {name}: {e}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousApproach()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()