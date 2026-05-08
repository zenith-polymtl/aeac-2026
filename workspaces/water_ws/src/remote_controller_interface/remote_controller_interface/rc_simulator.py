#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import RCIn
import sys
import select
import termios
import tty
import threading

class RCSimulator(Node):
    """ROS2 node to simulate RCIn messages from keyboard input."""

    def __init__(self):
        super().__init__('rc_simulator')

        # Publisher for simulated RCIn
        self.pub = self.create_publisher(RCIn, '/mavros/rc/in', 10)

        # Timer to publish at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_callback)

        # Save terminal settings
        self.settings = termios.tcgetattr(sys.stdin)

        # Initialize channels (PWM values, 16 channels for compatibility)
        self.channels = [1500] * 16
        
        self.stop_event = threading.Event()

        # Start keyboard reading thread
        self.keyboard_thread = threading.Thread(target=self.keyboard_loop)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()

        self.get_logger().info("RC Simulator started")

    def keyboard_loop(self):
        while rclpy.ok() and not self.stop_event.is_set():
            key = self.get_key()
            if key:
                self.process_key(key)

    def get_key(self):
        """Get a single key press non-blockingly."""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def process_key(self, key):
        """Update channels based on key press."""
        step = 50  # PWM adjustment step
        low_pwm = 1100
        high_pwm = 1900
        center_pwm = 1500
        
        if key == 'w':
            self.channels[7] = high_pwm
        elif key == 's':
            self.channels[7] = center_pwm
        elif key == 'x':
            self.channels[7] = low_pwm
        elif key == 'e':
            self.channels[8] = high_pwm
        elif key == 'd':
            self.channels[8] = center_pwm
        elif key == 'c':
            self.channels[8] = low_pwm
        elif key == 'r':
            self.channels[9] = high_pwm
        elif key == 'f':
            self.channels[9] = center_pwm
        elif key == 'v':
            self.channels[9] = low_pwm
        
        elif key == 'q':
            self.stop_event.set()
            self.get_logger().info("===  STOP ===")
            raise KeyboardInterrupt("Stopping")
        # Log changes (optional, for debugging)
        self.get_logger().info(f"Channels updated: {self.channels[0:9]}")
        
        self.publish_callback()
        
    def publish_callback(self):
        """Publish current RCIn message."""
        msg = RCIn()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.channels = self.channels
        msg.rssi = 255  # Simulated full signal
        self.pub.publish(msg)

    def destroy_node(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RCSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        sys.exit()

if __name__ == '__main__':
    main()