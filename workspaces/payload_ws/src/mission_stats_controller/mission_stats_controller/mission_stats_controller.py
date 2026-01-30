import rclpy
from rclpy.node import Node
from custom_interfaces.msg import PayloadState

class MissionStatsController(Node):
    def __init__(self):
        super().__init__('mission_stats_controller')
        self.get_logger().info('Mission state controller started!')
        
        lol = PayloadState()
        
def main(args=None):
    rclpy.init(args=args)
    node = MissionStatsController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()