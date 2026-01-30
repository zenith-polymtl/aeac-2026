import rclpy
from rclpy.node import Node
from custom_interfaces.msg import PayloadState
from std_msgs.msg import Bool

class MissionStatsController(Node):
    def __init__(self):
        super().__init__('mission_stats_controller')
        self.get_logger().info('Mission state controller started!')
        
        self.subscription = self.create_subscription(
            PayloadState,
            '/mission/set_state',
            self.state_callback,
            10
        )

        self.start_lap_sub = self.create_subscription(Bool, '/mission/control_nav/lap/start', self.starting_laps, 10)
        self.finish_lap_sub = self.create_subscription(Bool, '/mission/control_nav/lap/finish', self.finishing_laps, 10)
        self.lap_finished_sub = self.create_subscription(Bool, '/mission/control_nav/lap/finished', self.laps_finished_callback, 10)
        self.move_to_scenen_finsihed_sub = self.create_subscription(Bool, '/mission/control_nav/move_to_scene/finished', self.move_to_site_finished_callback, 10)
        self.move_to_scene_publisher = self.create_subscription(Bool, "/mission/control_nav/move_to_scene", self.move_to_site_finished_callback, 10)

        self.current_state = PayloadState.INITAL_STATE

    def state_callback(self, msg):
        state = msg.mode
        if state != self.current_state:
            self.handle_state_change(state)
    
    def handle_state_change(self, state):
        self.current_state = state
        if state == PayloadState.INITAL_STATE:
            self.initial_state_handler()
        elif state == PayloadState.TAKEOFF:
            self.takeoff_handler()
        elif state == PayloadState.LAP_STARTED:
            self.lap_started()
        elif state == PayloadState.STOPING_LAPS:
            self.stopping_laps_handler()
        elif state == PayloadState.GOING_TO_SITE:
            self.going_to_site_handler()
        else:
            self.get_logger().warn(f'Unknown state: {state}')
    
    def starting_laps(self, _):
        if self.current_state != PayloadState.LAP_STARTED:
            self.handle_state_change(PayloadState.LAP_STARTED)
            
    def finishing_laps(self, _):
        if self.current_state != PayloadState.STOPING_LAPS:
            self.handle_state_change(PayloadState.STOPING_LAPS)
    
    def laps_finished_callback(self, _):
        self.get_logger().info('Received notification: Laps are finished')
        if self.current_state != PayloadState.GOING_TO_SITE:
            self.handle_state_change(PayloadState.GOING_TO_SITE)
    
    def move_to_site_finished_callback(self, _):
        self.get_logger().info('Received notification: Laps are finished')
        if self.current_state != PayloadState.IS_ON_SITE:
            self.handle_state_change(PayloadState.IS_ON_SITE)
    
    
    def initial_state_handler(self):
        self.get_logger().info('Handling initial state')
        print('In initial state')
    
    def takeoff_handler(self):
        self.get_logger().info('Handling takeoff')
        print('In takeoff state')
    
    def lap_started(self):
        self.get_logger().info('Handling lap started')
        print('In lap started state')
    
    def stopping_laps_handler(self):
        self.get_logger().info('Handling stopping laps')
        print('In stopping laps state')
    
    def going_to_site_handler(self):
        self.get_logger().info('Handling going to site')
        move_to_site_message = Bool()
        move_to_site_message = Bool().data = True
        self.move_to_scene_publisher.publish(move_to_site_message)
        
        
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