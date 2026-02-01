import rclpy
from rclpy.node import Node
from custom_interfaces.msg import PayloadState
from std_msgs.msg import Bool

class MissionStatsController(Node):
    def __init__(self):
        super().__init__('mission_stats_controller')
        self.get_logger().info('Mission state controller started!')
        
        # Subscriber
        self.change_state_sub = self.create_subscription(PayloadState, '/mission/set_state', self.change_state_callback, 10)
        self.start_lap_sub = self.create_subscription(Bool, '/mission/control_nav/lap/start', self.starting_laps, 10)
        self.finish_lap_sub = self.create_subscription(Bool, '/mission/control_nav/lap/finish', self.finishing_laps, 10)
        self.lap_finished_sub = self.create_subscription(Bool, '/mission/control_nav/lap/finished', self.laps_finished_callback, 10)
        self.move_to_scenen_finsihed_sub = self.create_subscription(Bool, '/mission/control_nav/move_to_scene/finished', self.move_to_site_finished_callback, 10)
        self.takeoff_completed_sub = self.create_subscription(Bool, '/mission/takeoff_completed', self.takeoff_completed_handler, 10)
        self.mission_go_sub = self.create_subscription(Bool, '/mission/go', self.go_mission_received, 10)
        
        # Publisher
        self.start_lap_publisher = self.create_publisher(Bool, "/mission/control_nav/lap/start", 10)
        self.move_to_scene_publisher = self.create_publisher(Bool, "/mission/control_nav/move_to_scene", 10)
        self.state_changed_publisher = self.create_publisher(PayloadState, '/mission/state_changed', 10)
        
        self.current_state = PayloadState.INITAL_STATE

        
    def change_state_callback(self, msg):
        state = msg.mode
        if state != self.current_state:
            self.handle_state_change(state)
    
    def handle_state_change(self, state):
        self.current_state = state
        if state == PayloadState.INITAL_STATE:
            self.undefined_state_handler("inital")
        elif state == PayloadState.TAKEOFF:
            self.undefined_state_handler("takeoff")
        elif state == PayloadState.TAKEOFF_COMPLETED:
            self.start_lag_handler()
        elif state == PayloadState.LAP_STARTED:
            self.undefined_state_handler("lap stared")
        elif state == PayloadState.STOPING_LAPS:
            self.undefined_state_handler("stop laps")
        elif state == PayloadState.GOING_TO_SITE:
            self.going_to_site_handler()
        elif state == PayloadState.IS_ON_SITE:
            self.undefined_state_handler("on site")
        else:
            self.get_logger().warn(f'Unknown state: {state}')
            
        change_state_msg = PayloadState()
        change_state_msg.mode = state
        self.state_changed_publisher.publish(change_state_msg)
        
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
        self.get_logger().info('Received notification: Move to site finished')
        if self.current_state != PayloadState.IS_ON_SITE:
            self.handle_state_change(PayloadState.IS_ON_SITE)
            
    def go_mission_received(self, _):
        self.get_logger().info('Received notification: Go mission!')
        if self.current_state != PayloadState.TAKEOFF:
            self.handle_state_change(PayloadState.TAKEOFF)
    
    def takeoff_completed_handler(self, msg):
        is_completed = msg.data
        self.get_logger().info(f'Received notification: TakeOff is {"completed" if is_completed else "aborted"}')
        if is_completed and self.current_state != PayloadState.TAKEOFF_COMPLETED:
            self.handle_state_change(PayloadState.TAKEOFF_COMPLETED)
        elif self.current_state == PayloadState.TAKEOFF:
            self.handle_state_change(PayloadState.INITAL_STATE)
            
    
    def undefined_state_handler(self, state):
        self.get_logger().info(f'Entered {state} state, but no handeling')
        
    def start_lag_handler(self):
        self.get_logger().info('Handling start lap')
        start_lap_msg = Bool()
        start_lap_msg.data = True
        self.start_lap_publisher.publish(start_lap_msg)
        
    def going_to_site_handler(self):
        self.get_logger().info('Handling going to site')
        move_to_site_message = Bool()
        move_to_site_message.data = True
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