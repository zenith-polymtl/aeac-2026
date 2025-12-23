import rclpy
from rclpy.node import Node
import json
import math

from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import PositionTarget
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import PoseStamped, Point



import os

NUMBER_OF_LAP = 1
DELAIS_FOR_POSITION_CHECK = 0.5
DISTANCE_FROM_OBJECTIF_THREASHOLD = 2

qos_profile_BE = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=8
)

class ControlNav(Node):

    def __init__(self):
        super().__init__('control_nav')

        self.declare_parameter('json_filename', 'waypoints.json')
        self.declare_parameter('json_subfolder', 'data')

        json_filename = self.get_parameter('json_filename').get_parameter_value().string_value
        json_subfolder = self.get_parameter('json_subfolder').get_parameter_value().string_value

        try:
            package_share_dir = get_package_share_directory('control_nav')
            json_path = os.path.join(package_share_dir, json_subfolder, json_filename)

            if not os.path.exists(json_path):
                self.get_logger().error(f'JSON file not found: {json_path}')
                raise FileNotFoundError(json_path)

            with open(json_path, 'r') as f:
                self.waypoints = json.load(f)

            self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints from {json_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to load JSON file: {e}')
            raise
        
        self.moving_to_position = False

        # self.publisher = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.publisher_raw = self.create_publisher(PositionTarget, '/mavros/setpoint_raw/local', 10)

        self.drone_position_sub = self.create_subscription(
            PoseStamped, "/mavros/local_position/pose", self.drone_pose_callback, qos_profile_BE
        )

        self.position_check_timer = self.create_timer(DELAIS_FOR_POSITION_CHECK, self.position_check_timer_callback)


    
    def read_waypoints(self):
        for wp in self.waypoints:
            print(wp)
            # msg = NavSatFix()
            # msg.header.stamp = self.get_clock().now().to_msg()
            # msg.header.frame_id = 'gps'

            # # msg.status.status = NavSatStatus.STATUS_FIX
            # # msg.status.service = NavSatStatus.SERVICE_GPS

            # msg.latitude = float(wp.get('latitude', 0.0))
            # msg.longitude = float(wp.get('longitude', 0.0))
            # msg.altitude = float(wp.get('altitude', 0.0))

            # msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

            # self.get_logger().info(
            #     f'[{self.index + 1}/{len(self.waypoints)}] '
            #     f'lat={msg.latitude:.7f}, lon={msg.longitude:.7f}, alt={msg.altitude:.2f}'
            # )
    
    def move_to_raw_pose(self, wp):
        target = PositionTarget()  
        target.header.stamp = self.get_clock().now().to_msg()  
        target.header.frame_id = "map"  
        target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        
        target.type_mask = (
            PositionTarget.IGNORE_PX |
            PositionTarget.IGNORE_PY |
            PositionTarget.IGNORE_PZ |
            PositionTarget.IGNORE_VX |
            PositionTarget.IGNORE_VY |
            PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_YAW
        )

        self.current_point_objectif = Point()

        self.current_point_objectif.x = wp.get('latitude', 0.0)
        self.current_point_objectif.y = wp.get('longitude', 0.0)
        self.current_point_objectif.z = wp.get('altitude', 0.0)

        target.postion = self.current_point_objectif

        self.publisher_raw.publish(target)


    def start_laps(self):
        self.current_lap = 0
        self.moving_to_position = True
        self.waypoint_index = 0
        self.move_to_raw_pose(self.waypoints[self.waypoint_index])
        
    def drone_pose_callback(self, msg):
        self.drone_pose = msg.pose.position
    
    def handle_reach_waypoint(self):
        self.waypoint_index += 1
        if self.waypoints.len() == self.waypoint_index:
            self.waypoint_index += 0
            self.current_lap += 1
            if self.current_lap == NUMBER_OF_LAP:
                self.moving_to_position = False
                self.current_lap = 0
                return
        
        self.move_to_raw_pose(self.waypoints[self.waypoint_index])


    def position_check_timer_callback(self):
        if not self.moving_to_position:
            return

        distance_form_objectif = math.sqrt(
            (self.current_point_objectif.x - self.drone_pose.x) ** 2 +
            (self.current_point_objectif.y - self.drone_pose.y) ** 2 +
            (self.current_point_objectif.z - self.drone_pose.z) ** 2
        )

        if distance_form_objectif < DISTANCE_FROM_OBJECTIF_THREASHOLD:
            self.handle_reach_waypoint()
        

def main(args=None):
    rclpy.init(args=args)
    node = ControlNav()
    node.read_waypoints()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()