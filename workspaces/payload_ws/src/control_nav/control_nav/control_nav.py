import rclpy
from rclpy.node import Node
import json
import math

from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import PositionTarget
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Bool
from mavros import setpoint as SP  
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from custom_interfaces.srv import ConvertGpsToLocal

import os

NUMBER_OF_LAP = 3
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
                self.waypoints_gps = json.load(f)
            self.get_logger().info(f'Loaded {len(self.waypoints_gps)} waypoints from {json_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to load JSON file: {e}')
            raise
                
        self.moving_to_position = False
        self.current_point_objectif = Point()
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # self.position_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', qos_profile)
        self.publisher_raw = self.create_publisher(PositionTarget, '/mavros/setpoint_raw/local', 10)
        
        self.start_lap_sub = self.create_subscription(Bool, '/mission/lap', self.start_laps, 10)

        self.drone_position_sub = self.create_subscription(
            PoseStamped, "/mavros/local_position/pose", self.drone_pose_callback, qos_profile_BE
        )
        
        self.convert_client = self.create_client(ConvertGpsToLocal, '/convert/gps_to_local')
        while not self.convert_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        self.waypoints_raw = []
        
        for i, waypoint in enumerate(self.waypoints_gps):
            request = ConvertGpsToLocal.Request()
            request.gps_point = NavSatFix()
            request.gps_point.latitude = waypoint['latitude']
            request.gps_point.longitude = waypoint['longitude']
            request.gps_point.altitude = waypoint['altitude']

            future = self.convert_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                local_pose = future.result().local_point
                self.get_logger().info(f"GPS position: latitude={waypoint['latitude']}, longitude={waypoint['longitude']}, altitude={waypoint['altitude']}")
                self.get_logger().info(f"Local Pose: position x={local_pose.pose.position.x}, y={local_pose.pose.position.y}, z={local_pose.pose.position.z}")
                self.waypoints_raw.append(local_pose)
            else:
                # TODO What should we do if the service fails
                self.get_logger().error(f'Failed to convert the gps point ({i}) to local pos')

        self.position_check_timer = self.create_timer(DELAIS_FOR_POSITION_CHECK, self.position_check_timer_callback)

    def move_to_pose(self, wp):
        # self.current_point_objectif = Point()
        # self.current_point_objectif = wp.pose.position
        
        # self.target_pose = PoseStamped()
        # self.target_pose.header.frame_id = 'map'
        # self.target_pose.pose.position = self.current_point_objectif
        # self.target_pose.pose.orientation.w = 1.0

        # self.get_logger().info(f"Currently moving to : x={wp.pose.position.x}, y={wp.pose.position.y}, z={wp.pose.position.z}")
        
        # self.target_pose.header.stamp = self.get_clock().now().to_msg()
        # self.position_pub.publish(self.target_pose)
        
        self.current_point_objectif = wp.pose.position
        
        target = PositionTarget()  
        target.header.stamp = self.get_clock().now().to_msg()  
        target.header.frame_id = "map"  
        target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        
        target.type_mask = (
            PositionTarget.IGNORE_AX |
            PositionTarget.IGNORE_AY |
            PositionTarget.IGNORE_AZ |
            PositionTarget.IGNORE_VX |
            PositionTarget.IGNORE_VY |
            PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_YAW |
            PositionTarget.IGNIRE_YAW_RATE
        )

        target.postion = self.current_point_objectif

        self.publisher_raw.publish(target)


    def start_laps(self, _):
        self.get_logger().info("Starting the laps")
        self.current_lap = 0
        self.moving_to_position = True
        self.waypoint_index = 0
        self.move_to_pose(self.waypoints_raw[self.waypoint_index])
        
    def drone_pose_callback(self, msg):
        self.drone_pose = msg.pose.position

    
    def handle_reach_waypoint(self):
        self.waypoint_index += 1
        if len(self.waypoints_raw) == self.waypoint_index:
            self.waypoint_index = 0
            self.current_lap += 1
            if self.current_lap == NUMBER_OF_LAP:
                self.moving_to_position = False
                self.current_lap = 0
                self.get_logger().info("Finised all laps")
                return
                
        self.move_to_pose(self.waypoints_raw[self.waypoint_index])

    def position_check_timer_callback(self):
        if not self.moving_to_position or not self.current_point_objectif:
            return
        
        # self.get_logger().info(f"Current Position : x={self.drone_pose.x}, y={self.drone_pose.y}, z={self.drone_pose.z}")

        distance_form_objectif = math.sqrt(
            (self.current_point_objectif.x - self.drone_pose.x) ** 2 +
            (self.current_point_objectif.y - self.drone_pose.y) ** 2 +
            (self.current_point_objectif.z - self.drone_pose.z) ** 2
        )
        
        self.get_logger().info(f"Current distance : {distance_form_objectif}")
        
        if self.current_lap == 1 and distance_form_objectif < 100:
            self.stop_laps()

        if distance_form_objectif < DISTANCE_FROM_OBJECTIF_THREASHOLD:
            self.get_logger().info("Objectif reached")
            self.handle_reach_waypoint()
    
    def stop_current_lap(self):
        self.current_point_objectif = Point()
        
        self.target_pose = PoseStamped()
        self.target_pose.header.frame_id = 'map'
        self.target_pose.pose.position = self.drone_pose
        self.target_pose.pose.orientation.w = 1.0
        
        self.target_pose.header.stamp = self.get_clock().now().to_msg()
        # self.position_pub.publish(self.target_pose)
    
    def skip_current_lap(self):
        self.stop_current_lap()
        self.handle_reach_waypoint()
    
    def stop_laps(self):
        self.moving_to_position = False
        self.current_lap = 0
        self.waypoint_index = 0
        
        self.stop_current_lap()

def main(args=None):
    rclpy.init(args=args)
    node = ControlNav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_laps()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()