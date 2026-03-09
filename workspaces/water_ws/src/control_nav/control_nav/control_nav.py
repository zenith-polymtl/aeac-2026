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
from rclpy.qos import QoSProfile

from custom_interfaces.srv import ConvertGpsToLocal

import os

qos_profile_BE = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=8
)

class ControlNav(Node):

    def __init__(self):
        super().__init__('control_nav')
        
        self.initialize_parameters()
        
        self.is_moving_to_position = False
        self.current_point_objectif = Point()
        
        # Publisher
        self.publisher_raw = self.create_publisher(PositionTarget, '/mavros/setpoint_raw/local', 10)
        self.move_to_scene_pub = self.create_publisher(Bool, '/mission/control_nav/move_to_scene/finished', 10)
        
        # Subscribers
        self.move_to_scene_sub = self.create_subscription(Bool, '/mission/control_nav/move_to_scene', self.move_to_scene_procedure, 10)
        
        # Genretal controle subscriber
        self.abort_all_sub = self.create_subscription(Bool, '/mission/abort_all', self.stop_drone, 10)

        self.drone_position_sub = self.create_subscription(
            PoseStamped, "/mavros/local_position/pose", self.drone_pose_callback, qos_profile_BE
        )
                        
        self.convert_client = self.create_client(ConvertGpsToLocal, '/convert/gps_to_local')
        while not self.convert_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Convert service not available, waiting again...')
                        
        self.position_check_timer = self.create_timer(self.delais_for_position_check, self.position_check_timer_callback)
        
        self.get_logger().info('Control Nav node launched')
        
    
    def initialize_parameters(self):
        ## Param decalration        
        self.declare_parameter('delais_for_position_check', 0.5)
        self.declare_parameter('distance_from_objectif_threashold', 3.0)

        self.declare_parameter('latitude_of_scene', -35.361450)
        self.declare_parameter('longitude_of_scene', 149.161448)
        self.declare_parameter('altitude_of_scene', 10.0)
        
        self.delais_for_position_check = self.get_parameter('delais_for_position_check').get_parameter_value().double_value
        self.distance_from_objectif_threashold = self.get_parameter('distance_from_objectif_threashold').get_parameter_value().double_value
        
        self.latitude_of_scene = self.get_parameter('latitude_of_scene').get_parameter_value().double_value
        self.longitude_of_scene = self.get_parameter('longitude_of_scene').get_parameter_value().double_value
        self.altitude_of_scene = self.get_parameter('altitude_of_scene').get_parameter_value().double_value
        
    
    def move_to_pose(self, wp):
        self.current_point_objectif = wp
        
        target = PositionTarget()  
        target.header.stamp = self.get_clock().now().to_msg()  
        target.header.frame_id = "map"  
        target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        
        target.type_mask = (
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_VX |
            PositionTarget.IGNORE_VY |
            PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_YAW |
            PositionTarget.IGNORE_YAW_RATE
        )

        target.position = self.current_point_objectif

        self.publisher_raw.publish(target)
    
    def move_to_scene_procedure(self, _):
        scene_conversion_request = ConvertGpsToLocal.Request()
        scene_conversion_request.gps_point = NavSatFix()
        scene_conversion_request.gps_point.latitude = self.latitude_of_scene
        scene_conversion_request.gps_point.longitude = self.longitude_of_scene
        scene_conversion_request.gps_point.altitude = self.altitude_of_scene
        future = self.convert_client.call_async(scene_conversion_request)
        self.get_logger().info(f"Move to Scene procedure started")
        future.add_done_callback(self.object_conversion_callback)
        
    def object_conversion_callback(self, future):
        if future.result() is not None:
            local_pose = future.result().local_point
            self.object_objectif_waypoint = local_pose.pose.position
            self.get_logger().info(f"Converted gps pose to local: x={local_pose.pose.position.x}, y={local_pose.pose.position.y}, z={local_pose.pose.position.z}")
        else:
            self.get_logger().error('Conversion failed')
            return
            
        distance = self.calculate_distance_from_point(self.object_objectif_waypoint)
        if (distance <= 30):
            self.get_logger().info(f"Already at a distance of {distance}m, which is smaller thant 30m. Not moving")
        else:
            dx = self.drone_pose.x - self.object_objectif_waypoint.x
            dy = self.drone_pose.y - self.object_objectif_waypoint.y
            horiz_dist = math.sqrt(dx**2 + dy**2)
            if horiz_dist > 0:
                ux = dx / horiz_dist
                uy = dy / horiz_dist
                target_pos = Point()
                target_pos.x = self.object_objectif_waypoint.x + 30 * ux
                target_pos.y = self.object_objectif_waypoint.y + 30 * uy
                target_pos.z = self.drone_pose.z  # Keep current altitude
                self.is_moving_to_position = True
                self.current_point_objectif = target_pos
                self.get_logger().info(f"Moving to target position: x={target_pos.x}, y={target_pos.y}, z={target_pos.z}")
                self.move_to_pose(self.current_point_objectif)
            else:
                self.get_logger().info("Drone is directly above/below the target; no horizontal movement needed.")
        
    def drone_pose_callback(self, msg):
        self.drone_pose = msg.pose.position
        
    def calculate_distance_from_point(self, point_1):
        return math.sqrt(
            (point_1.x - self.drone_pose.x) ** 2 +
            (point_1.y - self.drone_pose.y) ** 2 +
            (point_1.z - self.drone_pose.z) ** 2
        )

    def position_check_timer_callback(self):
        if not self.is_moving_to_position or not self.current_point_objectif:
            return
        
        distance_form_objectif = self.calculate_distance_from_point(self.current_point_objectif)
        
        self.get_logger().info(f"Current distance : {distance_form_objectif}")

        if distance_form_objectif < self.distance_from_objectif_threashold:
            self.get_logger().info("Objectif reached")
            self.is_moving_to_position = False
            reached_site_msg = Bool()
            reached_site_msg.data = True
            self.move_to_scene_pub.publish(reached_site_msg)
    
    # Stop the drone in current place
    def stop_current_lap(self):
        stop_target = PositionTarget()
        stop_target.header.stamp = self.get_clock().now().to_msg()
        stop_target.header.frame_id = "map"
        
        stop_target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        
        stop_target.type_mask = (
            PositionTarget.IGNORE_PX |
            PositionTarget.IGNORE_PY |
            PositionTarget.IGNORE_PZ |
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW
        )
        
        stop_target.velocity.x = 0.0
        stop_target.velocity.y = 0.0
        stop_target.velocity.z = 0.0
                
        self.publisher_raw.publish(stop_target)
        self.get_logger().info(f"Drone Stopped")
    
    # Pass the to the nex point
    def stop_drone(self, _):
        self.stop_laps()
    
    # Stop the laps completely
    def stop_laps(self):
        self.get_logger().info(f"Stopping Drone Laps")
        self.is_moving_to_position = False

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