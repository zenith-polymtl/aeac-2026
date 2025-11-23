#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped

from zenmav import Zenmav     
from builtin_interfaces.msg import Time


class MissionInit(Node):
    def __init__(self):
        super().__init__('mission_init')

        #ici y a les publishers (etat, takeoff, rtl)
        self.status_pub = self.create_publisher(String, '/mission/status', 10)   #noms des topics hardcoded mais je vais changer ca
        self.takeoff_pub = self.create_publisher(PoseStamped, '/drone/takeoff_cmd', 10)
        self.waypoint_pub = self.create_publisher(PoseStamped, '/mission/waypoint', 10)
        self.rtl_pub = self.create_publisher(Bool, '/drone/rtl', 10)

        #ici y a les subsciptions (le go, abort, internal, external) 
        self.ready_sub = self.create_subscription(Bool, '/drone/readyness_state', self.cb_ready, 10)
        self.internal_sub = self.create_subscription(Bool, '/drone/internal_ok', self.cb_internal, 10)
        self.external_sub = self.create_subscription(Bool, '/drone/external_ok', self.cb_external, 10)
        self.go_sub = self.create_subscription(Bool, '/mission/go', self.cb_go, 10)
        self.abort_sub = self.create_subscription(Bool, '/mission/abort', self.cb_abort, 10)

        self.ready = False
        self.internal_ok = False
        self.external_ok = False
        self.go = False

        self.declare_parameter("zenmav_ip", "tcp:127.0.0.1:5762")
        zenmav_ip = self.get_parameter("zenmav_ip").value

        self.declare_parameter("takeoff_alt", 10)
        self.takeoff_alt = self.get_parameter("takeoff_alt").value

        # parametres x, y, z coord du point avec des valeurs par defaut (5, 5, 10)
        self.declare_parameter("waypoint1_x", 5)
        self.declare_parameter("waypoint1_y", 5)
        self.declare_parameter("waypoint1_z", self.takeoff_alt)

        # recupere les coord
        self.waypoint1_x = self.get_parameter("waypoint1_x").value
        self.waypoint1_y = self.get_parameter("waypoint1_y").value
        self.waypoint1_z = self.get_parameter("waypoint1_z").value


        self.drone = Zenmav(zenmav_ip, gps_thresh=0.2)

        self.get_logger().info("init node ready.")

    def cb_ready(self, msg):
        self.ready = msg.data
        self.check_state()

    def cb_internal(self, msg):
        self.internal_ok = msg.data
        self.check_state()

    def cb_external(self, msg):
        self.external_ok = msg.data
        self.check_state()

    def cb_go(self, msg):
        self.go = msg.data
        if self.go:
            self.start_mission()

    def cb_abort(self, msg):
        if msg.data:
            self.global_abort()


    def check_state(self):
        if self.ready and self.internal_ok and self.external_ok:
            self.status_pub.publish(String(data="READY"))
            self.get_logger().info("System READY. Waiting for GO...")
        else:
            self.status_pub.publish(String(data="NOT_READY"))

    def start_mission(self):
        if not (self.ready and self.internal_ok and self.external_ok):
            self.get_logger().warn("GO received but system not ready.")
            return 

        self.get_logger().info("GO received : starting mission.")

        
        self.drone.set_mode("GUIDED")
        self.drone.arm()

        self.drone.takeoff(self.takeoff_alt)

        takeoff_msg = PoseStamped()
        takeoff_msg.pose.position.z = self.takeoff_alt
        self.takeoff_pub.publish(takeoff_msg)

        self.status_pub.publish(String(data="TAKEOFF"))

        #waypoint 1
        p1 = PoseStamped()
        p1.pose.position.x = self.waypoint1_x
        p1.pose.position.y = self.waypoint1_y
        p1.pose.position.z = self.waypoint1_z
        self.waypoint_pub.publish(p1)

        #va vers le premier point
        self.drone.local_target((self.waypoint1_x, self.waypoint1_y, -self.waypoint1_z))

        self.status_pub.publish(String(data="WAYPOINT_1_SENT"))
        self.get_logger().info("Waypoint #1 sent.")

    def global_abort(self):
        self.status_pub.publish(String(data="ABORT"))
        self.rtl_pub.publish(Bool(data=True))
        self.drone.set_mode("RTL")

        self.get_logger().warn("GLOBAL ABORT RTL engaged.")




def main(args=None):
    rclpy.init(args=args)
    node = MissionInit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()