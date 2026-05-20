#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from custom_interfaces.msg import TargetPosePolar, UiMessage
from std_msgs.msg import Bool, String, UInt8, Empty
from geometry_msgs.msg import PoseStamped, Point, Pose
from zed_msgs.msg import ObjectsStamped
from mavros_msgs.msg import State
import numpy as np
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose

class GimbalMode:
    LOCK = 0
    FOLLOW = 1
    AUTO_AIM = 2
    
class ApproachState:
    IDLE            = "IDLE"
    DETECTING       = "DETECTING"
    APPROACHING     = "APPROACHING"
    IN_POSITION     = "IN_POSITION"
    AIMING          = "AIMING"
    SHOOTING        = "SHOOTING"
    TALKING_PICTURE = "TALKING_PICTURE"
    # ABORTED         = "ABORTED"

class AutonomousApproach(Node):
    def __init__(self):
        super().__init__('autonomous_approach')

        self.set_up_parameters()
        self.set_up_topics()
        self.define_initial_state()

        
        self.get_logger().info(f"Autonomous Approach node launched")

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    
    @staticmethod
    def _create_qos_profile(reliability_policy):
        return QoSProfile(
            reliability=reliability_policy,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
    
    def _transition(self, new_state: str):
        """Central state-transition method — every change goes through here."""
        old = self._state
        if old == new_state:
            self.get_logger().debug(f"[STATE] Already in {new_state}, no transition needed.")
            return
        self._state = new_state
        
        msg = String()
        msg.data = new_state
        self.state_pub.publish(msg)
        
    def _assert_state(self, expected, context: str) -> bool:
        """
        Returns True and logs a warning when the current state does NOT
        match *expected*.  Use as an early-exit guard in callbacks.
        """
        # Skip state check
        if self.ignore_state_check:
            return
        
        if isinstance(expected, (list, tuple)):
            if self._state not in expected:
                self.get_logger().warn(
                    f"[GUARD] {context}: expected state in {expected}, "
                    f"currently {self._state}. Ignoring."
                )
                return False
        else:
            if self._state != expected:
                self.get_logger().warn(
                    f"[GUARD] {context}: expected {expected}, "
                    f"currently {self._state}. Ignoring."
                )
                return False
        return True

    # ------------------------------------------------------------------
    # State initialisation
    # ------------------------------------------------------------------
    
    def define_initial_state(self):
        self._state          = ApproachState.IDLE
        
        msg = String()
        msg.data = ApproachState.IDLE
        self.state_pub.publish(msg)
                
        self.in_position          = False
        self.target_aimed         = False
        
        self.mavros_fight_state = State() 
        
        self.continues_target_in_sight = 0
        
        self.get_logger().info("[STATE] State machine reset to IDLE")

    # ------------------------------------------------------------------
    # Parameters
    # ------------------------------------------------------------------
    def set_up_parameters(self):
        self.declare_parameter('approach_radius', 2.0)
        self.declare_parameter('approach_height', 1.0)
        self.declare_parameter('sim', False)
        self.declare_parameter('ignore_state_check', False)
        self.declare_parameter('continues_in_sight_threshold', 5)
        self.declare_parameter('polar_activation_delais', 0.5)
        self.declare_parameter('shoot_picture_delais', 1.0)
        self.declare_parameter('polar_distace', 3.0)
        self.declare_parameter('polar_target_topic', "/polar/goal_pose")

        
        self.approach_radius    = self.get_parameter('approach_radius').value
        self.approach_height    = self.get_parameter('approach_height').value
        self.sim                = self.get_parameter('sim').value
        self.ignore_state_check = self.get_parameter('ignore_state_check').value
        self.continues_in_sight_threshold = self.get_parameter('continues_in_sight_threshold').value
        self.shoot_picture_delais = self.get_parameter('shoot_picture_delais').value
        self.polar_activation_delais = self.get_parameter('polar_activation_delais').value
        self.polar_distace = self.get_parameter('polar_distace').value
        self.polar_target_topic = self.get_parameter('polar_target_topic').value

    
    # ------------------------------------------------------------------
    # Topics
    # ------------------------------------------------------------------ 
    def set_up_topics(self):
        qos_reliable = self._create_qos_profile(QoSReliabilityPolicy.RELIABLE)
 
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_callback, qos_reliable)
 
        self.message_to_ui_pub = self.create_publisher(
            UiMessage, 'aeac/external/send_to_ui', qos_reliable)
        
        self.state_pub = self.create_publisher(
            String, '/aeac/external/mission/state', qos_reliable
        )
 
        self.abort_publisher = self.create_subscription(
            Bool, '/aeac/external/mission/abort_all',
            self.abort_callback, qos_reliable)
 
        # Autonomous Approach
        self.auto_approach_sub = self.create_subscription(
            Bool, '/aeac/external/auto_approach/start',
            self.auto_approach_activation_callback, qos_reliable)
 
        self.in_position_to_shoot_sub = self.create_subscription(
            Bool, '/aeac/internal/auto_approach/in_position',
            self.in_position_callback, qos_reliable)
 
        # self.target_position_sub = self.create_subscription(
        #     ObjectsStamped, '/aeac/internal/auto_approach/target_detected',
        #     self.target_position_received_callback, qos_reliable)
 
        self.trigger_target_detection_pub = self.create_publisher(
            Empty, '/aeac/internal/auto_approach/detect_target', qos_reliable)
 
        self.activate_polar_pub = self.create_publisher(
            String, '/aeac/internal/auto_approach/activate_polar', qos_reliable)
 
        self.polar_target_pub = self.create_publisher(
            TargetPosePolar, self.polar_target_topic, qos_reliable)
 
        self.polar_center_location_pub = self.create_publisher(
            PoseStamped, '/aeac/internal/auto_shoot/center_location', qos_reliable)
 
        # Autonomous Shoot
        self.start_target_detection_pub = self.create_publisher(
            Bool, '/aeac/internal/auto_shoot/start_hr_aiming', qos_reliable)
 
        self.start_auto_shoot_pub = self.create_publisher(
            UInt8, '/aeac/external/gimbal/set_mode', qos_reliable)
 
        self.target_in_aim_sub = self.create_subscription(
            Bool, '/aeac/internal/auto_shoot/target_in_aim',
            self.target_aimed_callback, qos_reliable)
 
        self.auto_shoot_sub = self.create_subscription(
            Bool, '/aeac/external/auto_shoot/start',
            self.auto_shoot_callback, qos_reliable)
 
        # Shoot procedure
        self.shoot_sub = self.create_subscription(
            Bool, '/aeac/external/shoot',
            self.shoot_target_callback, qos_reliable)
 
        self.take_picture_sub = self.create_subscription(
            Bool, '/aeac/external/take_picture',
            self.take_picture_callback, qos_reliable)
        
        self.finished_shoot_sub = self.create_subscription(
            Bool, '/aeac/internal/shot_finished',
            self.finished_shoot_callback, qos_reliable)
 
        self.shoot_pub = self.create_publisher(
            Bool, '/aeac/internal/shoot', qos_reliable)
         
        self.take_picture_pub = self.create_publisher(
            Empty, '/aeac/internal/take_picture', qos_reliable)
      
    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------  
    def state_callback(self, msg):
        self.mavros_fight_state = msg
        
        if not self.mavros_fight_state.mode == "GUIDED" and self._state in [ApproachState.DETECTING, ApproachState.APPROACHING]:
            self._stop_polar()
            self.define_initial_state()
            
    def auto_approach_activation_callback(self, msg: Bool):
        """Activates or deactivates the autonomous approach.""" 
        if msg.data:
            if self._state != ApproachState.IDLE:
                self.get_logger().warn(
                    f"[GUARD] auto_approach/start: already running (state={self._state}). Ignoring.")
                return
            
            if not self.mavros_fight_state.mode == "GUIDED":
                self.get_logger().info("Auto approached trigger but not in guided, skipping")
                return
 
            self._transition(ApproachState.DETECTING)
            
            self.activate_polar_callback()
 
        else:
            if self._state == ApproachState.IDLE:
                self.get_logger().debug("[GUARD] Deactivation received but already IDLE. Ignoring.")
                return
            self.get_logger().info("[APPROACH] Deactivation received — resetting state machine")
            self._stop_polar()
            self.define_initial_state()

    # def target_position_received_callback(self, msg: ObjectsStamped):
    #     self.get_logger().info(f"[DETECT] Received ObjectsStamped with {len(msg.objects)} object(s)")
        
    #     if not self._assert_state(ApproachState.DETECTING, "target_position_received_callback"):
    #         return
        
    #     if len(msg.objects) == 0:
    #         self.get_logger().warn("[DETECT] No objects in message.")
    #         self.send_message_to_ui("No object detected", is_success=False)
    #         return
        
    #     best_confidence    = 0.0
    #     best_found_object  = None
    #     for obj in msg.objects:
    #         if obj.confidence > best_confidence:
    #             best_confidence   = obj.confidence
    #             best_found_object = obj
 
    #     self.get_logger().info(
    #         f"[DETECT] Best object: label={best_found_object.label}  "
    #         f"confidence={best_confidence*100:.1f}%  pos={list(best_found_object.position)}"
    #     )
    #     self.send_message_to_ui(
    #         f"Object {best_found_object.label} detected with "
    #         f"{best_confidence*100:.1f}% confidence at {list(best_found_object.position)}"
    #     )
 
    #     CONFIDENCE_THRESHOLD = 0.5
    #     if best_confidence < CONFIDENCE_THRESHOLD:
    #         self.get_logger().warn(
    #             f"[DETECT] Confidence {best_confidence:.2f} below threshold "
    #             f"{CONFIDENCE_THRESHOLD}. Aborting approach."
    #         )
    #         self.send_message_to_ui(
    #             f"Object detected with too low confidence ({best_confidence*100:.1f}%). "
    #             f"Returning to idle.", is_success=False
    #         )
    #         self.define_initial_state()
    #         return
 
    #     self.get_logger().info("[DETECT] Target accepted — proceeding to approach")
    #     self.send_polar_target(best_found_object)
        
    # def send_polar_target(self, best_found_object):
    #     """
    #     Transforms the detected target into the map frame and publishes the
    #     centre location, then kicks off the polar approach.
    #     """
    #     if not self._assert_state(ApproachState.DETECTING, "send_polar_target"):
    #         return
        
    #     try:
    #         pose_stamped                 = PoseStamped()
    #         pose_stamped.header.stamp    = self.get_clock().now().to_msg()
    #         pose_stamped.header.frame_id = "zed_camera_link"
 
    #         target_position = Pose()
    #         if not self.sim:
    #             target_position.position.x = float(best_found_object.position[2])
    #             target_position.position.y = float(best_found_object.position[1])
    #             target_position.position.z = float(best_found_object.position[0])
    #             self.get_logger().info(
    #                 f"[TF] Raw camera-frame position: "
    #                 f"x={target_position.position.x:.3f}  "
    #                 f"y={target_position.position.y:.3f}  "
    #                 f"z={target_position.position.z:.3f}"
    #             )
    #         else:
    #             target_position.position.x = 5.0
    #             target_position.position.y = 0.0
    #             target_position.position.z = 0.0
    #             self.get_logger().info("[TF] Sim mode — using hardcoded position (5, 0, 0)")
 
    #         transform = self.tf_buffer.lookup_transform(
    #             'map', 
    #             # 'base_link',
    #             'zed_camera_link', 
    #             rclpy.time.Time())
 
    #         pose_stamped.pose = do_transform_pose(target_position, transform)
    #         self.get_logger().info(
    #             f"[TF] Map-frame position: "
    #             f"x={pose_stamped.pose.position.x:.3f}  "
    #             f"y={pose_stamped.pose.position.y:.3f}  "
    #             f"z={pose_stamped.pose.position.z:.3f}"
    #         )
 
    #         self.polar_center_location_pub.publish(pose_stamped)
    #         self.get_logger().info("[APPROACH] Published polar centre location")
 
    #         self.activate_polar_callback()
 
    #     except Exception as e:
    #         self.get_logger().error(f"[TF] Transform failed: {e}")
    #         self.send_message_to_ui("TF transform failed — approach aborted.", is_success=False)
    #         self.define_initial_state()
            
    def activate_polar_callback(self):   
        if not self._assert_state(ApproachState.DETECTING, "activate_polar_callback"):
            return     
        
        start_msg      = String()
        start_msg.data = "start"
        self.activate_polar_pub.publish(start_msg)
        self.get_logger().info("[APPROACH] Published 'start' to polar controller")
        
        self.send_movement_command_timer = self.create_timer(self.polar_activation_delais, self.send_polar_target_pose)  

    def send_polar_target_pose(self):
        
        self.send_movement_command_timer.destroy()

        if not self._assert_state(ApproachState.DETECTING, "send_polar_target_pose"):
            return
        
        request         = TargetPosePolar()
        request.relative = False
        request.z        = np.nan
        request.theta    = np.nan
        request.r        = self.polar_distace
        self.polar_target_pub.publish(request)
 
        self.get_logger().info(
            f"[APPROACH] Polar target published: r={request.r}  theta={request.theta}  z={request.z}  relative={request.relative}"
        )
        self.send_message_to_ui(f"Starting auto approach — target distance: {request.r} m")
        self._transition(ApproachState.APPROACHING)        
    
    def in_position_callback(self, msg: Bool):
        self.get_logger().info(f"[TOPIC] in_position received: {msg.data}")
 
        if not self._assert_state(ApproachState.APPROACHING, "in_position_callback"):
            return
 
        if not msg.data:
            self.get_logger().debug("[APPROACH] in_position=False, still moving.")
            return
        
        self.in_position = True
        self._transition(ApproachState.IN_POSITION)
        self.send_message_to_ui("Target reached — ready to shoot")
 
        stop_msg      = String()
        stop_msg.data = "stop"
        self.activate_polar_pub.publish(stop_msg)
        self.get_logger().info("[APPROACH] Published 'stop' to polar controller")
        
        self.auto_shoot_callback(msg)
    
    def auto_shoot_callback(self, msg: Bool):
        self.get_logger().info(
            f"[TOPIC] auto_shoot/start received: {msg.data} "
        )
        
        if self._state == ApproachState.AIMING:
            self._transition(ApproachState.IDLE)
            self.stop_auto_aim()
        else:
            self.start_auto_aim()
        
    def start_auto_aim(self):
        self._transition(ApproachState.AIMING)
 
        det_msg      = Bool()
        det_msg.data = True
        self.start_target_detection_pub.publish(det_msg)
 
        gimbal_msg      = UInt8()
        gimbal_msg.data = GimbalMode.AUTO_AIM
        self.start_auto_shoot_pub.publish(gimbal_msg)
        self.get_logger().info(
            f"[SHOOT] Gimbal mode set to AUTO_AIM"
        )
    
    def stop_auto_aim(self):
        msg = Bool()
        msg.data = False
        self.start_target_detection_pub.publish(msg)
    
    def finished_shoot_callback(self, msg):
        self.get_logger().info(f"Received shoot finish message. Taking picture in {self.shoot_picture_delais} seconds")
        self.shoot_timer = self.create_timer(self.shoot_picture_delais, self.finished_shoot_timer_callback)  
        self._transition(ApproachState.TALKING_PICTURE)
    
    def finished_shoot_timer_callback(self):
        self.get_logger().info(f"Taking picture")

        self.shoot_timer.destroy()
        self.take_picture()
        self.stop_auto_aim()
        
    def target_aimed_callback(self, msg: Bool): 
        if self._state != ApproachState.AIMING:
            return
        
        if msg.data != self.target_aimed:
            self.get_logger().info(f"[AIM] target_aimed changed: {self.target_aimed}")

        self.target_aimed = msg.data
        
        if msg.data:
            self.continues_target_in_sight += 1
        else:
            self.continues_target_in_sight = 0
                
        if self.continues_target_in_sight >= self.continues_in_sight_threshold:
            self.shoot()
            self.continues_target_in_sight = 0
    
    def shoot_target_callback(self, msg: Bool):
        self.get_logger().info(f"[TOPIC] external shoot received: {msg.data}")
        if msg.data:
            self.shoot()
            
    def take_picture_callback(self, msg: Bool):
        self.get_logger().info(f"[TOPIC] take_picture received: {msg.data}")
        if msg.data:
            self.take_picture()
            
            
    def abort_callback(self, msg: Bool):
        self.get_logger().info(f"[TOPIC] abort_all received: {msg.data}")
        if msg.data:
            self.get_logger().warn("[ABORT] Abort command received — resetting state machine")
            self._stop_polar()
            self.define_initial_state()
            self.send_message_to_ui("Abort command processed — returning to idle.")
 
    # ------------------------------------------------------------------
    # Actions
    # ------------------------------------------------------------------
    
    def _stop_polar(self):
        """Safely tell the polar controller to stop, regardless of current state."""
        stop_msg      = String()
        stop_msg.data = "stop"
        self.activate_polar_pub.publish(stop_msg)
        self.get_logger().info("[APPROACH] 'stop' sent to polar controller")
    
    def shoot(self):
        if self._state == ApproachState.SHOOTING:
            self.get_logger().warn("[SHOOT] shoot() called but already SHOOTING — ignoring duplicate")
            return
 
        self.get_logger().info("[SHOOT] Initiating shoot sequence")
        self._transition(ApproachState.SHOOTING)
 
        request      = Bool()
        request.data = True
        self.shoot_pub.publish(request)
        self.get_logger().info("[SHOOT] Shoot command published")
  
    def take_picture(self):
        self.get_logger().info("[PICTURE] Publishing take_picture trigger")
        self.take_picture_pub.publish(Empty())
        self._transition(ApproachState.IDLE)
    
    # ------------------------------------------------------------------
    # UI helper
    # ------------------------------------------------------------------        
    def send_message_to_ui(self, msg: str, is_success: bool = True):
        req            = UiMessage()
        req.message    = msg
        req.is_success = is_success
        self.message_to_ui_pub.publish(req)
        level = self.get_logger().info if is_success else self.get_logger().warn
        level(f"[UI] {'✓' if is_success else '✗'} {msg}")


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousApproach()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down.\n")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()