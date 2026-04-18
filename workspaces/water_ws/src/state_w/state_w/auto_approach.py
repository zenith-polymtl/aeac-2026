#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from custom_interfaces.msg import TargetPosePolar, UiMessage
from std_msgs.msg import Bool, String, UInt8, Empty
from geometry_msgs.msg import PoseStamped, Point, Pose
from zed_msgs.msg import ObjectsStamped
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
    # ABORTED         = "ABORTED"

class AutonomousApproach(Node):
    def __init__(self):
        super().__init__('autonomous_approach')

        self.define_initial_state()
        self.set_up_parameters()
        self.set_up_topics()
        
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
    
    def _transition(self, new_state: str, reason: str = ""):
        """Central state-transition method — every change goes through here."""
        old = self._state
        if old == new_state:
            self.get_logger().debug(f"[STATE] Already in {new_state}, no transition needed.")
            return
        self.get_logger().info(f"[STATE] {old} → {new_state}" + (f"  ({reason})" if reason else ""))
        self._state = new_state
        
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
        self.auto_aim_enable = False
        
        self.in_position          = False
        self.target_aimed         = False
        
        self.get_logger().info("[STATE] State machine reset to IDLE")

    # ------------------------------------------------------------------
    # Parameters
    # ------------------------------------------------------------------
    def set_up_parameters(self):
        self.declare_parameter('approach_radius', 2.0)
        self.declare_parameter('approach_height', 1.0)
        self.declare_parameter('sim', False)
        self.declare_parameter('ignore_state_check', False)
        
        self.approach_radius    = self.get_parameter('approach_radius').value
        self.approach_height    = self.get_parameter('approach_height').value
        self.sim                = self.get_parameter('sim').value
        self.ignore_state_check = self.get_parameter('ignore_state_check').value

    
    # ------------------------------------------------------------------
    # Topics
    # ------------------------------------------------------------------ 
    def set_up_topics(self):
        qos_reliable = self._create_qos_profile(QoSReliabilityPolicy.RELIABLE)
 
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
 
        self.message_to_ui_pub = self.create_publisher(
            UiMessage, 'aeac/external/send_to_ui', qos_reliable)
 
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
 
        self.target_position_sub = self.create_subscription(
            ObjectsStamped, '/aeac/internal/auto_approach/target_detected',
            self.target_position_received_callback, qos_reliable)
 
        self.trigger_target_detection_pub = self.create_publisher(
            Empty, '/aeac/internal/auto_approach/detect_target', qos_reliable)
 
        self.activate_polar_pub = self.create_publisher(
            String, '/aeac/internal/auto_approach/activate_polar', qos_reliable)
 
        self.polar_target_pub = self.create_publisher(
            TargetPosePolar, '/aeac/internal/auto_approach/target_position', qos_reliable)
 
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
 
        self.shoot_pub = self.create_publisher(
            Bool, '/aeac/internal/shoot', qos_reliable)
 
        self.take_picture_pub = self.create_publisher(
            Empty, '/aeac/internal/take_picture', qos_reliable)
      
    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------          
    def auto_approach_activation_callback(self, msg: Bool):
        """Activates or deactivates the autonomous approach.""" 
        if msg.data:
            if self._state != ApproachState.IDLE:
                self.get_logger().warn(
                    f"[GUARD] auto_approach/start: already running (state={self._state}). Ignoring.")
                return
 
            self._transition(ApproachState.DETECTING, "approach activation received")
 
            if not self.sim:
                self.get_logger().info("[DETECT] Publishing trigger to detection node")
                self.trigger_target_detection_pub.publish(Empty())
            else:
                self.get_logger().info("[DETECT] Sim mode — bypassing detection, sending polar target directly")
                self.send_polar_target(None)
 
        else:
            if self._state == ApproachState.IDLE:
                self.get_logger().debug("[GUARD] Deactivation received but already IDLE. Ignoring.")
                return
            self.get_logger().info("[APPROACH] Deactivation received — resetting state machine")
            self._stop_polar()
            self.define_initial_state()

    def target_position_received_callback(self, msg: ObjectsStamped):
        self.get_logger().info(f"[DETECT] Received ObjectsStamped with {len(msg.objects)} object(s)")
        
        if not self._assert_state(ApproachState.DETECTING, "target_position_received_callback"):
            return
        
        if len(msg.objects) == 0:
            self.get_logger().warn("[DETECT] No objects in message.")
            self.send_message_to_ui("No object detected", is_success=False)
            return
        
        best_confidence    = 0.0
        best_found_object  = None
        for obj in msg.objects:
            if obj.confidence > best_confidence:
                best_confidence   = obj.confidence
                best_found_object = obj
 
        self.get_logger().info(
            f"[DETECT] Best object: label={best_found_object.label}  "
            f"confidence={best_confidence*100:.1f}%  pos={list(best_found_object.position)}"
        )
        self.send_message_to_ui(
            f"Object {best_found_object.label} detected with "
            f"{best_confidence*100:.1f}% confidence at {list(best_found_object.position)}"
        )
 
        CONFIDENCE_THRESHOLD = 0.5
        if best_confidence < CONFIDENCE_THRESHOLD:
            self.get_logger().warn(
                f"[DETECT] Confidence {best_confidence:.2f} below threshold "
                f"{CONFIDENCE_THRESHOLD}. Aborting approach."
            )
            self.send_message_to_ui(
                f"Object detected with too low confidence ({best_confidence*100:.1f}%). "
                f"Returning to idle.", is_success=False
            )
            self.define_initial_state()
            return
 
        self.get_logger().info("[DETECT] Target accepted — proceeding to approach")
        self.send_polar_target(best_found_object)
        
    def send_polar_target(self, best_found_object):
        """
        Transforms the detected target into the map frame and publishes the
        centre location, then kicks off the polar approach.
        """
        if not self._assert_state(ApproachState.DETECTING, "send_polar_target"):
            return
        
        try:
            pose_stamped                 = PoseStamped()
            pose_stamped.header.stamp    = self.get_clock().now().to_msg()
            pose_stamped.header.frame_id = "zed_camera_link"
 
            target_position = Pose()
            if not self.sim:
                target_position.position.x = float(best_found_object.position[2])
                target_position.position.y = float(best_found_object.position[1])
                target_position.position.z = float(best_found_object.position[0])
                self.get_logger().info(
                    f"[TF] Raw camera-frame position: "
                    f"x={target_position.position.x:.3f}  "
                    f"y={target_position.position.y:.3f}  "
                    f"z={target_position.position.z:.3f}"
                )
            else:
                target_position.position.x = 5.0
                target_position.position.y = 0.0
                target_position.position.z = 0.0
                self.get_logger().info("[TF] Sim mode — using hardcoded position (5, 0, 0)")
 
            transform = self.tf_buffer.lookup_transform(
                'map', 
                # 'base_link',
                'zed_camera_link', 
                rclpy.time.Time())
 
            pose_stamped.pose = do_transform_pose(target_position, transform)
            self.get_logger().info(
                f"[TF] Map-frame position: "
                f"x={pose_stamped.pose.position.x:.3f}  "
                f"y={pose_stamped.pose.position.y:.3f}  "
                f"z={pose_stamped.pose.position.z:.3f}"
            )
 
            self.polar_center_location_pub.publish(pose_stamped)
            self.get_logger().info("[APPROACH] Published polar centre location")
 
            self.activate_polar_callback()
 
        except Exception as e:
            self.get_logger().error(f"[TF] Transform failed: {e}")
            self.send_message_to_ui("TF transform failed — approach aborted.", is_success=False)
            self.define_initial_state()
            
    def activate_polar_callback(self):   
        if not self._assert_state(ApproachState.DETECTING, "activate_polar_callback"):
            return     
        
        start_msg      = String()
        start_msg.data = "start"
        self.activate_polar_pub.publish(start_msg)
        self.get_logger().info("[APPROACH] Published 'start' to polar controller")
 
        self.send_polar_target_pose()

    def send_polar_target_pose(self):
        if not self._assert_state(ApproachState.DETECTING, "send_polar_target_pose"):
            return
 
        request         = TargetPosePolar()
        request.relative = False
        request.z        = np.nan
        request.theta    = np.nan
        request.r        = self.approach_radius   # use the declared parameter, not a magic number
        self.polar_target_pub.publish(request)
 
        self.get_logger().info(
            f"[APPROACH] Polar target published: r={request.r}  theta={request.theta}  z={request.z}  relative={request.relative}"
        )
        self.send_message_to_ui(f"Starting auto approach — target distance: {request.r} m")
        self._transition(ApproachState.APPROACHING, "polar target sent")        
    
    def in_position_callback(self, msg: Bool):
        self.get_logger().info(f"[TOPIC] in_position received: {msg.data}")
 
        if not self._assert_state(ApproachState.APPROACHING, "in_position_callback"):
            return
 
        if not msg.data:
            self.get_logger().debug("[APPROACH] in_position=False, still moving.")
            return
        
        self.in_position = True
        self._transition(ApproachState.IN_POSITION, "drone reported in position")
        self.send_message_to_ui("Target reached — ready to shoot")
 
        stop_msg      = String()
        stop_msg.data = "stop"
        self.activate_polar_pub.publish(stop_msg)
        self.get_logger().info("[APPROACH] Published 'stop' to polar controller")
    
    def auto_shoot_callback(self, msg: Bool):
        self.get_logger().info(
            f"[TOPIC] auto_shoot/start received: {msg.data}  "
            f"(auto_aim currently={self.auto_aim_enable})"
        )
        if msg.data != self.auto_aim_enable:
            self.get_logger().info("[SHOOT] Toggling auto aim")
            self.toggle_auto_aim()
        else:
            self.get_logger().debug("[SHOOT] auto_aim already matches requested state — no toggle needed")

    def toggle_auto_aim(self):
        self.auto_aim_enable = not self.auto_aim_enable
        self.get_logger().info(f"[SHOOT] auto_aim_enable → {self.auto_aim_enable}")
 
        det_msg      = Bool()
        det_msg.data = self.auto_aim_enable
        self.start_target_detection_pub.publish(det_msg)
 
        gimbal_msg      = UInt8()
        gimbal_msg.data = GimbalMode.AUTO_AIM if self.auto_aim_enable else GimbalMode.FOLLOW
        self.start_auto_shoot_pub.publish(gimbal_msg)
        self.get_logger().info(
            f"[SHOOT] Gimbal mode set to "
            f"{'AUTO_AIM' if self.auto_aim_enable else 'FOLLOW'} ({gimbal_msg.data})"
        )
    
    def target_aimed_callback(self, msg: Bool):
        self.get_logger().info(f"[TOPIC] target_in_aim received: {msg.data}")
 
        # This callback is valid from IN_POSITION or AIMING states
        if self._state not in (ApproachState.IN_POSITION, ApproachState.AIMING,
                                ApproachState.APPROACHING):
            self.get_logger().debug(
                f"[GUARD] target_aimed_callback: state={self._state}, ignoring.")
            return
 
        prev = self.target_aimed
        self.target_aimed = msg.data
 
        if prev != self.target_aimed:
            self.get_logger().info(f"[AIM] target_aimed changed: {prev} → {self.target_aimed}")
 
        if self.target_aimed and self.in_position:
            if self._state != ApproachState.SHOOTING:
                self.get_logger().info("[AIM] In position AND aimed — triggering shoot")
                self.shoot()
        elif self.target_aimed:
            self.get_logger().info("[AIM] Aimed but not yet in position — waiting")
        else:
            self.get_logger().debug("[AIM] Target not aimed")
    
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
        self._transition(ApproachState.SHOOTING, "shoot triggered")
 
        request      = Bool()
        request.data = True
        self.shoot_pub.publish(request)
        self.get_logger().info("[SHOOT] Shoot command published")
 
        self.take_picture()
        self.get_logger().info("[SHOOT] Shoot + picture sequence complete")
 
    def take_picture(self):
        self.get_logger().info("[PICTURE] Publishing take_picture trigger")
        self.take_picture_pub.publish(Empty())
    
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