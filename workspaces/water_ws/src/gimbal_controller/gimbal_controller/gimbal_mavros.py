import rclpy
from rclpy.node import Node
import math

from mavros_msgs.msg import MountControl
from mavros_msgs.srv import MountConfigure
from std_msgs.msg import Bool, UInt8
from custom_interfaces.msg import AimError, GimbalState
from geometry_msgs.msg import Quaternion, TransformStamped
from std_srvs.srv import SetBool
from tf_transformations import euler_from_quaternion
from mavros_msgs.msg import GimbalManagerSetPitchyaw, GimbalDeviceAttitudeStatus
from tf2_ros import TransformBroadcaster
from mavros_msgs.srv import MessageInterval
import numpy as np

class GIMBAL_MANAGER_FLAGS:
    GIMBAL_MANAGER_FLAGS_RETRACT = 1
    GIMBAL_MANAGER_FLAGS_NEUTRAL = 2
    GIMBAL_MANAGER_FLAGS_ROLL_LOCK = 4
    GIMBAL_MANAGER_FLAGS_PITCH_LOCK = 8
    GIMBAL_MANAGER_FLAGS_YAW_LOCK = 16
    GIMBAL_MANAGER_FLAGS_YAW_IN_VEHICLE_FRAME = 32
    GIMBAL_MANAGER_FLAGS_YAW_IN_EARTH_FRAME = 64
    GIMBAL_MANAGER_FLAGS_ACCEPTS_YAW = 128
    GIMBAL_MANAGER_FLAGS_RC_EXCLUSIVE = 256
    GIMBAL_MANAGER_FLAGS_RC_MIXED = 512

POSITION_FLAG = GIMBAL_MANAGER_FLAGS.GIMBAL_MANAGER_FLAGS_YAW_IN_VEHICLE_FRAME + GIMBAL_MANAGER_FLAGS.GIMBAL_MANAGER_FLAGS_PITCH_LOCK + GIMBAL_MANAGER_FLAGS.GIMBAL_MANAGER_FLAGS_ROLL_LOCK
SPEED_FLAGS = GIMBAL_MANAGER_FLAGS.GIMBAL_MANAGER_FLAGS_YAW_IN_VEHICLE_FRAME + GIMBAL_MANAGER_FLAGS.GIMBAL_MANAGER_FLAGS_PITCH_LOCK + GIMBAL_MANAGER_FLAGS.GIMBAL_MANAGER_FLAGS_ROLL_LOCK

#Keep YAW_IN_VEHICLE_FRAME to ensure proper dynamic transform rotation 

# real limits
# MAX_ANGLE_PITCH = 120.0
# MAX_ANGLE_YAW = 325.0

# safe limits
MAX_ANGLE_PITCH = 45.0
MAX_ANGLE_YAW =60.0

AIM_ERROR_ACCEPTANCE = 10.0

class GimbalMode:
    LOCK = 0
    FOLLOW = 1
    AUTO_AIM = 2
class AimingState:
    def __init__(self, gimbal):
        self.gimbal = gimbal

        self.in_aim = False
        self.detecting = False
        self.recovery_active = False

        self.last_error = None
        self.aim_start_time = None
        self.last_detection_time = None

        self.angle_lost_increment = 20.0
        self.first_recovery = True

        self.search_increment = 40.0
        self.search_index = 0

        self.lost_pitch = 0.0
        self.lost_yaw = 0.0

        # Sweep target tracking
        self.current_sweep_target = None          # (pitch_deg, yaw_deg)
        self.current_sweep_target_time = None
        self.sweep_tolerance_deg = 3.0
        self.sweep_target_timeout = 2.5       # safety timeout in seconds

        self.final_aim = None

        s = self.search_increment
        self.search_patterns = [
            (s, 0.0),
            (-s, 0.0),
            (0.0, s),
            (0.0, -s),
        ]

        self.control_timer = self.gimbal.create_timer(0.1, self.control_loop)
        self.recovery_timer = None

    def _log_event(self, message):
        self.gimbal.get_logger().info(f"[recovery] {message}")

    def _log_warn(self, message):
        self.gimbal.get_logger().warn(f"[recovery] {message}")

    def reset(self):
        self.in_aim = False
        self.detecting = False
        self.recovery_active = False

        self.last_error = None
        self.aim_start_time = None
        self.last_detection_time = None

        self.first_recovery = True
        self.search_index = 0

        self.current_sweep_target = None
        self.current_sweep_target_time = None

        if self.recovery_timer is not None:
            self.recovery_timer.cancel()
            self.recovery_timer = None

    def control_loop(self):
        if self.in_aim:
            return

        if self.gimbal.gimbal_mode != GimbalMode.AUTO_AIM:
            return

        if self.last_detection_time is None:
            return

        elapsed = (
            self.gimbal.get_clock().now() - self.last_detection_time
        ).nanoseconds / 1e9

        if elapsed > 0.5 and not self.recovery_active:
            self.detecting = False
            self.recovery_active = True
            self.first_recovery = True
            self.search_index = 0
            self.current_sweep_target = None
            self.current_sweep_target_time = None

            self._log_event(
                f"target lost for {elapsed:.2f}s; starting recovery"
            )

            self.gimbal.gimbal_mode = GimbalMode.FOLLOW
            self.gimbal.reset_pid_memory()
            self.gimbal.send_speed_cmd(0.0, 0.0)

            self.lost_pitch = self.gimbal.current_pitch
            self.lost_yaw = self.gimbal.current_yaw

            if self.first_recovery and self.last_error is not None:
                transform_error = self.transform_error_to_angle(self.last_error)

                pitch_cmd = self.lost_pitch + transform_error[0]
                yaw_cmd = self.lost_yaw + transform_error[1]

                pitch_cmd = max(-MAX_ANGLE_PITCH, min(MAX_ANGLE_PITCH, pitch_cmd))
                yaw_cmd = max(-MAX_ANGLE_YAW, min(MAX_ANGLE_YAW, yaw_cmd))

                self._log_event(
                    f"first recovery target: pitch={pitch_cmd:.2f} deg, "
                    f"yaw={yaw_cmd:.2f} deg, "
                    f"last_error=({self.last_error[0]:.2f}, {self.last_error[1]:.2f}), "
                    f"position when lost=({self.lost_pitch:.2f}, {self.lost_yaw:.2f})"
                )

            self.recovery_timer = self.gimbal.create_timer(
                0.1,
                self.recovery_loop
            )

            self.recovery_loop()

    def _is_at_sweep_target(self):
        if self.current_sweep_target is None:
            return False

        target_pitch, target_yaw = self.current_sweep_target

        pitch_error = abs(self.gimbal.current_pitch - target_pitch)
        yaw_error = abs(self.gimbal.current_yaw - target_yaw)

        return (
            pitch_error < self.sweep_tolerance_deg and
            yaw_error < self.sweep_tolerance_deg
        )

    def _sweep_target_timed_out(self):
        if self.current_sweep_target_time is None:
            return False

        elapsed = (
            self.gimbal.get_clock().now() - self.current_sweep_target_time
        ).nanoseconds / 1e9

        return elapsed > self.sweep_target_timeout

    def _send_sweep_target(self):
        pitch_offset, yaw_offset = self.search_patterns[self.search_index]

        pitch_cmd = self.lost_pitch + pitch_offset
        yaw_cmd = self.lost_yaw + yaw_offset

        pitch_cmd = max(-MAX_ANGLE_PITCH, min(MAX_ANGLE_PITCH, pitch_cmd))
        yaw_cmd = max(-MAX_ANGLE_YAW, min(MAX_ANGLE_YAW, yaw_cmd))

        self.current_sweep_target = (pitch_cmd, yaw_cmd)
        self.current_sweep_target_time = self.gimbal.get_clock().now()

        self._log_event(
            f"search target {self.search_index + 1}/{len(self.search_patterns)}: "
            f"pitch={pitch_cmd:.2f} deg, yaw={yaw_cmd:.2f} deg"
        )

        self.gimbal.send_position_cmd(
            math.radians(pitch_cmd),
            math.radians(yaw_cmd)
        )

    def recovery_loop(self):
        recent_detection = False

        if self.last_detection_time is not None:
            recent_detection = (
                self.gimbal.get_clock().now() - self.last_detection_time
            ).nanoseconds / 1e9 < 0.5

        if self.detecting and recent_detection:
            self._log_event("target reacquired; stopping recovery")

            self.reset()
            self.gimbal.gimbal_mode = GimbalMode.AUTO_AIM
            return

        # If a sweep target was already sent, wait until the gimbal reaches it.
        if self.current_sweep_target is not None:
            if self._is_at_sweep_target():
                target_pitch, target_yaw = self.current_sweep_target

                self._log_event(
                    f"target reached: pitch={target_pitch:.2f} deg, "
                    f"yaw={target_yaw:.2f} deg"
                )

                self.current_sweep_target = None
                self.current_sweep_target_time = None
                self.search_index += 1

            elif self._sweep_target_timed_out():
                target_pitch, target_yaw = self.current_sweep_target

                self._log_warn(
                    f"target timeout: pitch={target_pitch:.2f} deg, "
                    f"yaw={target_yaw:.2f} deg; moving to next sweep target"
                )

                self.current_sweep_target = None
                self.current_sweep_target_time = None
                self.search_index += 1

            else:
                # Still moving toward current target. Do not send another command yet.
                return

        # All sweep targets completed.
        if self.search_index >= len(self.search_patterns):
            self._log_event("search pattern finished; returning to neutral")

            self.gimbal.reset_pid_memory()
            self.gimbal.send_speed_cmd(0.0, 0.0)
            self.gimbal.send_position_cmd(0.0, 0.0)

            self.gimbal.gimbal_mode = GimbalMode.FOLLOW

            self.final_aim = self.gimbal.create_timer(
                0.1,
                self.final_aim_callback
            )

            self.reset()
            return

        # Send next sweep target only after the previous one was reached or timed out.
        self._send_sweep_target()

    def final_aim_callback(self):
        if math.sqrt(self.gimbal.current_pitch**2 + self.gimbal.current_yaw**2) < 3.0:
            if self.final_aim is not None:
                self.final_aim.cancel()
                self.final_aim = None

            self.gimbal.gimbal_mode = GimbalMode.AUTO_AIM
            self._log_event("neutral reached; returning to AUTO_AIM")

    def transform_error_to_angle(self, error):
        error = np.array(error, dtype=float)
        norm = np.linalg.norm(error)

        if norm < 1e-6:
            return np.array([0.0, 0.0])

        unit_vector = error / norm
        return self.angle_lost_increment * unit_vector

    def update(self, error):
        self.last_error = np.array(error, dtype=float)
        self.last_detection_time = self.gimbal.get_clock().now()
        self.detecting = True

    def lost(self):
        self.detecting = False
        


class PIDController():
    def __init__(self, kp, ki, kd, max_output=0.5, max_i=1.0,
                 deriv_tau=0.0, d_clip=None):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.max_output, self.max_i = max_output, max_i
        self.prev_error = 0.0
        self.prev_error2 = 0.0
        self.integral = 0.0
        self.deriv_tau = deriv_tau  # [s], 0.06–0.12 works well at 30 Hz
        self._d_state = 0.0
        self.d_clip = d_clip
        # expose last computed PID components for logging
        self.last_p = 0.0
        self.last_i = 0.0
        self.last_d = 0.0


    def compute(self, error, dt, d_meas=None):
        if dt <= 0: return 0.0
        # I
        self.integral += error * dt
        self.integral = max(-self.max_i, min(self.integral, self.max_i))

        # store I term (kiintegral) for logging
        self.last_i = self.ki * self.integral

        # P term for logging
        self.last_p = self.kp * error

        #Second order backward derivative. If d_meas is provided, it's already a rate.
        raw_d = d_meas if d_meas is not None else (3 * error - 4 * self.prev_error + self.prev_error2)/ (2 * dt)

        # Low-pass filter
        a = self.deriv_tau / (self.deriv_tau + dt)  # 0<a<1
        self._d_state = a * self._d_state + (1.0 - a) * raw_d
        dterm = self.kd * self._d_state
        if self.d_clip is not None:
            dterm = max(-self.d_clip, min(dterm, self.d_clip))
    
        # store D term (post clipping) for logging
        self.last_d = dterm

        # Sum & clamp
        u = self.last_p + self.last_i + dterm
        self.prev_error2 = self.prev_error
        self.prev_error = error

        return max(-self.max_output, min(u, self.max_output))

    def reset(self):
        self.prev_error = 0.0
        self.prev_error2 = 0.0
        self.integral = 0.0
        self._d_state = 0.0

class GremsyMavros(Node):
    def __init__(self):
        super().__init__('gremsy_mavros_ctrl')

        # talk=True keeps logs enabled; talk=False silences logs for faster runtime.
        self.declare_parameter('talk', True)
        self.talk = bool(self.get_parameter('talk').value)
        if not self.talk:
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.FATAL)

        self.pid_pitch = PIDController(kp=0.008, ki=0.0001, kd=0.0004, max_output=1.5, max_i=0.05, deriv_tau=0.12)
        self.pid_yaw = PIDController(kp=0.008, ki=0.0001, kd=0.0004, max_output=1.5, max_i=0.05, deriv_tau=0.12)
        #self.pid_yaw = PIDController(kp=0.01, ki=0.001, kd=0.0001, max_output=1.5, max_i=0.3, deriv_tau=0.12)

        self.last_sent_pitch_vel = 0.0
        self.last_sent_yaw_vel = 0.0

        self.gimbal_mode = GimbalMode.FOLLOW
        
        # --- Publishers ---
        self.mavros_pub = self.create_publisher(GimbalManagerSetPitchyaw, '/mavros/gimbal_control/manager/set_pitchyaw', 10)

        self.state_timer = self.create_timer(0.5, self.publish_state)
        self.state_pub = self.create_publisher(GimbalState, '/aeac/external/gimbal/state', 10)
        self.target_in_aim_pub = self.create_publisher(Bool, '/aeac/internal/auto_shoot/target_in_aim', 10)
        # self.shoot_pub = self.create_publisher(Bool, '', 10)

        # --- Subscribers ---

        self.create_subscription(UInt8, '/aeac/external/gimbal/set_mode', self.set_mode_callback, 10)

        self.create_subscription(AimError, '/aeac/external/gimbal/move', self.gimbal_move_callback, 10)

        self.create_subscription(AimError, '/aeac/internal/gimbal/target_error', self.aiming_callback, 10)
        self.create_subscription(Bool, '/aeac/internal/auto_shoot/start_hr_aiming', self.finished_aiming_callback, 10)


        self.sub_orientation = self.create_subscription(
            GimbalDeviceAttitudeStatus,
            '/mavros/gimbal_control/device/attitude_status',
            self.gimbal_attitude_callback,
            10)
 
        self.msg_interval_client = self.create_client(
            MessageInterval,
            '/mavros/set_message_interval'
        )
        
        # TF broadcaster
        self.tf_br = TransformBroadcaster(self)

        # Frame names
        self.parent_frame = 'base_link'
        self.child_frame  = 'gimbal_link'

        # Gimbal mount position relative to base_link (meters)
        # Exemple: 10 cm forward, 0 left, -10 cm up (FLU base_link: x fwd, y left, z up)
        self.gimbal_x = 0.10
        self.gimbal_y = 0.0
        self.gimbal_z = -0.10
        

        self.current_pitch = 0.0
        self.current_yaw = 0.0
        self.current_roll = 0.0

        self.last_sent_pitch_vel = 0.0
        self.last_sent_yaw_vel = 0.0
        self.last_update_time = self.get_clock().now()

        self.send_position_cmd(0.0,0.0)

        self.recovery_state = AimingState(self)

        self.request_message_interval()

        self.get_logger().info("Gimbal MAVROS controller initialized.")

    def request_message_interval(self):
        if not self.msg_interval_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn('Message interval service not available')
            return  

        req = MessageInterval.Request()
        req.message_id = 285
        req.message_rate = 20.0

        future = self.msg_interval_client.call_async(req)
        future.add_done_callback(self.message_interval_callback)

    def message_interval_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Message interval set successfully')
            else:
                self.get_logger().error('Failed to set message interval')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def publish_state(self):
        state_msg = GimbalState()

        if self.gimbal_mode == GimbalMode.LOCK:
            mode_str = "LOCK"
        elif self.gimbal_mode == GimbalMode.FOLLOW:
            mode_str = "FOLLOW"
        elif self.gimbal_mode == GimbalMode.AUTO_AIM:
            mode_str = "Auto AIM"
        else: 
            mode_str = "UNKNOWN"

        state_msg.mode = mode_str
        state_msg.pitch = self.current_pitch
        state_msg.yaw = self.current_yaw
        self.state_pub.publish(state_msg)
        self.log_debug(f"Published gimbal state: mode={mode_str}, pitch={self.current_pitch:.2f}, yaw={self.current_yaw:.2f}")

    # --- Rate-limited logging helpers (log 1 out of `self._log_mod` calls) ---
    def _should_log(self):
        self._log_tick = getattr(self, '_log_tick', 0) + 1
        if self._log_tick >= getattr(self, '_log_mod', 10):
            self._log_tick = 0
            setattr(self, '_log_tick', self._log_tick)
            return True
        setattr(self, '_log_tick', self._log_tick)
        return False

    def log_debug(self, msg):
        if self._should_log():
            self.get_logger().debug(msg)

    def log_warn(self, msg):
        if self._should_log():
            self.get_logger().warn(msg)

    def log_info(self, msg):
        if self._should_log():
            self.get_logger().info(msg)

    def gimbal_attitude_callback(self, msg):
        q = msg.q

        # --- Publish TF: base_link -> gimbal_link (dynamic rotation) ---
        t = TransformStamped()
        t.header.stamp = msg.header.stamp  # use MAVROS stamp
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame

        # Fixed mount translation (gimbal center wrt base_link)
        t.transform.translation.x = float(self.gimbal_x)
        t.transform.translation.y = float(self.gimbal_y)
        t.transform.translation.z = float(self.gimbal_z)

        # Dynamic rotation (gimbal attitude)
        # IMPORTANT: this assumes q is already expressed as rotation of gimbal_link in base_link frame
        # and in the same axis convention as your TF tree.
        t.transform.rotation.x = float(q.x)
        t.transform.rotation.y = float(q.y)
        t.transform.rotation.z = float(q.z)
        t.transform.rotation.w = float(q.w)

        self.tf_br.sendTransform(t)

        angles = euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        self.current_roll = math.degrees(angles[0])
        self.current_pitch = math.degrees(angles[1])
        self.current_yaw = math.degrees(angles[2])

        
        self.log_debug(f"Current pitch: {self.current_pitch}, Current yaw: {self.current_yaw}")
        target_pitch, target_yaw = self.check_angle_limit(self.last_sent_pitch_vel, self.last_sent_yaw_vel)
    

    def reset_pid_memory(self):
        self.pid_pitch.reset()
        self.pid_yaw.reset()

    def reset_gimbal_position(self):
        self.send_position_cmd(0.0, 0.0)
        self.get_logger().info("Gimbal position reset to neutral (0,0).")

    def finished_aiming_callback(self, msg):
        if not msg.data:
            self.get_logger().info("Finished aiming received. Resetting gimbal position and PID memory.")
            self.reset_pid_memory()
            self.reset_gimbal_position()
            self.gimbal_mode = GimbalMode.FOLLOW
            self.publish_state()
            self.recovery_state.reset()

    def set_mode_callback(self, msg):
        self.reset_pid_memory()
        self.recovery_state.reset()

        new_mode = msg.data

        if new_mode == GimbalMode.LOCK:
            self.send_speed_cmd(0.0, 0.0)
            self.get_logger().info("Changed mode to LOCK.")

        elif new_mode == GimbalMode.FOLLOW:
            self.send_position_cmd(0.0, 0.0)
            self.get_logger().info("Changed mode to FOLLOW.")
        
        elif new_mode == GimbalMode.AUTO_AIM:
            self.send_speed_cmd(0.0, 0.0)
            self.recovery_state.last_detection_time = self.get_clock().now() + rclpy.duration.Duration(seconds=1.0)  
            self.get_logger().info("Changed mode to AUTO_AIM.")

        self.gimbal_mode = new_mode
        self.publish_state()



    def check_angle_limit(self, target_vel_pitch, target_vel_yaw):
        # --- SECURITY PITCH ---
        if self.current_pitch >= MAX_ANGLE_PITCH and target_vel_pitch > 0:
            self.log_warn("MAX PITCH reached!")
            target_vel_pitch = 0.0
        elif self.current_pitch <= -MAX_ANGLE_PITCH and target_vel_pitch < 0:
            self.log_warn("MIN PITCH reached!")
            target_vel_pitch = 0.0

        # --- SECURITY YAW---
        if self.current_yaw >= MAX_ANGLE_YAW and target_vel_yaw > 0:
            self.log_warn("MAX YAW reached!")
            target_vel_yaw = 0.0
        elif self.current_yaw <= -MAX_ANGLE_YAW and target_vel_yaw < 0:
            self.log_warn("MIN YAW reached!")
            target_vel_yaw = 0.0
        
        return target_vel_pitch, target_vel_yaw
    
    def gimbal_move_callback(self, msg):
        if self.gimbal_mode != GimbalMode.LOCK:
            return
        
        target_vel_pitch, target_vel_yaw = self.check_angle_limit(msg.pitch_error, msg.yaw_error)
        self.send_speed_cmd(target_vel_pitch, target_vel_yaw)
        self.log_debug(f"Received move command: pitch_error={msg.pitch_error}, yaw_error={msg.yaw_error}. Sent speeds: pitch_vel={target_vel_pitch}, yaw_vel={target_vel_yaw}")

    def aiming_callback(self, msg):
        err_norm = math.sqrt(msg.pitch_error**2 + msg.yaw_error**2)

        # During recovery, only use target_error to detect reacquisition.
        # Do not send PID speed commands.
        if self.recovery_state.recovery_active:
            if err_norm >= 1.0:
                self.recovery_state.update((msg.pitch_error, msg.yaw_error))
            return

        if self.gimbal_mode != GimbalMode.AUTO_AIM:
            return

        current_time = self.get_clock().now()
        dt = (current_time - self.last_update_time).nanoseconds / 1e9
        self.last_update_time = current_time

        if dt <= 0.0 or dt > 0.1:
            dt = 0.01

        if err_norm < 1.0:
            self.log_info("NOT SENDING COMMANDS - ERROR NONE")
            self.send_speed_cmd(0.0, 0.0)
            self.recovery_state.lost()
            return

        self.recovery_state.update((msg.pitch_error, msg.yaw_error))

        target_vel_pitch = self.pid_pitch.compute(msg.pitch_error, dt)
        target_vel_yaw = self.pid_yaw.compute(msg.yaw_error, dt)

        in_sight_msg = Bool()
        in_sight_msg.data = (
            abs(msg.pitch_error) < AIM_ERROR_ACCEPTANCE and
            abs(msg.yaw_error) < AIM_ERROR_ACCEPTANCE
        )
        self.target_in_aim_pub.publish(in_sight_msg)

        target_vel_pitch, target_vel_yaw = self.check_angle_limit(
            target_vel_pitch,
            target_vel_yaw
        )

        self.send_speed_cmd(target_vel_pitch, target_vel_yaw)

    def send_position_cmd(self, pitch, yaw):
        self.last_sent_pitch_vel = 0.0
        self.last_sent_yaw_vel = 0.0

        msg = GimbalManagerSetPitchyaw()

        msg.flags = POSITION_FLAG
        msg.pitch = float(pitch)
        msg.yaw = float(yaw)

        msg.pitch_rate = float('nan')
        msg.yaw_rate = float('nan')
        self.mavros_pub.publish(msg)
        self.log_debug(f"Sent position command: pitch={pitch}, yaw={yaw}")

    def send_speed_cmd(self, pitch_rate, yaw_rate):
        self.last_sent_pitch_vel = pitch_rate
        self.last_sent_yaw_vel = yaw_rate

        msg = GimbalManagerSetPitchyaw()
        
        msg.flags = SPEED_FLAGS
        msg.pitch = float('nan')
        msg.yaw = float('nan')

        msg.pitch_rate = float(pitch_rate)
        msg.yaw_rate = float(yaw_rate)
        self.mavros_pub.publish(msg)
        self.log_debug(f"Sent speed command: pitch_rate={pitch_rate}, yaw_rate={yaw_rate}")

def main(args=None):
    rclpy.init(args=args)
    node = GremsyMavros()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.reset_gimbal_position()
        node.send_speed_cmd(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()