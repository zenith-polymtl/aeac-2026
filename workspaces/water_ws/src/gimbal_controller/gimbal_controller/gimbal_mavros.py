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
SPEED_FLAGS = GIMBAL_MANAGER_FLAGS.GIMBAL_MANAGER_FLAGS_YAW_LOCK + GIMBAL_MANAGER_FLAGS.GIMBAL_MANAGER_FLAGS_PITCH_LOCK + GIMBAL_MANAGER_FLAGS.GIMBAL_MANAGER_FLAGS_ROLL_LOCK

#Keep YAW_IN_VEHICLE_FRAME to ensure proper dynamic transform rotation 

# real limits
# MAX_ANGLE_PITCH = 120.0
# MAX_ANGLE_YAW = 325.0

# safe limits
MAX_ANGLE_PITCH = 110.0
MAX_ANGLE_YAW = 130.0

AIM_ERROR_ACCEPTANCE = 10.0

class GimbalMode:
    LOCK = 0
    FOLLOW = 1
    AUTO_AIM = 2

class PIDController():
    def __init__(self, kp, ki, kd, max_output=3.0, max_i=1.0,
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

        self.pid_pitch = PIDController(kp=0.001, ki=0.0001, kd=0.002, max_output=0.7, max_i=0.5, deriv_tau=0.15)
        self.pid_yaw = PIDController(kp=0.001, ki=0.0001, kd=0.002, max_output=0.7, max_i=0.5, deriv_tau=0.15)

        self.last_sent_pitch_vel = 0.0
        self.last_sent_yaw_vel = 0.0

        self.gimbal_mode = GimbalMode.FOLLOW
        
        # --- Publishers ---
        self.mavros_pub = self.create_publisher(GimbalManagerSetPitchyaw, '/mavros/gimbal_control/manager/set_pitchyaw', 10)

        self.state_timer = self.create_timer(0.5, self.publish_state)
        self.state_pub = self.create_publisher(GimbalState, '/aeac/external/gimbal/state', 10)

        # self.shoot_pub = self.create_publisher(Bool, '', 10)

        # --- Subscribers ---

        self.create_subscription(UInt8, '/aeac/external/gimbal/set_mode', self.set_mode_callback, 10)

        self.create_subscription(AimError, '/aeac/external/gimbal/move', self.gimbal_move_callback, 10)

        self.create_subscription(AimError, '/aeac/internal/gimbal/target_error', self.aiming_callback, 10)

        self.sub_orientation = self.create_subscription(
            GimbalDeviceAttitudeStatus,
            '/mavros/gimbal_control/device/attitude_status',
            self.gimbal_attitude_callback,
            10)
        
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
        
        # --- Services ---
        self.create_service(SetBool, '/gimbal/lock_mode', self.enable_lock_mode_callback)

        self.get_logger().info("Gimbal MAVROS ready.")

    def publish_state(self):
        state_msg = GimbalState()
        mode_str = "LOCK" if self.gimbal_mode == GimbalMode.LOCK else "FOLLOW"
        state_msg.mode = mode_str
        state_msg.pitch = self.current_pitch
        state_msg.yaw = self.current_yaw
        self.state_pub.publish(state_msg)

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

        # self.get_logger().info(f"Current pitch: {self.current_pitch}, Current yaw: {self.current_yaw}")
        target_pitch, target_yaw = self.check_angle_limit(self.last_sent_pitch_vel, self.last_sent_yaw_vel)
        
        if target_pitch != self.last_sent_pitch_vel or target_yaw != self.last_sent_yaw_vel:
            self.send_speed_cmd(target_pitch, target_yaw)

    def reset_pid_memory(self):
        self.pitch_integral = 0.0
        self.pitch_prev_error = 0.0
        self.yaw_integral = 0.0
        self.yaw_prev_error = 0.0
        self.last_update_time = self.get_clock().now()

    def set_mode_callback(self, msg):
        self.reset_pid_memory()

        new_mode = msg.data

        if new_mode == GimbalMode.LOCK:
            self.send_speed_cmd(0.0, 0.0)
            self.get_logger().info("Changed mode to LOCK.")

        elif new_mode == GimbalMode.FOLLOW:
            self.send_position_cmd(0.0, 0.0)
            self.get_logger().info("Changed mode to FOLLOW.")
        
        elif new_mode == GimbalMode.AUTO_AIM:
            self.send_speed_cmd(0.0, 0.0)
            self.get_logger().info("Changed mode to AUTO_AIM.")

        self.gimbal_mode = new_mode
        self.publish_state()

    def check_angle_limit(self, target_vel_pitch, target_vel_yaw):
        # --- SECURITY PITCH ---
        if self.current_pitch >= MAX_ANGLE_PITCH and target_vel_pitch > 0:
            self.get_logger().warn("MAX PITCH reached!")
            target_vel_pitch = 0.0
        elif self.current_pitch <= -MAX_ANGLE_PITCH and target_vel_pitch < 0:
            self.get_logger().warn("MIN PITCH reached!")
            target_vel_pitch = 0.0

        # --- SECURITY YAW---
        if self.current_yaw >= MAX_ANGLE_YAW and target_vel_yaw > 0:
            self.get_logger().warn("MAX YAW reached!")
            target_vel_yaw = 0.0
        elif self.current_yaw <= -MAX_ANGLE_YAW and target_vel_yaw < 0:
            self.get_logger().warn("MIN YAW reached!")
            target_vel_yaw = 0.0
        
        return target_vel_pitch, target_vel_yaw
    
    def gimbal_move_callback(self, msg):
        if self.gimbal_mode != GimbalMode.LOCK:
            return
        
        target_vel_pitch, target_vel_yaw = self.check_angle_limit(msg.pitch_error, msg.yaw_error)
        self.send_speed_cmd(target_vel_pitch, target_vel_yaw)

    def aiming_callback(self, msg):
        if self.gimbal_mode != GimbalMode.AUTO_AIM:
            return

        target_vel_yaw = msg.yaw_error
    
        current_time = self.get_clock().now()
        dt = (current_time - self.last_update_time).nanoseconds / 1e9
        self.last_update_time = current_time

        if dt <= 0.0 or dt > 0.1:
            dt = 0.01

        target_vel_pitch = self.pid_pitch.compute(msg.pitch_error, dt)
        target_vel_yaw = self.pid_yaw.compute(msg.yaw_error, dt)

        if abs(msg.pitch_error) < AIM_ERROR_ACCEPTANCE and abs(msg.yaw_error) < AIM_ERROR_ACCEPTANCE:
            target_vel_pitch = 0.0   
            target_vel_yaw = 0.0
            # self.shoot_pub.publish(Bool(data=True))

        target_vel_pitch, target_vel_yaw = self.check_angle_limit(target_vel_pitch, target_vel_yaw)
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

def main(args=None):
    rclpy.init(args=args)
    node = GremsyMavros()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()