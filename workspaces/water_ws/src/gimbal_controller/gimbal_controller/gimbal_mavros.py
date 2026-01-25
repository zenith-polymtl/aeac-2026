import rclpy
from rclpy.node import Node
import math

from mavros_msgs.msg import MountControl
from mavros_msgs.srv import MountConfigure
from custom_interfaces.msg import AimError
from geometry_msgs.msg import Quaternion
from std_srvs.srv import SetBool
from tf_transformations import euler_from_quaternion
from mavros_msgs.msg import GimbalManagerSetPitchyaw, GimbalDeviceAttitudeStatus

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

# real limits
# MAX_ANGLE_PITCH = 120.0
# MAX_ANGLE_YAW = 325.0

# safe limits
MAX_ANGLE_PITCH = 110.0
MAX_ANGLE_YAW = 100

class GimbalMode:
    LOCK = 1
    FOLLOW = 2

class GremsyMavros(Node):
    def __init__(self):
        super().__init__('gremsy_mavros_ctrl')

        # --- PID params ---
        self.kp = 2.0
        self.ki = 0.1
        self.kd = 0.05
        self.max_speed = 30.0 

        # PID Memory
        self.pitch_integral = 0.0
        self.pitch_prev_error = 0.0
        self.yaw_integral = 0.0
        self.yaw_prev_error = 0.0
        self.last_update_time = self.get_clock().now()

        self.gimbal_mode = 2 # 1=LOCK, 2=FOLLOW
        
        # --- Publishers ---
        self.mavros_pub = self.create_publisher(GimbalManagerSetPitchyaw, '/mavros/gimbal_control/manager/set_pitchyaw', 10)

        # --- Subscribers ---
        self.create_subscription(AimError, '/gimbal/target_error', self.aiming_callback, 10)
        
        self.sub_orientation = self.create_subscription(
            GimbalDeviceAttitudeStatus,
            '/mavros/gimbal_control/device/attitude_status',
            self.gimbal_attitude_callback,
            10)

        self.current_pitch = 0.0
        self.current_yaw = 0.0
        self.current_roll = 0.0

        self.last_sent_pitch_vel = 0.0
        self.last_sent_yaw_vel = 0.0
        
        # --- Services ---
        self.create_service(SetBool, '/gimbal/lock_mode', self.enable_lock_mode_callback)

        self.get_logger().info("Gimbal MAVROS ready.")

    def gimbal_attitude_callback(self, msg):
        q = msg.q

        angles = euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        self.current_roll = math.degrees(angles[0])
        self.current_pitch = math.degrees(angles[1])
        self.current_yaw = math.degrees(angles[2])

        print(f"{self.current_pitch}, {self.current_yaw}")
        target_pitch, target_yaw = self.check_angle_limit(self.last_sent_pitch_vel, self.last_sent_yaw_vel)
        
        if target_pitch != self.last_sent_pitch_vel or target_yaw != self.last_sent_yaw_vel:
            self.send_speed_cmd(target_pitch, target_yaw)

    def reset_pid_memory(self):
        self.pitch_integral = 0.0
        self.pitch_prev_error = 0.0
        self.yaw_integral = 0.0
        self.yaw_prev_error = 0.0
        self.last_update_time = self.get_clock().now()

    def enable_lock_mode_callback(self, request, response):
        self.reset_pid_memory()

        new_mode = GimbalMode.LOCK if request.data else GimbalMode.FOLLOW

        if new_mode == GimbalMode.LOCK:
            self.gimbal_mode = new_mode
            self.send_speed_cmd(0.0, 0.0)
            response.success = True
            response.message = "Changed mode to LOCK"

        elif new_mode == GimbalMode.FOLLOW:
            self.gimbal_mode = new_mode
            self.send_position_cmd(0.0, 0.0)
            response.success = True
            response.message = "Changed mode to FOLLOW"

        return response

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
    
    def aiming_callback(self, msg):
        if self.gimbal_mode != GimbalMode.LOCK:
            return

        target_vel_pitch = msg.pitch_error
        target_vel_yaw = msg.yaw_error

        target_vel_pitch, target_vel_yaw = self.check_angle_limit(target_vel_pitch, target_vel_yaw)

        self.send_speed_cmd(target_vel_pitch, target_vel_yaw)
        return
    
        current_time = self.get_clock().now()
        dt = (current_time - self.last_update_time).nanoseconds / 1e9
        self.last_update_time = current_time

        if dt <= 0.0 or dt > 0.1:
            dt = 0.01

        cmd_pitch = self.compute_pid(msg.pitch_error, "pitch", dt)
        cmd_yaw = self.compute_pid(msg.yaw_error, "yaw", dt)

        cmd_pitch = max(min(cmd_pitch, self.max_speed), -self.max_speed)
        cmd_yaw = max(min(cmd_yaw, self.max_speed), -self.max_speed)

        self.send_speed_cmd(cmd_pitch, cmd_yaw)

    def compute_pid(self, error, axis, dt):
        if axis == "pitch":
            p = error * self.kp
            self.pitch_integral += error * dt
            self.pitch_integral = max(min(self.pitch_integral, 10.0), -10.0)
            i = self.pitch_integral * self.ki
            d = ((error - self.pitch_prev_error) / dt) * self.kd
            self.pitch_prev_error = error
        else:
            p = error * self.kp
            self.yaw_integral += error * dt
            self.yaw_integral = max(min(self.yaw_integral, 10.0), -10.0)
            i = self.yaw_integral * self.ki
            d = ((error - self.yaw_prev_error) / dt) * self.kd
            self.yaw_prev_error = error
        return p + i + d

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