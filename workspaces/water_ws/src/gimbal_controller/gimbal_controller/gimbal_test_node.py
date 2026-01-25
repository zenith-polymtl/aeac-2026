import rclpy
from rclpy.node import Node
from custom_interfaces.msg import AimError
from std_srvs.srv import SetBool
from geometry_msgs.msg import TwistStamped 
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
import sys, select, termios, tty
import time

# Configuration
FAKE_ERROR_MAGNITUDE = 1.0 
DRONE_SPEED = 1.0 
DRONE_ROT_SPEED = 0.5 
CLIMB_SPEED = 0.5 

msg = """
---------------------------
COCKPIT DE TEST PRO (HOVER AUTO)
---------------------------
VISÉE (GIMBAL):
   ^ v < >  : Maintenir pour bouger

PILOTAGE (DRONE):
   W / S    : Avancer / Reculer
   A / D    : Translation Gauche / Droite
   Q / E    : Rotation (Yaw) Gauche / Droite
   ESPACE   : Monter
   Z        : Descendre

COMMANDES:
   T : Arm+Takeoff (Attend 5s avant d'activer le frein auto)
   L : Lock | F : Follow
   CTRL-C : Quitter
---------------------------
"""

class PIDTester(Node):
    def __init__(self):
        super().__init__('pid_tester_keys')
        
        self.aim_pub = self.create_publisher(AimError, '/gimbal/target_error', 10)
        self.vel_pub = self.create_publisher(TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10)
        
        self.mode_client = self.create_client(SetBool, 'gimbal/lock_mode')
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        
        self.settings = termios.tcgetattr(sys.stdin)
        
        # --- Variables pour le Hover ---
        self.flight_started = False
        self.start_time = 0.0

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
            if key == '\x1b':
                key += sys.stdin.read(2)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def arm_and_guided(self):
        req_mode = SetMode.Request()
        req_mode.custom_mode = 'GUIDED'
        self.set_mode_client.call_async(req_mode)
        time.sleep(0.5)
        
        req_arm = CommandBool.Request()
        req_arm.value = True
        self.arming_client.call_async(req_arm)
        time.sleep(1.0)
        
        req_to = CommandTOL.Request()
        req_to.altitude = 5.0
        self.takeoff_client.call_async(req_to)
        
        # On note l'heure du décollage
        self.start_time = time.time()
        self.flight_started = True
        self.get_logger().info("Décollage ! Frein auto activé dans 5 secondes...")

    def send_velocity(self, x, y, z, yaw_speed):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = float(x)
        msg.twist.linear.y = float(y)
        msg.twist.linear.z = float(z)
        msg.twist.angular.z = float(yaw_speed) 
        self.vel_pub.publish(msg)

    def run(self):
        print(msg)
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0)
                
                key = self.get_key()
                k = key.lower()

                fake_vision = AimError()
                vx, vy, vz, vyaw = 0.0, 0.0, 0.0, 0.0
                
                user_input_active = False

                # --- GIMBAL ---
                if key == '\x1b[A': fake_vision.pitch_error = float(FAKE_ERROR_MAGNITUDE)
                elif key == '\x1b[B': fake_vision.pitch_error = -float(FAKE_ERROR_MAGNITUDE)
                elif key == '\x1b[C': fake_vision.yaw_error = float(FAKE_ERROR_MAGNITUDE)
                elif key == '\x1b[D': fake_vision.yaw_error = -float(FAKE_ERROR_MAGNITUDE)
                
                # --- DRONE ---
                elif k == 'w': 
                    vy = DRONE_SPEED
                    user_input_active = True
                elif k == 's': 
                    vy = -DRONE_SPEED
                    user_input_active = True
                elif k == 'a': 
                    vx = -DRONE_SPEED
                    user_input_active = True
                elif k == 'd': 
                    vx = DRONE_SPEED
                    user_input_active = True
                elif k == 'q': 
                    vyaw = DRONE_ROT_SPEED
                    user_input_active = True
                elif k == 'e': 
                    vyaw = -DRONE_ROT_SPEED
                    user_input_active = True
                elif k == ' ': 
                    vz = CLIMB_SPEED
                    user_input_active = True
                elif k == 'z': 
                    vz = -CLIMB_SPEED
                    user_input_active = True
                
                # --- COMMANDES ---
                elif k == 't': 
                    self.get_logger().info("Action: Arm + Takeoff")
                    self.arm_and_guided()
                elif k == 'l': self.call_mode_service(True)
                elif k == 'f': self.call_mode_service(False)
                elif k == '\x03': break 

                self.aim_pub.publish(fake_vision)
                
                # --- LOGIQUE DE SURPLACE ---
                if user_input_active:
                    self.send_velocity(vx, vy, vz, vyaw)
                
                elif self.flight_started:
                    if (time.time() - self.start_time) > 5.0:
                        self.send_velocity(0.0, 0.0, 0.0, 0.0)
                    else:
                        pass

        except Exception as e:
            self.get_logger().error(f"Erreur loop: {e}")

    def call_mode_service(self, val):
        req = SetBool.Request()
        req.data = val
        self.mode_client.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    node = PIDTester()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.send_velocity(0.0, 0.0, 0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()