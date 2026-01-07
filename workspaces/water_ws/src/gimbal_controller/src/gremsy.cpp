#include "gimbal_controller/gremsy.hpp"
#include <thread>
#include <chrono>
#include "gremsy.hpp"

void Gremsy::declareParameters()
{

    //0: MIO, 1: S1, 2: T3V3, 3: T7
    //this->declare_parameter("device_id", 0);

    this->declare_parameter("baudrate", 115200);

    // LOCK_MODE = 1, FOLLOW_MODE = 2
    this->declare_parameter("gimbal_mode", 2);

    // input mode: 0: CTRL_ANGLE_BODY_FRAME, 1: CTRL_ANGULAR_RATE, 2:CTRL_ANGLE_ABSOLUTE_FRAME
    // Note: Only Gimbal Pixy and T3V3 support CTRL_ANGLE_BODY_FRAME mode with pitch and yaw axis.
    this->declare_parameter("tilt_axis_input_mode", 1);

    this->declare_parameter("tilt_axis_stabilize", true);

    this->declare_parameter("roll_axis_input_mode", 1);

    this->declare_parameter("roll_axis_stabilize", true);

    this->declare_parameter("pan_axis_input_mode", 1);

    this->declare_parameter("pan_axis_stabilize", true);

    this->declare_parameter("lock_yaw_to_vehicle", true);

}

inline control_gimbal_axis_input_mode_t convertIntToAxisInputMode(int mode)
{
  switch (mode) {
    case 0: return CTRL_ANGLE_BODY_FRAME;
    case 1: return CTRL_ANGULAR_RATE;
    case 2: return CTRL_ANGLE_ABSOLUTE_FRAME;
    default:
      return CTRL_ANGLE_ABSOLUTE_FRAME;
  }
}

Gremsy::Gremsy(const rclcpp::NodeOptions & options, const std::string & com_port = "/dev/ttyUSB0")
: Node("ros2_gremsy", options), com_port_(com_port)
{

    declareParameters();
    //device_id_ = gremsy_model_t(this->get_parameter("device_id").as_int());
    baud_rate_ = this->get_parameter("baudrate").as_int();
    int mode_value = this->get_parameter("gimbal_mode").as_int();
    gimbal_mode_ = mode_value == 1 ? control_gimbal_mode_t::LOCK_MODE : control_gimbal_mode_t::FOLLOW_MODE;

    tilt_axis_input_mode_ = convertIntToAxisInputMode(this->get_parameter("tilt_axis_input_mode").as_int());
    tilt_axis_stabilize_ = this->get_parameter("tilt_axis_stabilize").as_bool();

    roll_axis_input_mode_ = convertIntToAxisInputMode(this->get_parameter("roll_axis_input_mode").as_int());
    roll_axis_stabilize_ = this->get_parameter("roll_axis_stabilize").as_bool();

    pan_axis_input_mode_ = convertIntToAxisInputMode(this->get_parameter("pan_axis_input_mode").as_int());
    pan_axis_stabilize_ = this->get_parameter("pan_axis_stabilize").as_bool();

    // Define SDK objects
    serial_port_ = new Serial_Port(com_port_.c_str(), baud_rate_);
    gimbal_interface_ = new Gimbal_Interface(serial_port_);

    serial_port_->start();
    gimbal_interface_->start();

    // publishers
    orientation_pub_ = this->create_publisher<geometry_msgs::msg::Quaternion>("/gimbal/orientation", 10);

    orientation_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50), 
        std::bind(&Gremsy::publish_orientation_callback, this)
    );

    // Initialize subscribers
    this->aiming_subscriber_ =
        this->create_subscription<int>(
        "gimbal/target_error", 10,
        std::bind(&Gremsy::aimingCallback, this, std::placeholders::_1));

    // Create services
    this->enable_lock_mode_service_ =
        this->create_service<std_srvs::srv::SetBool>("gimbal/lock_mode",
        std::bind(&Gremsy::enableLockModeCallback, this, std::placeholders::_1, std::placeholders::_2));


    if (gimbal_interface_->get_gimbal_status().mode == GIMBAL_STATE_OFF) {
        RCLCPP_INFO(this->get_logger(), "Gimbal is off, turning it on");
        gimbal_interface_->set_gimbal_motor_mode(TURN_ON);
    }

    while (gimbal_interface_->get_gimbal_status().mode < GIMBAL_STATE_ON) {
        RCLCPP_INFO(this->get_logger(), "Waiting for gimbal to turn on");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  
  gimbal_interface_->set_gimbal_mode(gimbal_mode_);

  // Set modes for each axis
  control_gimbal_axis_mode_t tilt_axis_mode, roll_axis_mode, pan_axis_mode;
  tilt_axis_mode.input_mode = tilt_axis_input_mode_;
  tilt_axis_mode.stabilize = tilt_axis_stabilize_;

  roll_axis_mode.input_mode = roll_axis_input_mode_;
  roll_axis_mode.stabilize = roll_axis_stabilize_;

  pan_axis_mode.input_mode = pan_axis_input_mode_;
  pan_axis_mode.stabilize = pan_axis_stabilize_;

  gimbal_interface_->set_gimbal_axes_mode(tilt_axis_mode, roll_axis_mode, pan_axis_mode);
}

Gremsy::~Gremsy()
{
  serial_port_->stop();
  delete gimbal_interface_;
  delete serial_port_;
}

void Gremsy::publish_orientation_callback()
{
    mavlink_mount_orientation_t m = gimbal_interface_->get_gimbal_mount_orientation();

    double roll_rad = m.roll * (M_PI / 180.0);
    double pitch_rad = m.pitch * (M_PI / 180.0);
    double yaw_local_rad = m.yaw * (M_PI / 180.0);

    tf2::Quaternion q;
    q.setRPY(roll_rad, pitch_rad, yaw_local_rad);

    geometry_msgs::msg::Quaternion msg;
    msg.x = q.x();
    msg.y = q.y();
    msg.z = q.z();
    msg.w = q.w();

    orientation_pub_->publish(msg);
}

void Gremsy::reset_pid_memory() {
  pitch_integral_ = 0.0f;
  pitch_prev_error_ = 0.0f;

  yaw_integral_ = 0.0f;
  yaw_prev_error_ = 0.0f;

  last_update_time_us_ = get_time_usec();
}

void Gremsy::enableLockModeCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, const std::shared_ptr<std_srvs::srv::SetBool::Response> response){
    
    reset_pid_memory();
    gimbal_interface_->set_gimbal_move(0, 0, 0);

    control_gimbal_mode_t new_mode = request->data ? control_gimbal_mode_t::LOCK_MODE : control_gimbal_mode_t::FOLLOW_MODE;
    
    if (new_mode == gimbal_mode_){
        response->success = true;
        response->message = "Gimbal is already in requested mode.";
    } else {
        gimbal_mode_ = new_mode;
        gimbal_interface_->set_gimbal_mode(gimbal_mode_);

        response->success = true;
        response->message = "Gimbal mode successfully changed.";
    }
}

void Gremsy::aimingCallback(float msg) {

    if (gimbal_mode_ != control_gimbal_mode_t::LOCK_MODE){
        RCLCPP_WARN(this->get_logger(), "gimball must be in LOCK mode in oder to aim the target.");
        return;
    }
    float yaw_error = msg;
    float pitch_error = msg;

    update_gun_aiming(pitch_error, yaw_error);
}

float Gremsy::compute_pid_output(float error, float &integral, float &prev_error, float dt) {
    
    const float Kp = 2.0f;
    const float Ki = 0.1f;
    const float Kd = 0.05f;
    const float MAX_INTEGRAL = 10.0f;

    // 1. Proportionnel
    float P = error * Kp;

    // 2. Intégral
    integral += error * dt;

    if (integral > MAX_INTEGRAL) integral = MAX_INTEGRAL;
    if (integral < -MAX_INTEGRAL) integral = -MAX_INTEGRAL;
    
    float I = integral * Ki;

    // 3. Dérivé
    float D = 0.0f;
    if (dt > 0.0001f) {
        D = ((error - prev_error) / dt) * Kd;
    }

    prev_error = error;

    return P + I + D;
}

void Gremsy::update_gun_aiming(float error_pitch, float error_yaw) {

  uint64_t current_time = get_time_usec();
  float dt = (current_time - last_update_time_us_) / 1000000.0f;
  last_update_time_us_ = current_time;

  if (dt <= 0.0f || dt > 0.1f) dt = 0.01f;

  float cmd_pitch = compute_pid_output(error_pitch, pitch_integral_, pitch_prev_error_, dt);
  float cmd_yaw   = compute_pid_output(error_yaw, yaw_integral_, yaw_prev_error_, dt);

  float max_speed = 30.0f;
  if (cmd_pitch > max_speed) cmd_pitch = max_speed;
  if (cmd_pitch < -max_speed) cmd_pitch = -max_speed;
  if (cmd_yaw > max_speed) cmd_yaw = max_speed;
  if (cmd_yaw < -max_speed) cmd_yaw = -max_speed;

  gimbal_interface_->set_gimbal_move(cmd_pitch, 0.0f, cmd_yaw);
}




int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    std::cout << "ROS2 Gremsy node." << std::endl;
    rclcpp::executors::MultiThreadedExecutor exec;
    rclcpp::NodeOptions options;
    auto gremsy = std::make_shared<Gremsy>(options, "/dev/ttyUSB0");
    exec.add_node(gremsy);
    exec.spin();

    rclcpp::shutdown();

    return 0;
}