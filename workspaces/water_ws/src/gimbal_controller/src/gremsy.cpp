#include "gimbal_controller/gremsy.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> 
#include <cmath>

uint64_t get_time_usec()
{
    static auto start_time = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::microseconds>(now - start_time).count();
}

void Gremsy::declareParameters()
{
    this->declare_parameter("baudrate", 115200);
    this->declare_parameter("gimbal_mode", 2); 
}

Gremsy::Gremsy(const rclcpp::NodeOptions & options, const std::string & com_port)
: Node("ros2_gremsy", options), com_port_(com_port)
{
    declareParameters();
    baud_rate_ = this->get_parameter("baudrate").as_int();
    gimbal_mode_ = this->get_parameter("gimbal_mode").as_int();

    serial_port_ = new Serial_Port(com_port_.c_str(), baud_rate_);
    gimbal_interface_ = new Gimbal_Interface(serial_port_, 1, 154, 
                                             Gimbal_Interface::MAVLINK_GIMBAL_V2, 
                                             MAVLINK_COMM_0);

    serial_port_->start();
    gimbal_interface_->start();

    // publisher
    orientation_pub_ = this->create_publisher<geometry_msgs::msg::Quaternion>("/gimbal/orientation", 10);

    orientation_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50), 
        std::bind(&Gremsy::publish_orientation_callback, this)
    );

    // subscriber
    this->aiming_subscriber_ = this->create_subscription<custom_interfaces::msg::AimError>(
        "gimbal/target_error", 10,
        std::bind(&Gremsy::aimingCallback, this, std::placeholders::_1));
    
    // service
    this->enable_lock_mode_service_ = this->create_service<std_srvs::srv::SetBool>("gimbal/lock_mode",
        std::bind(&Gremsy::enableLockModeCallback, this, std::placeholders::_1, std::placeholders::_2));

    // --- STARTUP LOGIC ---
    if (gimbal_interface_->get_gimbal_status().state == Gimbal_Interface::GIMBAL_STATE_OFF) {
        RCLCPP_INFO(this->get_logger(), "Gimbal seems OFF, sending ON command...");
        gimbal_interface_->set_gimbal_motor(Gimbal_Interface::TURN_ON);
    }

    int retries = 0;

    while (gimbal_interface_->get_gimbal_status().state < Gimbal_Interface::GIMBAL_STATE_ON && retries < 20) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for gimbal init...");
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        retries++;
    }
  
    // Set Mode
    if (gimbal_mode_ == 1) { // LOCK
        gimbal_interface_->set_gimbal_lock_mode_sync();
    } else {
        gimbal_interface_->set_gimbal_follow_mode_sync();
    }
}

Gremsy::~Gremsy()
{
  serial_port_->stop();
  delete gimbal_interface_;
  delete serial_port_;
}

void Gremsy::publish_orientation_callback()
{
    auto attitude = gimbal_interface_->get_gimbal_attitude();

    float roll_deg = attitude.eu_angle_forward.roll;
    float pitch_deg = attitude.eu_angle_forward.pitch;
    float yaw_deg = attitude.eu_angle_forward.yaw;

    double roll_rad = roll_deg * (M_PI / 180.0);
    double pitch_rad = pitch_deg * (M_PI / 180.0);
    double yaw_rad = yaw_deg * (M_PI / 180.0);

    tf2::Quaternion q;
    q.setRPY(roll_rad, pitch_rad, yaw_rad);

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

void Gremsy::enableLockModeCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, 
                                    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    reset_pid_memory();

    gimbal_interface_->set_gimbal_rotation_rate_sync(0.0f, 0.0f, 0.0f);

    int new_mode = request->data ? 1 : 2; // 1=LOCK, 2=FOLLOW
    
    if (new_mode == gimbal_mode_){
        response->success = true;
        response->message = "Already in mode.";
    } else {
        gimbal_mode_ = new_mode;
        
        if (gimbal_mode_ == 1) {
            gimbal_interface_->set_gimbal_lock_mode_sync();
        } else {
            gimbal_interface_->set_gimbal_follow_mode_sync();
        }

        response->success = true;
        response->message = "Mode changed.";
    }
}

void Gremsy::aimingCallback(const custom_interfaces::msg::AimError::SharedPtr msg) {
    if (gimbal_mode_ != 1){ // 1 = LOCK
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Gimbal must be in LOCK mode.");
        return;
    }
    update_gun_aiming(msg->pitch_error, msg->yaw_error);
}

float Gremsy::compute_pid_output(float error, float &integral, float &prev_error, float dt) {
    const float Kp = 2.0f;
    const float Ki = 0.1f;
    const float Kd = 0.05f;
    
    float P = error * Kp;
    integral += error * dt;
    if (integral > 10.0f) integral = 10.0f;
    if (integral < -10.0f) integral = -10.0f;
    
    float I = integral * Ki;
    float D = 0.0f;
    if (dt > 0.0001f) D = ((error - prev_error) / dt) * Kd;

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

  gimbal_interface_->set_gimbal_rotation_rate_sync(cmd_pitch, 0.0f, cmd_yaw);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto gremsy = std::make_shared<Gremsy>(options, "/dev/ttyUSB0");
    rclcpp::spin(gremsy);
    rclcpp::shutdown();
    return 0;
}