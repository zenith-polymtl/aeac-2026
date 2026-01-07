#include <rclcpp/rclcpp.hpp>
#include <string>
#include <../../gSDK/src/gimbal_interface.h>
#include <../../gSDK/src/serial_port.h>
#include <memory>
#include "std_srvs/srv/set_bool.hpp"
// #include "custom_interfaces/msg/aim_error.hpp"

class Gremsy : public rclcpp::Node
{
public:
    
    Gremsy(const rclcpp::NodeOptions &options, const std::string &serial_port = "/dev/ttyUSB0");
    ~Gremsy();

private:
    void declareParameters();
    std::string com_port_;
    Serial_Port * serial_port_;
    Gimbal_Interface * gimbal_interface_;

    int baud_rate_;

    control_gimbal_mode_t gimbal_mode_;

    control_gimbal_axis_input_mode_t tilt_axis_input_mode_;

    control_gimbal_axis_input_mode_t roll_axis_input_mode_;

    control_gimbal_axis_input_mode_t pan_axis_input_mode_;

    bool tilt_axis_stabilize_;

    bool roll_axis_stabilize_;

    bool pan_axis_stabilize_;

    rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr orientation_pub_;
    rclcpp::TimerBase::SharedPtr orientation_timer_;
    void publish_orientation_callback();

    rclcpp::Subscription<int>::SharedPtr aiming_subscriber_;

    void aimingCallback(float msg);

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_lock_mode_service_;
    void enableLockModeCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, const std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    
    // PID functions
    float compute_pid_output(float error, float &integral, float &prev_error, float dt);    

    void update_gun_aiming(float error_pitch, float error_yaw);

    void reset_pid_memory();

    float pitch_integral_ = 0.0f;
    float pitch_prev_error_ = 0.0f;

    float yaw_integral_ = 0.0f;
    float yaw_prev_error_ = 0.0f;

    uint64_t last_update_time_us_ = 0;
};