#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <thread>

// ROS Includes
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "tf2/LinearMath/Quaternion.h"

// Custom Interfaces
#include "custom_interfaces/msg/aim_error.hpp"

// SDK Includes
#include "gimbal_interface.h"
#include "serial_port.h"

class Gremsy : public rclcpp::Node
{
public:
    Gremsy(const rclcpp::NodeOptions & options, const std::string & com_port = "/dev/ttyUSB0");
    ~Gremsy();

private:
    void declareParameters();
    void publish_orientation_callback();
    
    // Callbacks
    void aimingCallback(const custom_interfaces::msg::AimError::SharedPtr msg);
    void enableLockModeCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    void update_gun_aiming(float error_pitch, float error_yaw);
    float compute_pid_output(float error, float &integral, float &prev_error, float dt);
    void reset_pid_memory();

    // SDK Objects
    Serial_Port *serial_port_;
    Gimbal_Interface *gimbal_interface_;
    
    std::string com_port_;
    int baud_rate_;
    
    // 1 = LOCK, 2 = FOLLOW
    uint8_t gimbal_mode_; 

    // PID Variables
    float pitch_integral_ = 0.0f;
    float pitch_prev_error_ = 0.0f;
    float yaw_integral_ = 0.0f;
    float yaw_prev_error_ = 0.0f;
    uint64_t last_update_time_us_ = 0;

    // ROS Topics
    rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr orientation_pub_;
    rclcpp::TimerBase::SharedPtr orientation_timer_;
    rclcpp::Subscription<custom_interfaces::msg::AimError>::SharedPtr aiming_subscriber_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_lock_mode_service_;
};