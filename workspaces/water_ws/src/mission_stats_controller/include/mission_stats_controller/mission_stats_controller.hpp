#ifndef MISSION_STATS_CONTROLLER_HPP_
#define MISSION_STATS_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include "custom_interfaces/msg/water_state.hpp"
#include <vector>
#include <string>
#include <map>
#include <functional>

using custom_interfaces::msg::WaterState;

class MissionStatsController : public rclcpp::Node
{
public:
  MissionStatsController();

private:
  uint8_t current_state_ = WaterState::IDLE;

  // void handleAskStateChange(const custom_interfaces::msg::WaterState msg);

  void handleStateChange(const custom_interfaces::msg::WaterState msg);

  // Publisher
  rclcpp::Publisher<custom_interfaces::msg::WaterState>::SharedPtr state_publisher_;
  
  // Subscriber
  rclcpp::Subscription<custom_interfaces::msg::WaterState>::SharedPtr ask_state_change_subscriber_;
};

#endif // MISSION_STATS_CONTROLLER_HPP_