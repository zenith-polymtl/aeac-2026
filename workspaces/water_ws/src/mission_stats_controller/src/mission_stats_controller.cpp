#include "mission_stats_controller/mission_stats_controller.hpp"

MissionStatsController::MissionStatsController() : Node("mission_stats_controller")
{
  RCLCPP_INFO(get_logger(), "Mission stats node started!");

  state_publisher_ = this->create_publisher<custom_interfaces::msg::WaterState>("change_state", 10);

  ask_state_change_subscriber_ = create_subscription<custom_interfaces::msg::WaterState>(
		"ask_state_change", 10, std::bind(&MissionStatsController::handleStateChange, this, std::placeholders::_1));

}

void MissionStatsController::handleStateChange(const custom_interfaces::msg::WaterState state)
{
  switch (state.mode)
  {
  case custom_interfaces::msg::WaterState::IDLE:
    RCLCPP_INFO(get_logger(), "Received change to IDLE");

    break;

  default:
    RCLCPP_WARN(get_logger(), "Unknown water state: %d", static_cast<int>(state.mode));
    break;
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionStatsController>());
  rclcpp::shutdown();
  return 0;
}