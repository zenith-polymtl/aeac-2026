#ifndef MISSION_STATS_CONTROLLER_HPP_
#define MISSION_STATS_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include "custom_interfaces/msg/water_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include <vector>
#include <string>
#include <map>
#include <functional>

class MissionStatsController : public rclcpp::Node
{
public:
	MissionStatsController();

private:
	// This line make is possible to use water state and use it as simply WaterState
	using WaterState = custom_interfaces::msg::WaterState;

	// void handleAskStateChange(const custom_interfaces::msg::WaterState msg);

	/// @brief This function handle the transition between state
	/// @param state This is the state that we should change to
	void handleStateChange(const WaterState state);

	/// @brief This function handle when the Error state occure
	void enterErrorState();

	void HandleSwitchToIddle();

	void HandleSwitchToActive();

	void HandleIncrementState(const std_msgs::msg::Bool& msg);

	/// @brief This variable holds the current state of the mission. Initale state is IDLE
	uint8_t current_state_ = WaterState::IDLE;

	// Publishers
	rclcpp::Publisher<WaterState>::SharedPtr state_publisher_;

	// Subscribers
	rclcpp::Subscription<WaterState>::SharedPtr ask_state_change_subscriber_;
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr increment_state_subscriber_;
};

#endif // MISSION_STATS_CONTROLLER_HPP_