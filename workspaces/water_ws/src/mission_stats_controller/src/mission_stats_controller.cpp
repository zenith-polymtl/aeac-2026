#include "mission_stats_controller/mission_stats_controller.hpp"

MissionStatsController::MissionStatsController() : Node("mission_stats_controller")
{
	RCLCPP_INFO(get_logger(), "Mission stats node started!");

	// Publishers Initalisation
	state_publisher_ = this->create_publisher<WaterState>("change_state", 10);

	// Subscribers Initalisation
	ask_state_change_subscriber_ = create_subscription<WaterState>(
		"ask_state_change", 10, std::bind(&MissionStatsController::handleStateChange, this, std::placeholders::_1));
}

void MissionStatsController::handleStateChange(const WaterState state)
{
	// Print for debug/information
	RCLCPP_INFO(get_logger(), "Received change to  %d", static_cast<int>(state.mode));

	// This switch case handle how the diffrent state demande should be handled.
	// The cases represent the state demanded for a switch.
	// The switch case should stay clean, and function should be used for more complex logic.
	// E.g: If we are in the IDDLE and want to pass to the ACTIVE state, the different values should be:
	//      - current_state = IDDLE
	//      - state.mode = ACTIVE
	// Note: The sate are stored as uint8, so the value of IDDLE is actualy 0.
	switch (state.mode)
	{
	case WaterState::IDLE:
		HandleSwitchToIddle();
		break;
	case WaterState::ACTIVE:
		HandleSwitchToActive();
		break;
	case WaterState::ERROR:
		enterErrorState();
		break;
	default:
		RCLCPP_WARN(get_logger(), "State transition not covered: %d", static_cast<int>(state.mode));
		break;
	}
}

void MissionStatsController::HandleSwitchToIddle()
{
	RCLCPP_INFO(get_logger(), "Received change to IDLE");
	if (current_state_ == WaterState::ACTIVE)
	{
		RCLCPP_INFO(get_logger(), "Deactivating the drone");
		current_state_ = WaterState::IDLE;
	}
}

void MissionStatsController::HandleSwitchToActive()
{
	RCLCPP_INFO(get_logger(), "Received change to ACTIVE");
	if (current_state_ == WaterState::IDLE)
	{
		RCLCPP_INFO(get_logger(), "Activating the drone");
		current_state_ = WaterState::ACTIVE;
	}
}

void MissionStatsController::enterErrorState()
{
	RCLCPP_INFO(get_logger(), "Received change to ERROR");
	current_state_ = WaterState::ERROR;
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MissionStatsController>());
	rclcpp::shutdown();
	return 0;
}