#pragma once

#include "rclcpp/rclcpp.hpp"
#include "game_controller_spl_interfaces/msg/rcgcd15.hpp"
#include "ijnek_interfaces/msg/localization_transition.hpp"

namespace localization
{

class LocalizationTransitionDetector : public rclcpp::Node
{
public:
  explicit LocalizationTransitionDetector(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

private:
  void rcgcd_callback(const game_controller_spl_interfaces::msg::RCGCD15::SharedPtr msg);

  // uint8_t prevCompetitionType;
  // uint8_t prevGameState;
  // uint8_t prevGamePhase;
  // uint8_t prevPenalty;
  // bool prevPickedup;

  // Timer penalisedTimer;
  // Timer refPickupTimer;
  // Timer unpenalisedTimer;
  // bool pickedUpDuringPenalised;
};

}  // namespace localization
