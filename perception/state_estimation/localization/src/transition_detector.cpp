#include "localization/transition_detector.hpp"

namespace localization
{

TransitionDetector::TransitionDetector(const rclcpp::NodeOptions & options)
: rclcpp::Node{"transition_detector", options}
{
  // Params
  // - team number
  // - player number
  // - handle referee mistakes

  // Subscription
  // - pickedup (some info about being picked up)
  rcgcd_sub_ = this->create_subscription<game_controller_spl_interfaces::msg::RCGCD15>(
    "gc/data", 10,
    std::bind(&TransitionDetector::rcgcd_callback, this, std::placeholders::_1));

  // Publisher
  localization_transition_pub_ =
    this->create_publisher<ijnek_interfaces::msg::LocalizationTransition>(
    "transition", 10);
}

void TransitionDetector::rcgcd_callback(
  const game_controller_spl_interfaces::msg::RCGCD15::SharedPtr msg)
{
  if (msg->competition_type != prevCompetitionType) {
    ijnek_interfaces::msg::LocalizationTransition transition;
    transition.type = 1;
    localization_transition_pub_->publish(transition);
  }

  prevCompetitionType = msg->competition_type;
}

}  // namespace localization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(localization::TransitionDetector)
