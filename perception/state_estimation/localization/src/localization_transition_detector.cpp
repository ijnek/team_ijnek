#include "localization/localization_transition_detector.hpp"

namespace localization
{

LocalizationTransitionDetector::LocalizationTransitionDetector(const rclcpp::NodeOptions & options)
: rclcpp::Node{"localization_transition_detector", options}
{
  // Params
  // - team number
  // - player number
  // - handle referee mistakes

  // Subscription
  // - gc/data (game_controller_spl_interfaces::msg::RCGCD15)
  // - pickedup (some info about being picked up)

  // Publisher
  // - localization/transition (ijnek_interfaces::msg::LocalizationTransition)
}

void rcgcd_callback(const game_controller_spl_interfaces::msg::RCGCD15::SharedPtr msg)
{

}

}  // namespace localization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(localization::LocalizationTransitionDetector)
