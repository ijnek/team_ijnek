#include "localization/pose_resetter.hpp"

namespace localization
{

PoseResetter::PoseResetter(const rclcpp::NodeOptions & options)
: rclcpp::Node{"pose_resetter", options}
{
  // Params
  // - player number
  // - transition poses (Provide sample param files for challenger league and champion league)

  // Subscription
  // - localization/transition (ijnek_interfaces::msg::LocalizationTransition)

  // Create service client
  // - localization/reset_poses (ijnek_interfaces::srv::SetPoses)
  // Wait for the service server to be available...? (This might be a bad idea to wait in constructor)
}

void PoseResetter::localization_transition_cb(
  const ijnek_interfaces::msg::LocalizationTransition::SharedPtr msg)
{
}


}  // namespace localization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(localization::PoseResetter)
