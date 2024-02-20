#pragma once

#include "rclcpp/rclcpp.hpp"
#include "ijnek_interfaces/msg/localization_transition.hpp"
#include "ijnek_interfaces/srv/set_poses.hpp"

namespace localization
{

class PoseResetter : public rclcpp::Node
{
public:
  explicit PoseResetter(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});
private:
  void localization_transition_cb(const ijnek_interfaces::msg::LocalizationTransition::SharedPtr msg);
  rclcpp::Client<ijnek_interfaces::srv::SetPoses>::SharedPtr set_poses_srv_client_;
};

}  // namespace localization
