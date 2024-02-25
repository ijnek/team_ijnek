// Copyright 2024 Kenji Brameld
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "localization/pose_resetter.hpp"
#include "pose_resetter_params.hpp"

namespace localization
{

PoseResetter::PoseResetter(const rclcpp::NodeOptions & options)
: rclcpp::Node{"pose_resetter", options}
{
  // Params
  // - player number
  // - transition poses (Provide sample param files for challenger league and champion league)
  auto param_listener = std::make_shared<pose_resetter::ParamListener>(get_node_parameters_interface());
  auto params = param_listener->get_params();

  // Subscription
  localization_transition_sub_ =
    this->create_subscription<ijnek_interfaces::msg::LocalizationTransition>(
    "transition", 10,
    std::bind(&PoseResetter::localization_transition_cb, this, std::placeholders::_1));

  // Service client
  set_poses_srv_client_ = this->create_client<ijnek_interfaces::srv::SetPoses>(
    "set_poses");
}

void PoseResetter::localization_transition_cb(
  const ijnek_interfaces::msg::LocalizationTransition::SharedPtr msg)
{
  (void) msg;
  // Call the service to reset the poses
  auto request = std::make_shared<ijnek_interfaces::srv::SetPoses::Request>();
  geometry_msgs::msg::PoseWithCovariance pose;
  pose.pose.position.x = 1.0;
  request->poses.push_back(pose);
  set_poses_srv_client_->async_send_request(request);
}

}  // namespace localization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(localization::PoseResetter)
