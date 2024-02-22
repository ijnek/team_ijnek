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

#include "gtest/gtest.h"
#include "localization/pose_resetter.hpp"

TEST(TestPoseResetter, TestResetPosesGetsCalled)
{
  rclcpp::init(0, nullptr);

  // Set up nodes
  auto pose_resetter_node = std::make_shared<localization::PoseResetter>();
  auto test_node = std::make_shared<rclcpp::Node>("test_node");

  // Set up service to check if set_poses gets called
  bool set_poses_called = false;
  auto srv = test_node->create_service<ijnek_interfaces::srv::SetPoses>(
    "set_poses",
    [&set_poses_called](const std::shared_ptr<ijnek_interfaces::srv::SetPoses::Request>,
    std::shared_ptr<ijnek_interfaces::srv::SetPoses::Response>) {
      set_poses_called = true;
    });

  // Publish a transition
  auto publisher = test_node->create_publisher<ijnek_interfaces::msg::LocalizationTransition>(
    "transition", 1);
  ijnek_interfaces::msg::LocalizationTransition msg;
  msg.type = ijnek_interfaces::msg::LocalizationTransition::UNPENALIZED_POSE;
  publisher->publish(msg);

  rclcpp::spin_some(pose_resetter_node);
  rclcpp::spin_some(test_node);

  EXPECT_TRUE(set_poses_called);

  rclcpp::shutdown();
}
