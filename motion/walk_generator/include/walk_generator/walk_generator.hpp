// Copyright 2021 Kenji Brameld
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

#ifndef WALK_GENERATOR__WALK_GENERATOR_HPP_
#define WALK_GENERATOR__WALK_GENERATOR_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nao_sensor_msgs/msg/joint_positions.hpp"
#include "motion_interfaces/msg/ik_command.hpp"
#include "motion_interfaces/action/walk.hpp"


class WalkGenerator : public rclcpp::Node
{
public:
  using WalkGoal = motion_interfaces::action::Walk::Goal;
  using WalkGoalHandle = rclcpp_action::ServerGoalHandle<motion_interfaces::action::Walk>;
  WalkGenerator();

private:
  enum WalkOption
  {
    STAND = 0,      // with knees straight
    STANDUP = 1,      // process of moving from WALK crouch to STAND
    CROUCH = 2,      // process of transitioning from STAND to WALK
    READY = 3,      // crouch still ready to walk
    WALK = 4,
  };

  WalkOption walkOption = STAND;
  float hiph;
  float dt = 0.02;    // make sure to change this for real robot
  float t = 0.0;
  float forwardL0, forwardR0, leftL0, leftR0, turnRL0;
  bool isLeftPhase = false;
  bool weightHasShifted = true;

  motion_interfaces::msg::IKCommand generate_ik_command(
    nao_sensor_msgs::msg::JointPositions & sensor_joints);

  void handleAccepted(
    const std::shared_ptr<WalkGoalHandle> goal_handle);

  rclcpp::Subscription<nao_sensor_msgs::msg::JointPositions>::SharedPtr sub_joint_states;
  rclcpp::Publisher<motion_interfaces::msg::IKCommand>::SharedPtr pub_ik_command;
  rclcpp_action::Server<motion_interfaces::action::Walk>::SharedPtr action_server_;

  std::shared_ptr<WalkGoalHandle> walk_goal_handle_;
  std::shared_ptr<motion_interfaces::action::Walk::Feedback> walk_feedback_;
  std::shared_ptr<motion_interfaces::action::Walk::Result> walk_result_;
  std::shared_ptr<geometry_msgs::msg::Twist> target_;
};

#endif  // WALK_GENERATOR__WALK_GENERATOR_HPP_
