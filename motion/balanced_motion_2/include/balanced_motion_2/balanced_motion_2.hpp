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
#include "nao_interfaces/msg/joints.hpp"
#include "balanced_motion_2/motion_transitioner.hpp"
#include "balanced_motion_2/walk.hpp"
#include "balanced_motion_2/stand.hpp"
#include "balanced_motion_2/getup.hpp"
#include "balanced_motion_2/dive.hpp"
#include "motion_msgs/msg/action.hpp"

class BalancedMotion2 : public rclcpp::Node
{
public:
  BalancedMotion2();

private:
  nao_interfaces::msg::Joints make_joints(nao_interfaces::msg::Joints & sensor_joints);
  State evaluate_current_state();
  State evaluate_aim(motion_msgs::msg::Action action);

  rclcpp::Subscription<nao_interfaces::msg::Joints>::SharedPtr sub_joint_states;
  rclcpp::Subscription<motion_msgs::msg::Action>::SharedPtr sub_action;
  rclcpp::Publisher<nao_interfaces::msg::Joints>::SharedPtr pub_joints;

  MotionTransitioner transitioner;
  Walk walk;
  Stand stand;
  Dive dive;
  Getup getup;

  Motion* currentMotion;
  State currentMotionAimState;

  State aim;
};

#endif  // WALK_GENERATOR__WALK_GENERATOR_HPP_
