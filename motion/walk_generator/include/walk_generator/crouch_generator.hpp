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

#ifndef CROUCH_GENERATOR__CROUCH_GENERATOR_HPP_
#define CROUCH_GENERATOR__CROUCH_GENERATOR_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nao_interfaces/msg/joints.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "motion_msgs/msg/ik_command.hpp"


class CrouchGenerator : public rclcpp::Node
{
public:
  CrouchGenerator();

private:
  enum WalkOption
  {
    STAND = 0,      // with knees straight
    STANDUP = 1,      // process of moving from CROUCH crouch to STAND
    CROUCH = 2,      // process of transitioning from STAND to CROUCH
    READY = 3,      // crouch still ready to walk
  };

  WalkOption walkOption = STAND;
  float hiph;
  float dt = 0.02;    // make sure to change this for real robot
  float t = 0.0;

  motion_msgs::msg::IKCommand generate_ik_command(nao_interfaces::msg::Joints & sensor_joints);

  rclcpp::Subscription<nao_interfaces::msg::Joints>::SharedPtr sub_joint_states;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist;
  rclcpp::Publisher<motion_msgs::msg::IKCommand>::SharedPtr pub_ik_command;

  geometry_msgs::msg::Twist twist;
};

#endif  // CROUCH_GENERATOR__CROUCH_GENERATOR_HPP_
