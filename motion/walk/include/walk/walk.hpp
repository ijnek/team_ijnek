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

#ifndef WALK__WALK_HPP_
#define WALK__WALK_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nao_sensor_msgs/msg/joint_positions.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "biped_interfaces/msg/ankle_poses.hpp"


class Walk : public rclcpp::Node
{
public:
  Walk();

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

  biped_interfaces::msg::AnklePoses generate_ankle_poses(
    nao_sensor_msgs::msg::JointPositions & sensor_joints);

  rclcpp::Subscription<nao_sensor_msgs::msg::JointPositions>::SharedPtr sub_joint_states;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist;
  rclcpp::Publisher<biped_interfaces::msg::AnklePoses>::SharedPtr pub_ankle_poses;

  geometry_msgs::msg::Twist twist;
};

#endif  // WALK__WALK_HPP_
