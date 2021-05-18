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

#ifndef WALK_GENERATOR__INVERSE_KINEMATICS_HPP_
#define WALK_GENERATOR__INVERSE_KINEMATICS_HPP_

#include "rclcpp/rclcpp.hpp"
#include "motion_msgs/msg/ik_command.hpp"
#include "nao_interfaces/msg/joints.hpp"
#include "geometry_msgs/msg/point.hpp"

// Use for iterative inverse kinematics for turning (see documentation BH 2010)
struct Hpr
{
  float Hp;
  float Hr;
};

class InverseKinematics : public rclcpp::Node
{
public:
  InverseKinematics();

private:
  nao_interfaces::msg::Joints calculate_joints(motion_msgs::msg::IKCommand & ik_command);
  geometry_msgs::msg::Point mf2b(
    float Hyp, float Hp, float Hr, float Kp, float Ap,
    float Ar, float xf, float yf, float zf);
  Hpr hipAngles(
    float Hyp, float Hp, float Hr, float Kp, float Ap,
    float Ar, float xf, float yf, float zf, geometry_msgs::msg::Point e);

  rclcpp::Subscription<motion_msgs::msg::IKCommand>::SharedPtr sub_ik_command;
  rclcpp::Publisher<nao_interfaces::msg::Joints>::SharedPtr pub_joints;
};

#endif  // WALK_GENERATOR__INVERSE_KINEMATICS_HPP_
