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
#include "nao_ik_interfaces/msg/ik_command.hpp"
#include "walk/step_variable.hpp"
#include "walk/step_calculator.hpp"


class Walk
{
public:
  Walk(
    std::function<void(void)> notifyGoalAchieved,
    std::function<void(nao_ik_interfaces::msg::IKCommand)> sendIKCommand);
  void setParams(
    float maxForward,  // max forward velocity (m/s)
    float maxLeft,  // max side velocity (m/s)
    float maxTurn,  // max turn velocity (rad/s)
    float speedMultiplier,  // how much to multiple speed by (0.0 - 1.0)
    float footLiftAmp,  // how much to raise foot when it is highest (m)
    float period,  // time taken for one step, (s)
    float ankleZ,  // z coordinate of ankle from hip when standing (m)
    float maxForwardChange,  // how much forward can change in one step (m/s)
    float maxLeftChange,  // how much left can change in one step (m/s)
    float maxTurnChange);  // how much turn can change in one step (rad/s)
  void notifyJoints(nao_sensor_msgs::msg::JointPositions & sensor_joints);
  void abort();
  void walk(const geometry_msgs::msg::Twist & target);
  void crouch();

private:
  std::function<void(void)> notifyGoalAchieved;
  std::function<void(nao_ik_interfaces::msg::IKCommand)> sendIKCommand;

  enum WalkOption
  {
    CROUCH = 1,      // crouch still ready to walk
    WALK = 2,
  };

  WalkOption walkOption = CROUCH;
  geometry_msgs::msg::Twist currTwist;
  StepVariable currStep;

  StepCalculator stepCalculator;

  float t = 0.0;
  float forwardL0, forwardR0, leftL0, leftR0, turnRL0;
  bool isLeftPhase = false;
  bool weightHasShifted = true;

  bool firstMsg = true;
  rclcpp::Time prev_time_;

  bool duringWalk = false;  // whether this action is active or not
  WalkOption targetWalkOption = CROUCH;  // target walk option to aim for
  geometry_msgs::msg::Twist target;  // target twist to aim for, if walking

  rclcpp::Logger logger;

  float period;
  float ankleZ;
};

#endif  // WALK__WALK_HPP_
