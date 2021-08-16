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

#ifndef KICK__KICK_HPP_
#define KICK__KICK_HPP_

#include <functional>
#include "motion_interfaces/msg/kick.hpp"
#include "motion_interfaces/msg/ik_command.hpp"
#include "nao_sensor_msgs/msg/joint_positions.hpp"

class Kick
{
public:
  Kick(
    std::function<void(void)> notifyKickDone,
    std::function<void(motion_interfaces::msg::IKCommand)> sendIKCommand);
  void start(bool use_left_foot);
  void notifyJoints(nao_sensor_msgs::msg::JointPositions joints);

private:
  std::function<void(void)> notifyKickDone;
  std::function<void(motion_interfaces::msg::IKCommand)> sendIKCommand;

  bool duringKick = false;
  // motion_interfaces::msg::Kick receivedMsg;
  bool use_left_foot;
  float kickT = 0;

  float lastKickForward;
  float lastSide;
  float lastFooth;
  float lastRock;
};

#endif  // KICK__KICK_HPP_
