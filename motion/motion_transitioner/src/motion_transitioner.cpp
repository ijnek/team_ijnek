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

#include "motion_transitioner/motion_transitioner.hpp"
#include <iostream>

MotionTransitioner::MotionTransitioner(
  std::function<void(motion_msgs::msg::Getup)> startGetup,
  std::function<void(motion_msgs::msg::Kick)> startKick,
  std::function<void(motion_msgs::msg::Crouch)> startCrouch)
: startGetup(startGetup), startKick(startKick), startCrouch(startCrouch)
{
}

void MotionTransitioner::request(motion_msgs::msg::Getup req)
{
  duringGetup = true;
  startGetup(req);
}

void MotionTransitioner::request(motion_msgs::msg::Kick req)
{
  if (!duringGetup) {
    duringKick = true;
    startKick(req);
  }
}

void MotionTransitioner::request(motion_msgs::msg::Crouch)
{
  std::cout << "not impelmented" << std::endl;
}

void MotionTransitioner::notifyGetupDone()
{
  duringGetup = false;
  startCrouch(motion_msgs::msg::Crouch{});
}

void MotionTransitioner::notifyKickDone()
{
  duringKick = false;
  startCrouch(motion_msgs::msg::Crouch{});
}
