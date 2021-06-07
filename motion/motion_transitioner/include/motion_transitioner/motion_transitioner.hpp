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

#ifndef MOTION_TRANSITIONER__MOTION_TRANSITIONER_HPP_
#define MOTION_TRANSITIONER__MOTION_TRANSITIONER_HPP_

#include <functional>
#include "motion_msgs/msg/kick.hpp"
#include "motion_msgs/msg/getup.hpp"
#include "std_msgs/msg/empty.hpp"

class MotionTransitioner
{
public:
  MotionTransitioner(
    std::function<void(motion_msgs::msg::Getup)> startGetup,
    std::function<void(motion_msgs::msg::Kick)> startKick,
    std::function<void(std_msgs::msg::Empty)> startCrouch);
  void request(motion_msgs::msg::Getup req);
  void request(motion_msgs::msg::Kick req);
  // void request(WalkRequest req);
  void request(std_msgs::msg::Empty req);
  void notifyGetupDone();
  void notifyKickDone();

private:
  std::function<void(motion_msgs::msg::Getup)> startGetup;
  std::function<void(motion_msgs::msg::Kick)> startKick;
  std::function<void(std_msgs::msg::Empty)> startCrouch;

  bool duringGetup = false;
  bool duringKick = false;
};

#endif  // MOTION_TRANSITIONER__MOTION_TRANSITIONER_HPP_
