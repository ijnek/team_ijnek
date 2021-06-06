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

#include "kick/kick.hpp"
#include <iostream>

Kick::Kick(
  std::function<void(void)> notifyKickDone,
  std::function<void(motion_msgs::msg::IKCommand)> sendIKCommand)
: notifyKickDone(notifyKickDone), sendIKCommand(sendIKCommand)
{
}

void Kick::start(motion_msgs::msg::Kick req)
{
  if (!duringKick) {
    duringKick = true;
    receivedMsg = req;
  }
}

void Kick::notifyJoints(nao_interfaces::msg::Joints)
{
  if (duringKick) {
    // Process joints here
    sendIKCommand(motion_msgs::msg::IKCommand{});

    // If kick is done,
    // notifyKickDone();
  }
}
