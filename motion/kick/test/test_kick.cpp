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

#include <gtest/gtest.h>
#include "kick/kick.hpp"

bool notifyKickDoneCalled = false;
bool sendSolePosesCalled = false;

void notifyKickDone()
{
  notifyKickDoneCalled = true;
}

void sendSolePoses(biped_interfaces::msg::SolePoses)
{
  sendSolePosesCalled = true;
}

TEST(TestKick, Test1)
{
  Kick kick(notifyKickDone, sendSolePoses);
  kick.notifyJoints(nao_sensor_msgs::msg::JointPositions{});
  ASSERT_FALSE(sendSolePosesCalled);

  kick.start(motion_interfaces::msg::Kick{});
  kick.notifyJoints(nao_sensor_msgs::msg::JointPositions{});
  ASSERT_TRUE(sendSolePosesCalled);
}
