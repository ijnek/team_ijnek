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
#include "motion_transitioner/motion_transitioner.hpp"

bool startGetupCalled = false;
bool startKickCalled = false;
bool startCrouchCalled = false;

void startGetup(motion_interfaces::msg::Getup)
{
  startGetupCalled = true;
}

void startKick(motion_interfaces::msg::Kick)
{
  startKickCalled = true;
}

void startCrouch(std_msgs::msg::Empty)
{
  startCrouchCalled = true;
}

TEST(TestMotionTransitioner, Test1)
{
  MotionTransitioner motionTransitioner(startGetup, startKick, startCrouch);
  motionTransitioner.request(motion_interfaces::msg::Getup{});
  ASSERT_TRUE(startGetupCalled);

  motionTransitioner.notifyGetupDone();
  ASSERT_TRUE(startCrouchCalled);
}

TEST(TestMotionTransitioner, Test2)
{
  MotionTransitioner motionTransitioner(startGetup, startKick, startCrouch);
  motionTransitioner.request(motion_interfaces::msg::Getup{});
  motionTransitioner.request(motion_interfaces::msg::Kick{});
  ASSERT_FALSE(startKickCalled);

  motionTransitioner.notifyGetupDone();
  motionTransitioner.request(motion_interfaces::msg::Kick{});
  ASSERT_TRUE(startKickCalled);
}
