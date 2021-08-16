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
#include "crouch/crouch.hpp"

bool sendIKCommandCalled = false;

void sendIKCommand(motion_interfaces::msg::IKCommand)
{
  sendIKCommandCalled = true;
}

TEST(TestCrouch, Test1)
{
  Crouch crouch(sendIKCommand);
  crouch.start();
  ASSERT_TRUE(sendIKCommandCalled);
}
