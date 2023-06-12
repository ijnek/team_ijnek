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

#include "crouch/crouch.hpp"
#include <iostream>

Crouch::Crouch(
  std::function<void(biped_interfaces::msg::SolePoses)> sendSolePoses)
: sendSolePoses(sendSolePoses)
{
  command.l_sole.position.y = 0.05;
  command.l_sole.position.z = -0.265;

  command.r_sole.position.y = -0.05;
  command.r_sole.position.z = -0.265;
}

void Crouch::start()
{
  sendSolePoses(command);
}
