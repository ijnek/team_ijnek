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

#include "walk_generator/crouch_generator.hpp"
#include "walk_generator/maths_functions.hpp"

#define STAND_HIP_HEIGHT 0.248
#define WALK_HIP_HEIGHT 0.23
#define CROUCH_STAND_PERIOD 0.5

CrouchGenerator::CrouchGenerator()
: Node("CrouchGenerator"),
  hiph(STAND_HIP_HEIGHT)
{
  sub_twist =
    create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 1,
    [this](geometry_msgs::msg::Twist::SharedPtr twist) {
      RCLCPP_DEBUG(
        get_logger(), "Recevied twist: %g, %g, %g, %g, %g, %g",
        twist->linear.x, twist->linear.y, twist->linear.z,
        twist->angular.x, twist->angular.y, twist->angular.z);
      this->twist = *twist;
    });

  sub_joint_states =
    create_subscription<nao_sensor_msgs::msg::JointPositions>(
    "sensors/joint_positions", 1,
    [this](nao_sensor_msgs::msg::JointPositions::SharedPtr sensor_joints) {
      motion_msgs::msg::IKCommand ikCommand = generate_ik_command(*sensor_joints);
      pub_ik_command->publish(ikCommand);
    });

  pub_ik_command = create_publisher<motion_msgs::msg::IKCommand>("motion/ik_command", 1);
}

motion_msgs::msg::IKCommand CrouchGenerator::generate_ik_command(
  nao_sensor_msgs::msg::JointPositions &)
{
  RCLCPP_DEBUG(get_logger(), "generate_ik_command called");
  if (twist.angular.z == 0.0) {
    if (abs(hiph - STAND_HIP_HEIGHT) < .0001) {
      walkOption = STAND;
      t = 0;
    } else {
      walkOption = STANDUP;
    }
  } else {
    if (abs(hiph - WALK_HIP_HEIGHT) < .0001) {
      walkOption = READY;
      t = 0;
    } else {
      walkOption = CROUCH;
    }
  }
  RCLCPP_DEBUG(get_logger(), ("walkOption: " + std::to_string(walkOption)).c_str());

  t += dt;

  if (walkOption == STAND) {
    hiph = STAND_HIP_HEIGHT;
  } else if (walkOption == READY) {
    hiph = WALK_HIP_HEIGHT;
  } else if (walkOption == CROUCH) {
    hiph = STAND_HIP_HEIGHT + (WALK_HIP_HEIGHT - STAND_HIP_HEIGHT) * parabolicStep(
      dt, t,
      CROUCH_STAND_PERIOD);
  } else if (walkOption == STANDUP) {
    hiph = WALK_HIP_HEIGHT + (STAND_HIP_HEIGHT - WALK_HIP_HEIGHT) * parabolicStep(
      dt, t,
      CROUCH_STAND_PERIOD);
  }

  motion_msgs::msg::IKCommand ikCommand;
  ikCommand.hiph = hiph;

  RCLCPP_DEBUG(get_logger(), "hiph is: %.5f", hiph);

  return ikCommand;
}
