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

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "crouch/crouch.hpp"
#include "std_msgs/msg/empty.hpp"
#include "motion_msgs/msg/ik_command.hpp"

using namespace std::placeholders;

class CrouchNode : public rclcpp::Node
{
public:
  CrouchNode()
  : Node("CrouchNode"),
    crouch(
      std::bind(&CrouchNode::sendIKCommand, this, _1))
  {
    sub_crouch_start =
      create_subscription<std_msgs::msg::Empty>(
      "motion/crouch", 1,
      [this](std_msgs::msg::Empty::SharedPtr) {
        crouch.start();
      });

    pub_ik_command = create_publisher<motion_msgs::msg::IKCommand>("motion/ik_command", 1);
  }

private:
  Crouch crouch;

  void sendIKCommand(motion_msgs::msg::IKCommand ik_command)
  {
    pub_ik_command->publish(ik_command);
  }

  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_crouch_start;
  rclcpp::Publisher<motion_msgs::msg::IKCommand>::SharedPtr pub_ik_command;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CrouchNode>());
  rclcpp::shutdown();
  return 0;
}
