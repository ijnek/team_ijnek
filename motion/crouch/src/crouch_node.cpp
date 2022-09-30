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
#include "biped_interfaces/msg/sole_poses.hpp"

using namespace std::placeholders;

class CrouchNode : public rclcpp::Node
{
public:
  CrouchNode()
  : Node("CrouchNode"),
    crouch(
      std::bind(&CrouchNode::sendAnklePoses, this, _1))
  {
    sub_crouch_start =
      create_subscription<std_msgs::msg::Empty>(
      "motion/crouch", 1,
      [this](std_msgs::msg::Empty::SharedPtr) {
        crouch.start();
      });

    pub_ankle_poses = create_publisher<biped_interfaces::msg::SolePoses>("motion/ankle_poses", 1);
  }

private:
  Crouch crouch;

  void sendAnklePoses(biped_interfaces::msg::SolePoses ankle_poses)
  {
    pub_ankle_poses->publish(ankle_poses);
  }

  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_crouch_start;
  rclcpp::Publisher<biped_interfaces::msg::SolePoses>::SharedPtr pub_ankle_poses;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CrouchNode>());
  rclcpp::shutdown();
  return 0;
}
