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
#include "kick/kick.hpp"
#include "std_msgs/msg/empty.hpp"
#include "nao_sensor_msgs/msg/joint_positions.hpp"
#include "biped_interfaces/msg/sole_poses.hpp"
#include "motion_interfaces/msg/kick.hpp"

using namespace std::placeholders;

class KickNode : public rclcpp::Node
{
public:
  KickNode()
  : Node("KickNode"),
    kick(
      std::bind(&KickNode::notifyKickDone, this),
      std::bind(&KickNode::sendAnklePoses, this, _1))
  {
    sub_joint_states =
      create_subscription<nao_sensor_msgs::msg::JointPositions>(
      "sensors/joint_positions", 1,
      [this](nao_sensor_msgs::msg::JointPositions::SharedPtr sensor_joints) {
        kick.notifyJoints(*sensor_joints);
      });

    sub_kick_start =
      create_subscription<motion_interfaces::msg::Kick>(
      "motion/kick", 1,
      [this](motion_interfaces::msg::Kick::SharedPtr kick_msg) {
        kick.start(*kick_msg);
      });

    pub_ankle_poses = create_publisher<biped_interfaces::msg::SolePoses>("motion/ankle_poses", 1);
    pub_kick_done = create_publisher<std_msgs::msg::Empty>("motion/kick_done", 1);
  }

private:
  Kick kick;

  void notifyKickDone()
  {
    pub_kick_done->publish(std_msgs::msg::Empty{});
  }

  void sendAnklePoses(biped_interfaces::msg::SolePoses ankle_poses)
  {
    pub_ankle_poses->publish(ankle_poses);
  }

  rclcpp::Subscription<nao_sensor_msgs::msg::JointPositions>::SharedPtr sub_joint_states;
  rclcpp::Subscription<motion_interfaces::msg::Kick>::SharedPtr sub_kick_start;
  rclcpp::Publisher<biped_interfaces::msg::SolePoses>::SharedPtr pub_ankle_poses;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_kick_done;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KickNode>());
  rclcpp::shutdown();
  return 0;
}
