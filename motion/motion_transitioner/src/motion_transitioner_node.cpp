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
#include "motion_transitioner/motion_transitioner.hpp"
#include "std_msgs/msg/empty.hpp"

using namespace std::placeholders;

class MotionTransitionerNode : public rclcpp::Node
{
public:
  MotionTransitionerNode()
  : Node("MotionTransitionerNode"),
    motionTransitioner(
      std::bind(&MotionTransitionerNode::startGetup, this, _1),
      std::bind(&MotionTransitionerNode::startKick, this, _1),
      std::bind(&MotionTransitionerNode::startCrouch, this, _1))
  {
    sub_getup_request =
      create_subscription<Getup>(
      "behaviour/getup", 1,
      [this](Getup::SharedPtr request) {
        motionTransitioner.request(*request);
      });

    sub_getup_done =
      create_subscription<std_msgs::msg::Empty>(
      "motion/getup_done", 1,
      [this](std_msgs::msg::Empty::SharedPtr) {
        motionTransitioner.notifyGetupDone();
      });
  
    sub_kick_request =
      create_subscription<Kick>(
      "behaviour/kick", 1,
      [this](Kick::SharedPtr request) {
        motionTransitioner.request(*request);
      });

    sub_kick_done =
      create_subscription<std_msgs::msg::Empty>(
      "motion/kick_done", 1,
      [this](std_msgs::msg::Empty::SharedPtr) {
      motionTransitioner.notifyKickDone();
      });

    sub_crouch_request =
      create_subscription<Crouch>(
      "behaviour/crouch", 1,
      [this](Crouch::SharedPtr request) {
        motionTransitioner.request(*request);
      });

    pub_start_getup = this->create_publisher<Getup>("motion/getup", 10);
    pub_start_kick = this->create_publisher<Kick>("motion/kick", 10);
    pub_start_crouch = this->create_publisher<Crouch>("motion/crouch", 10);
  }

private:
  MotionTransitioner motionTransitioner;

  void startGetup(Getup req)
  {
    pub_start_getup->publish(req);
  }

  void startKick(Kick req)
  {
    pub_start_kick->publish(req);
  }

  void startCrouch(Crouch req)
  {
    pub_start_crouch->publish(req);
  }

  rclcpp::Subscription<Getup>::SharedPtr sub_getup_request;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_getup_done;
  rclcpp::Publisher<Getup>::SharedPtr pub_start_getup;

  rclcpp::Subscription<Kick>::SharedPtr sub_kick_request;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_kick_done;
  rclcpp::Publisher<Kick>::SharedPtr pub_start_kick;

  rclcpp::Subscription<Crouch>::SharedPtr sub_crouch_request;
  rclcpp::Publisher<Crouch>::SharedPtr pub_start_crouch;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotionTransitionerNode>());
  rclcpp::shutdown();
  return 0;
}
