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
#include "rclcpp_action/rclcpp_action.hpp"
#include "kick/kick.hpp"
#include "std_msgs/msg/empty.hpp"
#include "nao_sensor_msgs/msg/joint_positions.hpp"
#include "motion_interfaces/msg/ik_command.hpp"
#include "motion_interfaces/msg/kick.hpp"
#include "motion_interfaces/action/kick.hpp"

using namespace std::placeholders;

class KickNode : public rclcpp::Node
{
public:
  using KickGoal = motion_interfaces::action::Kick::Goal;
  using KickGoalHandle = rclcpp_action::ServerGoalHandle<motion_interfaces::action::Kick>;

  KickNode()
  : Node("KickNode"),
    kick(
      std::bind(&KickNode::notifyKickDone, this),
      std::bind(&KickNode::sendIKCommand, this, _1))
  {
    sub_joint_states =
      create_subscription<nao_sensor_msgs::msg::JointPositions>(
      "sensors/joint_positions", 1,
      [this](nao_sensor_msgs::msg::JointPositions::SharedPtr sensor_joints) {
        RCLCPP_DEBUG(get_logger(), "Recevied joint_positions");
        if (kick_goal_handle_) {
          RCLCPP_DEBUG(get_logger(), "Found kick_goal_handle, executing kick.");
          if (kick_goal_handle_->is_canceling()) {
            RCLCPP_DEBUG(
              get_logger(), "Goal Cancelled");
            auto result = std::make_shared<motion_interfaces::action::Kick::Result>();
            result->done = false;
            kick_goal_handle_->canceled(result);
            kick_goal_handle_ = nullptr;
          } else {
            kick.notifyJoints(*sensor_joints);
          }
        }
      });

    pub_ik_command = create_publisher<motion_interfaces::msg::IKCommand>("motion/ik_command", 1);

    this->action_server_ = rclcpp_action::create_server<motion_interfaces::action::Kick>(
      this,
      "kick",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const KickGoal> goal)
      {
        RCLCPP_DEBUG(get_logger(), "Received goal request with foot: %s", goal->use_left_foot ? "left" : "right");
        // Accept all goals
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const std::shared_ptr<KickGoalHandle>)
      {
        RCLCPP_INFO(get_logger(), "Received request to cancel goal");
        // Accept all cancel requests
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      std::bind(&KickNode::handleAccepted, this, _1));
  }

private:
  void sendIKCommand(motion_interfaces::msg::IKCommand ik_command)
  {
    pub_ik_command->publish(ik_command);
  }

  void handleAccepted(
    const std::shared_ptr<KickGoalHandle> goal_handle)
  {
    // Abort any existing goal
    if (kick_goal_handle_) {
      RCLCPP_WARN(
        get_logger(),
        "Kick goal received before a previous goal finished. Aborting previous goal");
      auto result = std::make_shared<motion_interfaces::action::Kick::Result>();
      result->done = false;
      kick_goal_handle_->abort(result);
    }
    kick_goal_handle_ = goal_handle;
    kick.start(goal_handle->get_goal()->use_left_foot);
  }

  void notifyKickDone()
  {
    auto result = std::make_shared<motion_interfaces::action::Kick::Result>();
    result->done = true;
    kick_goal_handle_->succeed(result);
    kick_goal_handle_ = nullptr;
  }

  Kick kick;
  rclcpp::Subscription<nao_sensor_msgs::msg::JointPositions>::SharedPtr sub_joint_states;
  rclcpp::Publisher<motion_interfaces::msg::IKCommand>::SharedPtr pub_ik_command;
  rclcpp_action::Server<motion_interfaces::action::Kick>::SharedPtr action_server_;

  std::shared_ptr<KickGoalHandle> kick_goal_handle_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KickNode>());
  rclcpp::shutdown();
  return 0;
}
