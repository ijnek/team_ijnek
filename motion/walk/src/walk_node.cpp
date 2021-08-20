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
#include "walk/walk.hpp"
#include "motion_interfaces/action/walk.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nao_ik_interfaces/msg/ik_command.hpp"

class WalkNode : public rclcpp::Node
{
public:
  using WalkGoal = motion_interfaces::action::Walk::Goal;
  using WalkGoalHandle = rclcpp_action::ServerGoalHandle<motion_interfaces::action::Walk>;

  WalkNode()
  : Node("WalkNode"),
    walk(
      std::bind(&WalkNode::notifyGoalAchieved, this),
      std::bind(&WalkNode::sendIKCommand, this, std::placeholders::_1))
  {
    sub_joint_states =
      create_subscription<nao_sensor_msgs::msg::JointPositions>(
      "sensors/joint_positions", 1,
      [this](nao_sensor_msgs::msg::JointPositions::SharedPtr sensor_joints) {
        RCLCPP_DEBUG(get_logger(), "Received joint_positions");
        if (walk_goal_handle_) {
          RCLCPP_DEBUG(get_logger(), "Found walk_goal_handle, executing walk.");
          if (walk_goal_handle_->is_canceling()) {
            RCLCPP_DEBUG(
              get_logger(), "Goal Cancelled");
            auto result = std::make_shared<motion_interfaces::action::Walk::Result>();
            walk_goal_handle_->canceled(result);
            walk_goal_handle_ = nullptr;
          } else {
            walk.notifyJoints(*sensor_joints);
          }
        }
      });

    pub_ik_command = create_publisher<nao_ik_interfaces::msg::IKCommand>("motion/ik_command", 1);

    this->action_server_ = rclcpp_action::create_server<motion_interfaces::action::Walk>(
      this,
      "walk",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const WalkGoal> goal)
      {
        geometry_msgs::msg::Twist target = goal->target;
        RCLCPP_DEBUG(
          get_logger(), "Recevied WalkGoal with target: %g, %g, %g, %g, %g, %g",
          target.linear.x, target.linear.y, target.linear.z,
          target.angular.x, target.angular.y, target.angular.z);
        // Accept all goals
        walk.setTarget(target);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const std::shared_ptr<WalkGoalHandle>)
      {
        RCLCPP_INFO(get_logger(), "Received request to cancel goal");
        // Accept all cancel requests
        walk.abort();
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      std::bind(&WalkNode::handleAccepted, this, std::placeholders::_1));
  }

private:
  Walk walk;
  rclcpp::Subscription<nao_sensor_msgs::msg::JointPositions>::SharedPtr sub_joint_states;
  rclcpp::Publisher<nao_ik_interfaces::msg::IKCommand>::SharedPtr pub_ik_command;
  rclcpp_action::Server<motion_interfaces::action::Walk>::SharedPtr action_server_;

  std::shared_ptr<WalkGoalHandle> walk_goal_handle_;

  void sendIKCommand(nao_ik_interfaces::msg::IKCommand ik_command)
  {
    pub_ik_command->publish(ik_command);
  }

  void handleAccepted(
    const std::shared_ptr<WalkGoalHandle> goal_handle)
  {
    RCLCPP_DEBUG(get_logger(), "handleAccepted");
    // Abort any existing goal
    if (walk_goal_handle_) {
      RCLCPP_DEBUG(
        get_logger(),
        "Walk goal received before a previous goal finished. Aborting previous goal");
      auto result = std::make_shared<motion_interfaces::action::Walk::Result>();
      walk_goal_handle_->abort(result);
    }
    walk_goal_handle_ = goal_handle;
    walk.setTarget(goal_handle->get_goal()->target);
  }

  void notifyGoalAchieved()
  {
    auto result = std::make_shared<motion_interfaces::action::Walk::Result>();
    walk_goal_handle_->succeed(result);
    walk_goal_handle_ = nullptr;
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WalkNode>());
  rclcpp::shutdown();
  return 0;
}
