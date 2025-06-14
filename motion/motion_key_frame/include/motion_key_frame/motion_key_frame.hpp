// Copyright 2024 Kenji Brameld
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

#ifndef MOTION_KEY_FRAME__MOTION_KEY_FRAME_HPP_
#define MOTION_KEY_FRAME__MOTION_KEY_FRAME_HPP_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "motion_interfaces/action/motion_key_frame.hpp"
#include "motion_key_frame/key_frame.hpp"
#include "nao_lola_command_msgs/msg/joint_positions.hpp"
#include "nao_lola_command_msgs/msg/joint_stiffnesses.hpp"
#include "nao_lola_sensor_msgs/msg/joint_positions.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp_action/rclcpp_action.hpp"


namespace motion_key_frame
{

class MotionKeyFrame : public rclcpp::Node
{
public:
  explicit MotionKeyFrame(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});
  virtual ~MotionKeyFrame();

private:
  rclcpp::Subscription<nao_lola_sensor_msgs::msg::JointPositions>::SharedPtr sub_joint_states;
  rclcpp::Publisher<nao_lola_command_msgs::msg::JointPositions>::SharedPtr pub_joint_positions;
  rclcpp::Publisher<nao_lola_command_msgs::msg::JointStiffnesses>::SharedPtr pub_joint_stiffnesses;
  rclcpp_action::Server<motion_interfaces::action::MotionKeyFrame>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const motion_interfaces::action::MotionKeyFrame::Goal> goal);
  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<motion_interfaces::action::MotionKeyFrame>> goal_handle);
  void handleAccepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<motion_interfaces::action::MotionKeyFrame>> goal_handle);

  std::vector<std::string> readLines(std::ifstream & ifstream);
  void calculateEffectorJoints(nao_lola_sensor_msgs::msg::JointPositions & sensor_joints);
  const KeyFrame & findPreviousKeyFrame(int time_ms);
  const KeyFrame & findNextKeyFrame(int time_ms);
  bool posFinished(int time_ms);

  std::map<std::string, std::vector<KeyFrame>> motions;
  std::string posInAction;
  bool firstTickSinceActionStarted = true;
  std::unique_ptr<KeyFrame> keyFrameStart;
  rclcpp::Time begin;

  std::shared_ptr<rclcpp_action::ServerGoalHandle<motion_interfaces::action::MotionKeyFrame>>
  goal_handle_;
};

}  // namespace motion_key_frame

#endif  // MOTION_KEY_FRAME__MOTION_KEY_FRAME_HPP_
