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

#include "motion_key_frame/motion_key_frame.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "boost/filesystem.hpp"

#include "indexes.hpp"
#include "parser.hpp"

namespace fs = boost::filesystem;

namespace motion_key_frame
{

MotionKeyFrame::MotionKeyFrame(const rclcpp::NodeOptions & options)
: rclcpp::Node{"MotionKeyFrame", options}
{
  pub_joint_positions = create_publisher<nao_lola_command_msgs::msg::JointPositions>(
    "effectors/joint_positions", 1);
  pub_joint_stiffnesses = create_publisher<nao_lola_command_msgs::msg::JointStiffnesses>(
    "effectors/joint_stiffnesses", 1);

  sub_joint_states =
    create_subscription<nao_lola_sensor_msgs::msg::JointPositions>(
    "sensors/joint_positions", 1,
    [this](nao_lola_sensor_msgs::msg::JointPositions::SharedPtr sensor_joints) {
      if (!posInAction.empty()) {
        calculateEffectorJoints(*sensor_joints);
      }
    });

  action_server_ = rclcpp_action::create_server<motion_interfaces::action::MotionKeyFrame>(
    this,
    "action",
    std::bind(&MotionKeyFrame::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&MotionKeyFrame::handleCancel, this, std::placeholders::_1),
    std::bind(&MotionKeyFrame::handleAccepted, this, std::placeholders::_1));

  // Look through all the files in the directory and add them to the list of available motions
  std::vector<std::string> motion_names;
  auto pos_directory = ament_index_cpp::get_package_share_directory("motion_key_frame") + "/pos";
  fs::path pos_directory_path{pos_directory};
  if (fs::exists(pos_directory_path)) {
    fs::directory_iterator end_iter;
    for (fs::directory_iterator dir_itr(pos_directory_path); dir_itr != end_iter; ++dir_itr) {
      if (fs::is_regular_file(dir_itr->status())) {
        std::string file_name = dir_itr->path().filename().string();
        if (file_name.find(".pos") != std::string::npos) {
          std::string motion_name = file_name.substr(0, file_name.find(".pos"));
          motion_names.push_back(motion_name);
        }
      }
    }
  }

  // Print out motion names
  RCLCPP_INFO(this->get_logger(), "Available motions:");
  for (const auto & motion_name : motion_names) {
    RCLCPP_INFO(this->get_logger(), "  %s", motion_name.c_str());
  }

  // Load the motions into a map of motion name to motion
  for (const auto & motion_name : motion_names) {
    std::string file_path = pos_directory + "/" + motion_name + ".pos";
    std::ifstream ifstream(file_path);
    if (ifstream.is_open()) {
      RCLCPP_DEBUG(this->get_logger(), ("Pos file succesfully loaded from " + file_path).c_str());
      auto lines = readLines(ifstream);
      auto parseResult = parser::parse(lines);
      if (parseResult.successful) {
        motions[motion_name] = parseResult.keyFrames;
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to parse motion %s", motion_name.c_str());
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "Couldn't open file %s", file_path.c_str());
    }
  }
}

MotionKeyFrame::~MotionKeyFrame() {}

std::vector<std::string> MotionKeyFrame::readLines(std::ifstream & ifstream)
{
  std::vector<std::string> ret;

  while (!ifstream.eof()) {
    std::string line;
    std::getline(ifstream, line);
    ret.push_back(line);
  }

  return ret;
}

void MotionKeyFrame::calculateEffectorJoints(
  nao_lola_sensor_msgs::msg::JointPositions & sensor_joints)
{
  int time_ms = (rclcpp::Node::now() - begin).nanoseconds() / 1e6;

  if (posFinished(time_ms)) {
    // We've finished the motion, set to DONE
    posInAction.clear();
    RCLCPP_DEBUG(this->get_logger(), "Pos finished before");
    auto result = std::make_shared<motion_interfaces::action::MotionKeyFrame::Result>();
    goal_handle_->succeed(result);
    RCLCPP_DEBUG(this->get_logger(), "Pos finished after");
    return;
  }

  if (firstTickSinceActionStarted) {
    nao_lola_command_msgs::msg::JointPositions command;
    command.indexes = indexes::indexes;
    command.positions = std::vector<float>(
      sensor_joints.positions.begin(), sensor_joints.positions.end());
    keyFrameStart =
      std::make_unique<KeyFrame>(0, command, nao_lola_command_msgs::msg::JointStiffnesses{});
    firstTickSinceActionStarted = false;
  }

  RCLCPP_DEBUG(this->get_logger(), ("time_ms is: " + std::to_string(time_ms)).c_str());

  const auto & previousKeyFrame = findPreviousKeyFrame(time_ms);
  const auto & nextKeyFrame = findNextKeyFrame(time_ms);

  float timeFromPreviousKeyFrame = time_ms - previousKeyFrame.t_ms;
  float timeToNextKeyFrame = nextKeyFrame.t_ms - time_ms;
  float duration = timeFromPreviousKeyFrame + timeToNextKeyFrame;

  RCLCPP_DEBUG(
    this->get_logger(), ("timeFromPreviousKeyFrame, timeFromPreviousKeyFrame, duration: " +
    std::to_string(timeFromPreviousKeyFrame) + ", " + std::to_string(timeToNextKeyFrame) + ", " +
    std::to_string(duration)).c_str());

  float alpha = timeToNextKeyFrame / duration;
  float beta = timeFromPreviousKeyFrame / duration;

  RCLCPP_DEBUG(
    this->get_logger(), ("alpha, beta: " + std::to_string(alpha) + ", " + std::to_string(
      beta)).c_str());

  nao_lola_command_msgs::msg::JointPositions effector_joints;
  effector_joints.indexes = indexes::indexes;

  for (unsigned int i = 0; i < nao_lola_command_msgs::msg::JointIndexes::NUMJOINTS; ++i) {
    float previous = previousKeyFrame.positions.positions.at(i);
    float next = nextKeyFrame.positions.positions.at(i);
    effector_joints.positions.push_back(previous * alpha + next * beta);

    RCLCPP_DEBUG(
      this->get_logger(), ("previous, next, result: " + std::to_string(
        previous) + ", " + std::to_string(next) + ", " +
      std::to_string(effector_joints.positions.at(i))).c_str());
  }

  pub_joint_positions->publish(effector_joints);
  pub_joint_stiffnesses->publish(nextKeyFrame.stiffnesses);
}

const KeyFrame & MotionKeyFrame::findPreviousKeyFrame(int time_ms)
{
  const auto & keyFrames = motions.at(posInAction);
  for (auto it = keyFrames.rbegin(); it != keyFrames.rend(); ++it) {
    const auto & keyFrame = *it;
    int keyFrameDeadline = keyFrame.t_ms;
    if (time_ms >= keyFrameDeadline) {
      return keyFrame;
    }
  }

  return *keyFrameStart;
}

const KeyFrame & MotionKeyFrame::findNextKeyFrame(int time_ms)
{
  const auto & keyFrames = motions.at(posInAction);
  for (const auto & keyFrame : keyFrames) {
    int keyFrameDeadline = keyFrame.t_ms;
    if (time_ms < keyFrameDeadline) {
      return keyFrame;
    }
  }

  RCLCPP_ERROR(this->get_logger(), "findKeyFrame: Should never reach here");
  return keyFrames.back();
}

bool MotionKeyFrame::posFinished(int time_ms)
{
  const auto & keyFrames = motions.at(posInAction);
  if (keyFrames.size() == 0) {
    return true;
  }

  const auto lastKeyFrame = keyFrames.back();
  int lastKeyFrameTime = lastKeyFrame.t_ms;
  if (time_ms >= lastKeyFrameTime) {
    return true;
  }

  return false;
}

rclcpp_action::GoalResponse MotionKeyFrame::handleGoal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const motion_interfaces::action::MotionKeyFrame::Goal> goal)
{
  RCLCPP_DEBUG(get_logger(), "Received goal request");
  (void)uuid;
  (void)goal;
  if (!posInAction.empty()) {
    return rclcpp_action::GoalResponse::REJECT;
  } else {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
}
rclcpp_action::CancelResponse MotionKeyFrame::handleCancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<motion_interfaces::action::MotionKeyFrame>> goal_handle)
{
  RCLCPP_DEBUG(get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  posInAction.clear();
  goal_handle_.reset();
  return rclcpp_action::CancelResponse::ACCEPT;
}
void MotionKeyFrame::handleAccepted(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<motion_interfaces::action::MotionKeyFrame>> goal_handle)
{
  RCLCPP_DEBUG(this->get_logger(), "Starting Pos Action");
  begin = rclcpp::Node::now();
  posInAction = goal_handle->get_goal()->name;
  firstTickSinceActionStarted = true;
  goal_handle_ = goal_handle;
}

}  // namespace motion_key_frame

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(motion_key_frame::MotionKeyFrame)
