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

#include <string>
#include <vector>
#include <utility>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nao_interfaces/msg/joints.hpp"
#include "std_msgs/msg/bool.hpp"
#include "boost/filesystem.hpp"
#include "rclcpp/time.hpp"
#include "balanced_motion_2/linear.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "balanced_motion_2/motion_defs.hpp"

namespace fs = boost::filesystem;


Linear::Linear(std::string name, std::string fileName) : Motion(name), dt_ms(MOTION_DT_MS)
{
  std::string package_share_directory = ament_index_cpp::get_package_share_directory(
    "balanced_motion_2");

  fs::path dir_path(package_share_directory);
  fs::path file_path("pos/" + fileName);
  fs::path full_path = dir_path / file_path;
  std::string filePath = full_path.string();

  fileSuccessfullyRead = initialiseKeyFrameVector(filePath);
  RCLCPP_DEBUG(logger, "Pos file succesfully loaded from " + filePath);
}

void Linear::reset()
{
  t_ms = 0;
  firstTickSinceActionStarted = true;
}

bool Linear::initialiseKeyFrameVector(std::string filePath)
{
  std::ifstream in(filePath);
  if (!in.is_open()) {
    RCLCPP_ERROR(logger, "Couldn't open file: " + filePath);
    return false;
  }

  int keyFrameTime = 0;

  while (!in.eof()) {
    std::string line;
    std::getline(in, line);

    if (line.front() == '!') {
      RCLCPP_DEBUG(logger, "Found joint line: " + line);
      line.erase(line.begin());

      std::istringstream ss(line);

      std::vector<std::string> splitted_line(std::istream_iterator<std::string>{ss},
        std::istream_iterator<std::string>());

      if (splitted_line.size() != nao_interfaces::msg::Joints::NUMJOINTS + 1) {
        // +1 because there is the duration component as well
        RCLCPP_ERROR(logger, "pos file joint line missing joint values or duration!");
        return false;
      }

      nao_interfaces::msg::Joints newJoint;

      for (unsigned int i = 0; i < nao_interfaces::msg::Joints::NUMJOINTS; ++i) {
        std::string position_deg_string = splitted_line[i];

        try {
          float position_deg = std::stof(position_deg_string);
          float position_rad = position_deg * M_PI / 180;
          newJoint.angles.at(i) = position_rad;
        } catch (std::invalid_argument &) {
          RCLCPP_ERROR(
            logger,
            "joint value '" + position_deg_string +
            "' is not a valid joint value (cannot be converted to float)");
          return false;
        }
      }

      std::string duration_string = splitted_line.back();
      try {
        int duration = std::stoi(duration_string);
        keyFrameTime += duration;
      } catch (std::invalid_argument &) {
        RCLCPP_ERROR(
          logger,
          "duration '" + duration_string +
          "' is not a valid duration value (cannot be converted to int)");
        return false;
      }

      keyFrames.push_back(std::make_pair(newJoint, keyFrameTime));
    }
  }

  return true;
}

nao_interfaces::msg::Joints Linear::makeJoints (State &, State &, nao_interfaces::msg::Joints & sensor_joints)
{
  t_ms += dt_ms;

  if (hasFinished()) {
    // We've finished the motion, set to DONE
    RCLCPP_DEBUG(logger, "Pos finished");
    return keyFrames.back().first;
  }

  if (firstTickSinceActionStarted) {
    jointsWhenActionStarted = std::make_pair(sensor_joints, 0);
    firstTickSinceActionStarted = false;
  }

  RCLCPP_DEBUG(logger, "t_ms is: " + std::to_string(t_ms));

  std::pair<nao_interfaces::msg::Joints, int> & previousKeyFrame = findPreviousKeyFrame(t_ms);
  std::pair<nao_interfaces::msg::Joints, int> & nextKeyFrame = findNextKeyFrame(t_ms);

  float timeFromPreviousKeyFrame = t_ms - previousKeyFrame.second;
  float timeToNextKeyFrame = nextKeyFrame.second - t_ms;
  float duration = timeFromPreviousKeyFrame + timeToNextKeyFrame;

  RCLCPP_DEBUG(
    logger, "timeFromPreviousKeyFrame, timeFromPreviousKeyFrame, duration: " +
    std::to_string(timeFromPreviousKeyFrame) + ", " + std::to_string(timeToNextKeyFrame) + ", " +
    std::to_string(duration));

  float alpha = timeToNextKeyFrame / duration;
  float beta = timeFromPreviousKeyFrame / duration;

  RCLCPP_DEBUG(
    logger, "alpha, beta: " + std::to_string(alpha) + ", " + std::to_string(
      beta));

  nao_interfaces::msg::Joints effector_joints;

  for (unsigned int i = 0; i < nao_interfaces::msg::Joints::NUMJOINTS; ++i) {
    float previous = previousKeyFrame.first.angles[i];
    float next = nextKeyFrame.first.angles[i];
    effector_joints.angles[i] = previous * alpha + next * beta;

    RCLCPP_DEBUG(
      logger, "previous, next, result: " + std::to_string(
        previous) + ", " + std::to_string(next) + ", " +
      std::to_string(effector_joints.angles[i]));
  }

  return effector_joints;
}

std::pair<nao_interfaces::msg::Joints, int> & Linear::findPreviousKeyFrame(int t_ms)
{
  for (auto it = keyFrames.rbegin(); it != keyFrames.rend(); ++it) {
    int keyFrameDeadline = it->second;
    if (t_ms >= keyFrameDeadline) {
      return *it;
    }
  }

  return jointsWhenActionStarted;
}

std::pair<nao_interfaces::msg::Joints, int> & Linear::findNextKeyFrame(int t_ms)
{
  for (std::pair<nao_interfaces::msg::Joints, int> & keyFrame : keyFrames) {
    int keyFrameDeadline = keyFrame.second;
    if (t_ms < keyFrameDeadline) {
      return keyFrame;
    }
  }

  RCLCPP_ERROR(logger, "findKeyFrame: Should never reach here");
  return keyFrames.back();
}

bool Linear::hasFinished()
{
  if (keyFrames.size() == 0) {
    return true;
  }

  std::pair<nao_interfaces::msg::Joints, int> lastKeyFrame = keyFrames.back();
  int lastKeyFrameTime = lastKeyFrame.second;
  if (t_ms >= lastKeyFrameTime) {
    return true;
  }

  return false;
}