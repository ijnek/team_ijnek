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

#ifndef BALANCED_MOTION_2__LINEAR_HPP_
#define BALANCED_MOTION_2__LINEAR_HPP_

#include <string>
#include <vector>
#include <utility>
#include <memory>
#include "rclcpp/logger.hpp"
#include "nao_interfaces/msg/joints.hpp"
#include "std_msgs/msg/bool.hpp"
#include "boost/filesystem.hpp"
#include "balanced_motion_2/motion.hpp"

namespace fs = boost::filesystem;

class Linear : public Motion
{
public:
  Linear(std::string name, std::string fileName, int dt_ms);

  nao_interfaces::msg::Joints makeJoints (State state, nao_interfaces::msg::Joints & sensor_joints);

  bool hasFinished();

  void reset();

private:
  bool initialiseKeyFrameVector(std::string filePath);

  void calculate_effector_joints(nao_interfaces::msg::Joints & sensor_joints);

  std::pair<nao_interfaces::msg::Joints, int> & findPreviousKeyFrame(int time_ms);

  std::pair<nao_interfaces::msg::Joints, int> & findNextKeyFrame(int time_ms);

  std::vector<std::pair<nao_interfaces::msg::Joints, int>> keyFrames;

  bool fileSuccessfullyRead = false;
  bool posInAction = false;
  bool firstTickSinceActionStarted = true;
  std::pair<nao_interfaces::msg::Joints, int> jointsWhenActionStarted;
  int t_ms = 0;
  int dt_ms;

  rclcpp::Logger logger = rclcpp::get_logger("Linear");
};

#endif  // BALANCED_MOTION_2__LINEAR_HPP_