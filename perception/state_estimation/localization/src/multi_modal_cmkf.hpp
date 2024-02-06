#pragma once

#include <vector>

#include "nav_msgs/msg/odometry.hpp"
#include "soccer_vision_3d_msgs/msg/marking_array.hpp"

namespace localization
{

class CMKF;

class MultiModalCMKF
{
public:
  MultiModalCMKF();
  ~MultiModalCMKF();

  void predict(const nav_msgs::msg::Odometry & odometry);
  void update(const soccer_vision_3d_msgs::msg::MarkingArray & markers);

private:
  std::vector<std::unique_ptr<CMKF>> kfs;
};

}  // namespace localization
