#pragma once

#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
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

  // getAllRobotPos();
  // getRobotPos();
  geometry_msgs::msg::PoseWithCovariance getPoseWithCovariance();
  // getRobotPoseUncertainty();
  // getRobotHeadingUncertainty();

private:
  std::vector<std::unique_ptr<CMKF>> kfs_;
  nav_msgs::msg::Odometry last_odometry_;
};

}  // namespace localization
