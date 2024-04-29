#include "multi_modal_cmkf.hpp"

#include "cmkf.hpp"


namespace localization
{

// Forward declaration
static void predictCMKF(
  CMKF & cmkf, const nav_msgs::msg::Odometry & odometry,
  const nav_msgs::msg::Odometry & last_odom);

MultiModalCMKF::MultiModalCMKF()
{
  // For now, just initialize one CMKF
  StateVector state = StateVector::Zero();

  CovarianceMatrix covariance = CovarianceMatrix::Zero();
  covariance(ME_X_DIM, ME_X_DIM) = 100000;
  covariance(ME_Y_DIM, ME_Y_DIM) = 100000;
  covariance(ME_H_DIM, ME_H_DIM) = 0.5;

  kfs_.push_back(std::make_unique<CMKF>(state, covariance, 1.0));
}

MultiModalCMKF::~MultiModalCMKF()
{
}

void MultiModalCMKF::predict(const nav_msgs::msg::Odometry & odometry)
{
  for (auto & kf : kfs_) {
    predictCMKF(*kf, odometry, last_odometry_);
  }

  last_odometry_ = odometry;
}

void MultiModalCMKF::update(const soccer_vision_3d_msgs::msg::MarkingArray & markers)
{
  (void)markers;
}

geometry_msgs::msg::Pose MultiModalCMKF::getRobotPose()
{
  // For now, just return the pose of the first CMKF
  // Also need to fill the orientation field
  const auto& state = kfs_.at(0)->state;
  geometry_msgs::msg::Pose pose;
  pose.position.x = state(ME_X_DIM);
  pose.position.y = state(ME_Y_DIM);
  pose.position.z = state(ME_H_DIM);
  return pose;
}

static void predictCMKF(
  CMKF & cmkf, const nav_msgs::msg::Odometry & odometry,
  const nav_msgs::msg::Odometry & last_odom)
{
  // WARNING!!! THE CALCULATIONS HERE BELOW CURRENTLY ARE COMPLETELY WRONG!

  // Calculate difference in odometry
  float delta_x = odometry.pose.pose.position.x - last_odom.pose.pose.position.x;
  float delta_y = odometry.pose.pose.position.y - last_odom.pose.pose.position.y;

  // Predict the state
  cmkf.state(ME_X_DIM) += delta_x;
  cmkf.state(ME_Y_DIM) += delta_y;
}

}  // namespace localization
