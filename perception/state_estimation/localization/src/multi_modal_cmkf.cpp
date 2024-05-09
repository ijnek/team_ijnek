#include "multi_modal_cmkf.hpp"

#include "cmkf.hpp"


namespace localization
{

// Forward declaration
static void predictCMKF(
  CMKF & cmkf, const nav_msgs::msg::Odometry & odometry,
  const nav_msgs::msg::Odometry & last_odom);
static void updateCMKF(CMKF & cmkf, const soccer_vision_3d_msgs::msg::MarkingEllipse & ellipse);
static void updateCMKF(CMKF & cmkf, const soccer_vision_3d_msgs::msg::MarkingIntersection & intersection);
static void updateCMKF(CMKF & cmkf, const soccer_vision_3d_msgs::msg::MarkingSegment & segment);

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
  predictAllCMKFs(odometry);
  deleteOffFieldCMKFs();
  mergeCMKFs();
  normalizeCMKFWeights();
  deleteLowWeightCMKFs();
  determineBestCMKF();

  last_odometry_ = odometry;
}

void MultiModalCMKF::update(const soccer_vision_3d_msgs::msg::MarkingArray & markers)
{
  (void)markers;
  updateAllCMKFs(markers);
  deleteOffFieldCMKFs();
  mergeCMKFs();
  normalizeCMKFWeights();
  deleteLowWeightCMKFs();
  determineBestCMKF();
}

void MultiModalCMKF::predictAllCMKFs(const nav_msgs::msg::Odometry & odometry)
{
  for (auto & kf : kfs_) {
    predictCMKF(*kf, odometry, last_odometry_);
  }
}

void MultiModalCMKF::updateAllCMKFs(const soccer_vision_3d_msgs::msg::MarkingArray & markers)
{
  for (auto & kf : kfs_) {
    for (const auto & ellipse : markers.ellipses) {
      updateCMKF(*kf, ellipse);
    }
    for (const auto & intersection : markers.intersections) {
      updateCMKF(*kf, intersection);
    }
    for (const auto & segment : markers.segments) {
      updateCMKF(*kf, segment);
    }
  }
}

void MultiModalCMKF::deleteOffFieldCMKFs()
{
}

void MultiModalCMKF::mergeCMKFs()
{
}

void MultiModalCMKF::normalizeCMKFWeights()
{
}

void MultiModalCMKF::deleteLowWeightCMKFs()
{
}

void MultiModalCMKF::determineBestCMKF()
{
}

geometry_msgs::msg::PoseWithCovariance MultiModalCMKF::getPoseWithCovariance()
{
  // For now, just return the pose of the first CMKF
  // Also need to fill the orientation field
  geometry_msgs::msg::PoseWithCovariance pose_with_covariance;

  const auto & state = kfs_.at(0)->state;
  pose_with_covariance.pose.position.x = state(ME_X_DIM);
  pose_with_covariance.pose.position.y = state(ME_Y_DIM);
  pose_with_covariance.pose.position.z = state(ME_H_DIM);

  const auto & covariance = kfs_.at(0)->covariance;
  pose_with_covariance.covariance[0] = static_cast<double>(covariance(ME_X_DIM, ME_X_DIM));
  pose_with_covariance.covariance[1] = static_cast<double>(covariance(ME_X_DIM, ME_Y_DIM));
  pose_with_covariance.covariance[3] = static_cast<double>(covariance(ME_X_DIM, ME_H_DIM));
  pose_with_covariance.covariance[6] = static_cast<double>(covariance(ME_Y_DIM, ME_X_DIM));
  pose_with_covariance.covariance[7] = static_cast<double>(covariance(ME_Y_DIM, ME_Y_DIM));
  pose_with_covariance.covariance[9] = static_cast<double>(covariance(ME_Y_DIM, ME_H_DIM));
  pose_with_covariance.covariance[18] = static_cast<double>(covariance(ME_H_DIM, ME_X_DIM));
  pose_with_covariance.covariance[19] = static_cast<double>(covariance(ME_H_DIM, ME_Y_DIM));
  pose_with_covariance.covariance[21] = static_cast<double>(covariance(ME_H_DIM, ME_H_DIM));

  return pose_with_covariance;
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

static void updateCMKF(CMKF & cmkf, const soccer_vision_3d_msgs::msg::MarkingEllipse & ellipse)
{
  (void) cmkf;
  (void) ellipse;
}

static void updateCMKF(CMKF & cmkf, const soccer_vision_3d_msgs::msg::MarkingIntersection & intersection)
{
  (void) cmkf;
  (void) intersection;
}

static void updateCMKF(CMKF & cmkf, const soccer_vision_3d_msgs::msg::MarkingSegment & segment)
{
  (void) cmkf;
  (void) segment;
}

}  // namespace localization
