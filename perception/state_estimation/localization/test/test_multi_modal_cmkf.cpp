#include "gtest/gtest.h"

#include "../src/multi_modal_cmkf.hpp"

TEST(TestMultiModalCMKF, TestDefaultPoseWithCovariance)
{
  localization::MultiModalCMKF multi_modal_cmkf;

  auto pose_with_covariance = multi_modal_cmkf.getPoseWithCovariance();
  EXPECT_NEAR(pose_with_covariance.pose.position.x, 0.0, 0.01);
  EXPECT_NEAR(pose_with_covariance.pose.position.y, 0.0, 0.01);
  EXPECT_NEAR(pose_with_covariance.pose.position.z, 0.0, 0.01);
  unsigned index_x_x = 0;
  unsigned index_y_y = 7;
  unsigned index_z_z = 14;
  unsigned index_h_h = 21;
  EXPECT_NEAR(pose_with_covariance.covariance[index_x_x], 100000, 0.01);
  EXPECT_NEAR(pose_with_covariance.covariance[index_y_y], 100000, 0.01);
  EXPECT_NEAR(pose_with_covariance.covariance[index_z_z], 0.0, 0.01);
  EXPECT_NEAR(pose_with_covariance.covariance[index_h_h], 0.5, 0.01);
}

TEST(TestMultiModalCMKF, TestMoveOneMeterForward)
{
  localization::MultiModalCMKF multi_modal_cmkf;

  nav_msgs::msg::Odometry odometry;
  odometry.pose.pose.position.x = 1.0;
  multi_modal_cmkf.predict(odometry);

  auto pose_with_covariance = multi_modal_cmkf.getPoseWithCovariance();
  EXPECT_NEAR(pose_with_covariance.pose.position.x, 1.0, 0.01);
  EXPECT_NEAR(pose_with_covariance.pose.position.y, 0.0, 0.01);
  EXPECT_NEAR(pose_with_covariance.pose.position.z, 0.0, 0.01);
}

TEST(TestMultiModalCMKF, TestUpdateFarRightCorner)
{
  localization::MultiModalCMKF multi_modal_cmkf;

  // Let robot be at (3.5, -3.0, 0.0), looking at the corner marker at (4.5, -3.0)

  // Corner marker 1.0m in front of the robot, with two rays:
  // - First ray facing in -x
  // - Second ray facing in +y
  soccer_vision_3d_msgs::msg::MarkingArray marking_array;
  marking_array.header.frame_id = "base_footprint";
  soccer_vision_3d_msgs::msg::MarkingIntersection marking_intersection;
  marking_intersection.center.x = 1.0;
  marking_intersection.num_rays = 2;
  marking_intersection.rays = {
    geometry_msgs::msg::Vector3().set__x(-1.0).set__y(0.0).set__z(0.0),
    geometry_msgs::msg::Vector3().set__x(0.0).set__y(1.0).set__z(0.0)};
  marking_array.intersections.push_back(marking_intersection);

  multi_modal_cmkf.update(marking_array);

  // Pose should be updated to (3.5, -3.0, 0.0), if initial covariance is high enough
  auto pose_with_covariance = multi_modal_cmkf.getPoseWithCovariance();
  EXPECT_NEAR(pose_with_covariance.pose.position.x, 3.5, 0.01);
  EXPECT_NEAR(pose_with_covariance.pose.position.y, -3.0, 0.01);
}
