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
