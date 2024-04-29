#include "gtest/gtest.h"

#include "../src/multi_modal_cmkf.hpp"

TEST(TestMultiModalCMKF, TestConstructor)
{
  localization::MultiModalCMKF multi_modal_cmkf;
  auto robot_pose = multi_modal_cmkf.getRobotPose();

  // Just a random test. Remove later.
  EXPECT_NEAR(robot_pose.position.x, 0.0, 0.01);
  EXPECT_NEAR(robot_pose.position.y, 0.0, 0.01);
  EXPECT_NEAR(robot_pose.position.z, 0.0, 0.01);
}

TEST(TestMultiModalCMKF, TestMoveOneMeterForward)
{
  localization::MultiModalCMKF multi_modal_cmkf;

  nav_msgs::msg::Odometry odometry;
  odometry.pose.pose.position.x = 1.0;
  multi_modal_cmkf.predict(odometry);

  auto robot_pose = multi_modal_cmkf.getRobotPose();
  EXPECT_NEAR(robot_pose.position.x, 1.0, 0.01);
  EXPECT_NEAR(robot_pose.position.y, 0.0, 0.01);
  EXPECT_NEAR(robot_pose.position.z, 0.0, 0.01);
}
