#include <gtest/gtest.h>
#include "motion_transitioner/motion_transitioner.hpp"

bool startGetupCalled = false;
bool startKickCalled = false;

void startGetup(GetupRequest req)
{
  startGetupCalled = true;
}

void startKick(KickRequest req)
{
  startKickCalled = true;
}

TEST(TestMotionTransitioner2, Test1)
{
  MotionTransitioner motionTransitioner(startGetup, startKick);
  motionTransitioner.request(GetupRequest{});
  ASSERT_TRUE(startGetupCalled);
}

TEST(TestMotionTransitioner2, Test2)
{
  MotionTransitioner motionTransitioner(startGetup, startKick);
  motionTransitioner.request(GetupRequest{});
  motionTransitioner.request(KickRequest{});
  ASSERT_FALSE(startKickCalled);

  motionTransitioner.notifyGetupDone();
  motionTransitioner.request(KickRequest{});
  ASSERT_TRUE(startKickCalled);
}
