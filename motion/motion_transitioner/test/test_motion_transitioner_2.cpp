#include <gtest/gtest.h>
#include "motion_transitioner/motion_transitioner.hpp"

bool startGetupCalled = false;
bool startKickCalled = false;
bool startCrouchCalled = false;

void startGetup(GetupRequest)
{
  startGetupCalled = true;
}

void startKick(KickRequest)
{
  startKickCalled = true;
}

void startCrouch(CrouchRequest)
{
  startCrouchCalled = true;
}

TEST(TestMotionTransitioner2, Test1)
{
  MotionTransitioner motionTransitioner(startGetup, startKick, startCrouch);
  motionTransitioner.request(GetupRequest{});
  ASSERT_TRUE(startGetupCalled);

  motionTransitioner.notifyGetupDone();
  ASSERT_TRUE(startCrouchCalled);
}

TEST(TestMotionTransitioner2, Test2)
{
  MotionTransitioner motionTransitioner(startGetup, startKick, startCrouch);
  motionTransitioner.request(GetupRequest{});
  motionTransitioner.request(KickRequest{});
  ASSERT_FALSE(startKickCalled);

  motionTransitioner.notifyGetupDone();
  motionTransitioner.request(KickRequest{});
  ASSERT_TRUE(startKickCalled);
}
