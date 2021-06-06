#include <gtest/gtest.h>
#include "motion_transitioner/motion_transitioner.hpp"

bool startGetupCalled = false;
bool startKickCalled = false;
bool startCrouchCalled = false;

void startGetup(Getup)
{
  startGetupCalled = true;
}

void startKick(Kick)
{
  startKickCalled = true;
}

void startCrouch(Crouch)
{
  startCrouchCalled = true;
}

TEST(TestMotionTransitioner2, Test1)
{
  MotionTransitioner motionTransitioner(startGetup, startKick, startCrouch);
  motionTransitioner.request(Getup{});
  ASSERT_TRUE(startGetupCalled);

  motionTransitioner.notifyGetupDone();
  ASSERT_TRUE(startCrouchCalled);
}

TEST(TestMotionTransitioner2, Test2)
{
  MotionTransitioner motionTransitioner(startGetup, startKick, startCrouch);
  motionTransitioner.request(Getup{});
  motionTransitioner.request(Kick{});
  ASSERT_FALSE(startKickCalled);

  motionTransitioner.notifyGetupDone();
  motionTransitioner.request(Kick{});
  ASSERT_TRUE(startKickCalled);
}
