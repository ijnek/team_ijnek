#include <gtest/gtest.h>
#include "balanced_motion_2/motion_transitioner.hpp"
#include "balanced_motion_2/walk.hpp"
#include "balanced_motion_2/stand.hpp"
#include "balanced_motion_2/getup.hpp"
#include "balanced_motion_2/dive.hpp"

class TransitionerTest : public ::testing::Test
{
protected:
  TransitionerTest()
  {
    transitioner.addMotion(&walk);
    transitioner.addMotion(&stand);
    transitioner.addMotion(&dive);
    transitioner.addMotion(&getup);
  }
public:
  MotionTransitioner transitioner;
  Walk walk;
  Stand stand;
  Dive dive;
  Getup getup;
};

TEST_F(TransitionerTest, TestStandingToDiving)
{
  // EXPECTED: Dive
  State current(true);
  current.standing = true;
  current.on_feet = true;

  State aim;
  aim.diving = true;

  auto [motion, motion_aim_state] = transitioner.findNextMotion(current, aim);
  EXPECT_EQ(motion->name, dive.name);

  EXPECT_TRUE(motion_aim_state.satisfies(aim));
}

TEST_F(TransitionerTest, TestWalkingToDiving)
{
  // EXPECTED: Dive
  State current(true);
  current.walking = true;
  current.on_feet = true;

  State aim;
  aim.diving = true;

  auto [motion, motion_aim_state] = transitioner.findNextMotion(current, aim);
  EXPECT_EQ(motion->name, dive.name);

  EXPECT_TRUE(motion_aim_state.satisfies(aim));
}

TEST_F(TransitionerTest, TestGettingUpToDiving)
{
  // EXPECTED: nullptr, we have to wait until getup is coplete
  State current(true);
  current.getting_up = true;

  State aim;
  aim.diving = true;

  auto [motion_1, aim_state_1] = transitioner.findNextMotion(current, aim);
  EXPECT_EQ(motion_1, nullptr);
}

TEST_F(TransitionerTest, TestOnGroundToDiving)
{
  // EXPECTED: Getup -> Dive
  State current(true);
  current.on_ground = true;

  State aim;
  aim.diving = true;

  auto [motion_1, aim_state_1] = transitioner.findNextMotion(current, aim);
  ASSERT_NE(motion_1, nullptr);
  EXPECT_EQ(motion_1->name, getup.name);

  auto [motion_2, aim_state_2] = transitioner.findNextMotion(aim_state_1, aim);
  ASSERT_NE(motion_2, nullptr);
  EXPECT_EQ(motion_2->name, dive.name);

  EXPECT_TRUE(aim_state_2.satisfies(aim));
}

TEST_F(TransitionerTest, TestStandingToWalking)
{
  // EXPECTED: Walk
  State current(true);
  current.standing = true;
  current.on_feet = true;

  State aim;
  aim.walking = true;

  auto [motion, motion_aim_state] = transitioner.findNextMotion(current, aim);
  EXPECT_EQ(motion->name, walk.name);

  EXPECT_TRUE(motion_aim_state.satisfies(aim));
}

TEST_F(TransitionerTest, TestGettingUpToWalking)
{
  // EXPECTED: nullptr, we have to wait until getup is coplete
  State current(true);
  current.getting_up = true;

  State aim;
  aim.walking = true;

  auto [motion_1, aim_state_1] = transitioner.findNextMotion(current, aim);
  EXPECT_EQ(motion_1, nullptr);
}

TEST_F(TransitionerTest, TestOnGroundToWalking)
{
  // EXPECTED: Getup -> Walk
  State current(true);
  current.on_ground = true;

  State aim;
  aim.walking = true;

  auto [motion_1, aim_state_1] = transitioner.findNextMotion(current, aim);
  ASSERT_NE(motion_1, nullptr);
  EXPECT_EQ(motion_1->name, getup.name);

  auto [motion_2, aim_state_2] = transitioner.findNextMotion(aim_state_1, aim);
  ASSERT_NE(motion_2, nullptr);
  EXPECT_EQ(motion_2->name, walk.name);

  EXPECT_TRUE(aim_state_2.satisfies(aim));
}

TEST_F(TransitionerTest, TestDivingToWalking)
{
  // EXPECTED: nullptr, we have to wait until getup is coplete
  State current(true);
  current.diving = true;

  State aim;
  aim.walking = true;

  auto [motion_1, aim_state_1] = transitioner.findNextMotion(current, aim);
  EXPECT_EQ(motion_1, nullptr);
}

TEST_F(TransitionerTest, TestWalkingAndTravellingToStanding)
{
  // Walk -> Stand
  State current(true);
  current.walking = true;
  current.travelling = true;
  current.on_feet = true;

  State aim;
  aim.standing = true;

  auto [motion_1, aim_state_1] = transitioner.findNextMotion(current, aim);
  ASSERT_NE(motion_1, nullptr);
  EXPECT_EQ(motion_1->name, walk.name);

  auto [motion_2, aim_state_2] = transitioner.findNextMotion(aim_state_1, aim);
  ASSERT_NE(motion_2, nullptr);
  EXPECT_EQ(motion_2->name, stand.name);

  EXPECT_TRUE(aim_state_2.satisfies(aim));
}

TEST_F(TransitionerTest, TestGettingUpToStanding)
{
  // EXPECTED: nullptr, we have to wait until getup is coplete
  State current(true);
  current.getting_up = true;

  State aim;
  aim.standing = true;

  auto [motion_1, aim_state_1] = transitioner.findNextMotion(current, aim);
  EXPECT_EQ(motion_1, nullptr);
}

TEST_F(TransitionerTest, TestOnGroundToStanding)
{
  // Getup
  State current(true);
  current.on_ground = true;

  State aim;
  aim.standing = true;

  auto [motion_1, aim_state_1] = transitioner.findNextMotion(current, aim);
  ASSERT_NE(motion_1, nullptr);
  EXPECT_EQ(motion_1->name, getup.name);

  EXPECT_TRUE(aim_state_1.satisfies(aim));
}

TEST_F(TransitionerTest, TestDivingToStanding)
{
  // EXPECTED: nullptr, we have to wait until dive is coplete
  State current(true);
  current.diving = true;

  State aim;
  aim.standing = true;

  auto [motion_1, aim_state_1] = transitioner.findNextMotion(current, aim);
  EXPECT_EQ(motion_1, nullptr);
}
