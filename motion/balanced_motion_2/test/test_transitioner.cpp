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


TEST_F(TransitionerTest, TestGetupToDive)
{
  // **getting_up** -> Stand -> dived
  State current;
  current.getting_up = true;

  State aim;
  aim.diving = true;

  auto [first_motion, first_motion_end_state] = transitioner.findNextMotion(current, aim);
  EXPECT_EQ(first_motion->name, getup.name);

  auto [second_motion, second_motion_end_state] = transitioner.findNextMotion(first_motion_end_state, aim);
  EXPECT_EQ(second_motion->name, stand.name);

  auto [third_motion, third_motion_end_state] = transitioner.findNextMotion(second_motion_end_state, aim);
  EXPECT_EQ(third_motion->name, dive.name);

  EXPECT_TRUE(third_motion_end_state.satisfies(aim));
}

TEST(TestTransitioner, TestDiveToWalk)
{
  MotionTransitioner transitioner;
  Walk walk;
  Stand stand;
  Dive dive;
  Getup getup;
  transitioner.addMotion(&walk);
  transitioner.addMotion(&stand);
  transitioner.addMotion(&dive);
  transitioner.addMotion(&getup);

  // **diving** -> Getup ->  Stand -> Walk
  State current;
  current.diving = true;

  State aim;
  aim.walking = true;

  auto [motion, motion_aim] = transitioner.findNextMotion(current, aim);
  EXPECT_EQ(motion->name, getup.name);
}
