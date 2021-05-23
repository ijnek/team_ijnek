#include <gtest/gtest.h>
#include "motion_transitioner/motion_transitioner.hpp"
#include "motion_transitioner/motion.hpp"

class StandKick : public Motion
{
public:
  StandKick() : Motion("stand kick"){}
};

class WalkKick : public Motion
{
public:
  WalkKick() : Motion("walk kick"){}
};

class Walk : public Motion
{
public:
  Walk() : Motion("walk"){}
};

class Stand : public Motion
{
public:
  Stand() : Motion("stand"){}
};

class Getup : public Motion
{
public:
  Getup() : Motion("getup"){}
};

class Dive : public Motion
{
public:
  Dive() : Motion("dive"){}
};

class Dead : public Motion
{
public:
  Dead() : Motion("dead"){}
};

class TransitionerTest : public ::testing::Test
{
protected:
  TransitionerTest() :
    standKick(new StandKick()),
    walkKick(new WalkKick()),
    walk(new Walk()),
    stand(new Stand()),
    getup(new Getup()),
    dive(new Dive()),
    dead(new Dead())
  {
    standKick->canTransitionFrom = {stand};
    walkKick->canTransitionFrom = {walk};
    walk->canTransitionFrom = {stand, walkKick};
    stand->canTransitionFrom = {standKick, walk, getup};
    getup->canTransitionFrom = {dead};
    dive->canTransitionFrom = {walk, stand, walkKick, standKick};
    dead->canTransitionFrom = {dive, getup};
  }

public:
  std::shared_ptr<StandKick> standKick;
  std::shared_ptr<WalkKick> walkKick;
  std::shared_ptr<Walk> walk;
  std::shared_ptr<Stand> stand;
  std::shared_ptr<Getup> getup;
  std::shared_ptr<Dive> dive;
  std::shared_ptr<Dead> dead;
};

TEST_F(TransitionerTest, TestWalkToStand)
{
  std::shared_ptr<Motion> motion = findNextMotion(walk, stand);
  ASSERT_NE(motion, nullptr);
  EXPECT_EQ(stand->name, motion->name);
}

TEST_F(TransitionerTest, TestStandToWalkKick)
{
  // EXPECT: walk -> walkKick
  std::shared_ptr<Motion> motion1 = findNextMotion(stand, walkKick);
  ASSERT_NE(motion1, nullptr);
  EXPECT_EQ(walk->name, motion1->name);

  std::shared_ptr<Motion> motion2 = findNextMotion(motion1, walkKick);
  ASSERT_NE(motion2, nullptr);
  EXPECT_EQ(walkKick->name, motion2->name);
}

TEST_F(TransitionerTest, TestDiveToWalkKick)
{
  // EXPECT: dive -> dead -> getup -> stand -> walk -> walkKick
  std::shared_ptr<Motion> currentMotion = dive;
  std::shared_ptr<Motion> finalMotion = walkKick;
  
  std::vector<std::shared_ptr<Motion>> expected_steps({dead, getup, stand, walk, finalMotion});

  for (std::shared_ptr<Motion> expected : expected_steps)
  {
    currentMotion = findNextMotion(currentMotion, finalMotion);
    ASSERT_NE(currentMotion, nullptr);
    EXPECT_EQ(expected->name, currentMotion->name);
  }
}