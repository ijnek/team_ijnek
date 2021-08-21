// This file is based on UNSW Sydney's codebase, but has been modified significantly.
// Both copyright notices are provided below.
//
// Copyright (c) 2018 UNSW Sydney.  All rights reserved.
//
// Licensed under Team rUNSWift's original license. See the "LICENSE-runswift"
// file to obtain a copy of the license.
//
// ---------------------------------------------------------------------------------
//
// Copyright 2021 Kenji Brameld
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "walk/walk.hpp"
#include "walk/maths_functions.hpp"

#define WALK_ANKLE_Z -0.18
#define BASE_WALK_PERIOD 0.25
#define BASE_LEG_LIFT 0.012

Walk::Walk(
  std::function<void(void)> notifyGoalAchieved,
  std::function<void(nao_ik_interfaces::msg::IKCommand)> sendIKCommand)
: notifyGoalAchieved(notifyGoalAchieved),
  sendIKCommand(sendIKCommand),
  logger(rclcpp::get_logger("walk"))
{
}

void Walk::setParams(
  float maxForward, float maxLeft, float maxTurn, float speedMultiplier, float footLiftAmp,
  float period, float ankleZ, float maxForwardChange, float maxLeftChange, float maxTurnChange)
{
  this->period = period;
  this->ankleZ = ankleZ;
  stepCalculator.setParams(
    maxForward, maxLeft, maxTurn, speedMultiplier, footLiftAmp,
    maxForwardChange, maxLeftChange, maxTurnChange);
}

void Walk::notifyJoints(nao_sensor_msgs::msg::JointPositions & jointPositions)
{
  RCLCPP_DEBUG(logger, "notifyJoints called");
  if (!duringWalk) {
    RCLCPP_DEBUG(logger, "Returning, not during walk");
    return;
  }

  if (t == 0) {
    // Move towards targetWalkOption and targetTwist
    if (targetWalkOption == WALK) {
      if (walkOption == CROUCH) {
        walkOption = WALK;
      }
      auto [nextTwist, nextStep] = stepCalculator.calculateNext(currTwist, target);
      currTwist = nextTwist;
      currStep = nextStep;
    } else if (targetWalkOption == CROUCH) {
      if (walkOption == WALK) {
        auto [nextTwist, nextStep] = stepCalculator.calculateNext(currTwist, target);
        currTwist = nextTwist;
        currStep = nextStep;
      }
    }
  }

  // if (walkOption == CROUCH)
  // {
  //   if (targetWalkOption == WALK)
  //   {
  //   }
  // }

  // if (t == 0)
  // {
  //   if ()
  // }

  if (target.linear.x != 0.0 || target.linear.y != 0.0 || target.angular.z != 0.0) {
    walkOption = WALK;
  } else {
    walkOption = CROUCH;
    t = 0;
  }
  RCLCPP_DEBUG(logger, ("walkOption: " + std::to_string(walkOption)).c_str());

  rclcpp::Time time = jointPositions.header.stamp;
  double dt = 0;
  if (firstMsg) {
    firstMsg = false;
  } else {
    dt = (time - prev_time_).nanoseconds() / 1e9;
  }
  prev_time_ = time;

  t += dt;

  float forwardL = 0, forwardR = 0, leftL = 0, leftR = 0,
    foothL = 0, foothR = 0, turnRL = 0;

  if (walkOption == WALK) {
    float forward = target.linear.x;
    float left = target.linear.y;
    float turn = target.angular.z;

    // 5.1 Calculate the height to lift each swing foot
    float maxFootHeight = BASE_LEG_LIFT + abs(forward) * 0.01 + abs(left) * 0.03;
    float varfootHeight = maxFootHeight * parabolicReturnMod(t / BASE_WALK_PERIOD);
    // 5.2 When walking in an arc, the outside foot needs to travel further
    //     than the inside one - void
    // 5.3L Calculate intra-walkphase forward, left and turn at time-step dt,
    //      for left swing foot
    if (isLeftPhase) {                 // if the support foot is right
      if (weightHasShifted) {
        // 5.3.1L forward (the / by 2 is because the CoM moves as well and forwardL is wrt the CoM
        forwardR = forwardR0 + (-(forward / 2) - forwardR0) * linearStep(t, BASE_WALK_PERIOD);
        // swing-foot follow-through
        forwardL = forwardL0 +
          parabolicStep(dt, t, BASE_WALK_PERIOD, 0) * (forward / 2 - forwardL0);
        // 5.3.2L Jab kick with left foot - removed
        // 5.3.3L Determine how much to lean from side to side - removed
        // 5.3.4L left
        if (left > 0) {
          leftL = leftL0 + (left - leftL0) * parabolicStep(dt, t, BASE_WALK_PERIOD, 0.2);
          leftR = -leftL;
        } else {
          leftL = leftL0 * (1 - parabolicStep(dt, t, BASE_WALK_PERIOD, 0.0));
          leftR = -leftL;
        }
        // 5.3.5L turn (note, we achieve correct turn by splitting turn foot placement unevely over
        //        two steps, but 1.6 + 0.4 = 2 and adds up to two steps worth of turn)
        if (turn < 0) {
          turnRL = turnRL0 + (-1.6 * turn - turnRL0) * parabolicStep(dt, t, BASE_WALK_PERIOD, 0.0);
        } else {
          // turn back to restore previous turn angle
          turnRL = turnRL0 + (-0.4 * turn - turnRL0) * parabolicStep(dt, t, BASE_WALK_PERIOD, 0.0);
        }
      }
      // 5.3.6L determine how high to lift the swing foot off the ground
      foothL = varfootHeight;                            // lift left swing foot
      foothR = 0;                                   // do not lift support foot;
    }
    // 5.3R Calculate intra-walkphase forward, left and turn at time-step dt, for right swing foot
    if (!isLeftPhase) {              // if the support foot is left
      if (weightHasShifted) {
        // 5.3.1R forward
        forwardL = forwardL0 + (-(forward / 2) - forwardL0) * linearStep(t, BASE_WALK_PERIOD);
        // swing-foot follow-through
        forwardR = forwardR0 +
          parabolicStep(dt, t, BASE_WALK_PERIOD, 0) * (forward / 2 - forwardR0);
        // 5.3.2R Jab-Kick with right foot - removed
        // 5.3.3R lean - not used
        // 5.3.4R left
        if (left < 0) {
          leftR = leftR0 + (left - leftR0) * parabolicStep(dt, t, BASE_WALK_PERIOD, 0.2);
          leftL = -leftR;
        } else {
          leftR = leftR0 * (1 - parabolicStep(dt, t, BASE_WALK_PERIOD, 0.0));
          leftL = -leftR;
        }
        // 5.3.5R turn
        if (turn < 0) {
          // turn back to restore previous turn angle
          turnRL = turnRL0 + (0.4 * turn - turnRL0) * parabolicStep(dt, t, BASE_WALK_PERIOD, 0.0);
        } else {
          turnRL = turnRL0 + (1.6 * turn - turnRL0) * parabolicStep(dt, t, BASE_WALK_PERIOD, 0.0);
        }
        // 5.3.6R Foot height
      }
      foothR = varfootHeight;
      foothL = 0;
    }

    if (t >= BASE_WALK_PERIOD) {
      turnRL0 = turnRL;
      forwardR0 = forwardR;
      forwardL0 = forwardL;
      leftL0 = leftL;
      leftR0 = leftR;
      isLeftPhase = !isLeftPhase;
      t = 0;
    }
  }

  // Report if we've achieved any goals
  if (targetWalkOption == CROUCH) {
    if (currTwist == geometry_msgs::msg::Twist{}) { // if zero
      notifyGoalAchieved();
    }
  } else if (targetWalkOption == WALK) {
    if (currTwist == target) {
      notifyGoalAchieved();
    }
  }

  nao_ik_interfaces::msg::IKCommand command;
  command.left_ankle.position.x = forwardL;
  command.left_ankle.position.y = leftL + 0.050;
  command.left_ankle.position.z = WALK_ANKLE_Z + foothL;
  command.right_ankle.position.x = forwardR;
  command.right_ankle.position.y = leftR - 0.050;
  command.right_ankle.position.z = WALK_ANKLE_Z + foothR;

  RCLCPP_DEBUG(
    logger, "Sending IKCommand: %f, %f, %f, %f, %f, %f",
    command.left_ankle.position.x,
    command.left_ankle.position.y,
    command.left_ankle.position.z,
    command.right_ankle.position.x,
    command.right_ankle.position.y,
    command.right_ankle.position.z);

  sendIKCommand(command);
}

void Walk::abort()
{
  duringWalk = false;
  firstMsg = true;
}

void Walk::crouch()
{
  duringWalk = true;
  targetWalkOption = CROUCH;
  firstMsg = true;
}

void Walk::walk(const geometry_msgs::msg::Twist & target)
{
  duringWalk = true;
  targetWalkOption = WALK;
  this->target = target;
}
