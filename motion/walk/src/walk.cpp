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

#define STAND_HIP_HEIGHT 0.248
#define WALK_HIP_HEIGHT 0.23
#define CROUCH_STAND_PERIOD 0.5
#define BASE_WALK_PERIOD 0.25
#define BASE_LEG_LIFT 0.012

Walk::Walk()
: Node("Walk"),
  hiph(STAND_HIP_HEIGHT)
{
  sub_twist =
    create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 1,
    [this](geometry_msgs::msg::Twist::SharedPtr twist) {
      RCLCPP_DEBUG(
        get_logger(), "Recevied twist: %g, %g, %g, %g, %g, %g",
        twist->linear.x, twist->linear.y, twist->linear.z,
        twist->angular.x, twist->angular.y, twist->angular.z);
      this->twist = *twist;
    });

  sub_joint_states =
    create_subscription<nao_sensor_msgs::msg::JointPositions>(
    "sensors/joint_positions", 1,
    [this](nao_sensor_msgs::msg::JointPositions::SharedPtr sensor_joints) {
      biped_interfaces::msg::SolePoses ikCommand = generate_sole_poses(*sensor_joints);
      pub_sole_poses->publish(ikCommand);
    });

  pub_sole_poses = create_publisher<biped_interfaces::msg::SolePoses>("motion/sole_poses", 1);
}

biped_interfaces::msg::SolePoses Walk::generate_sole_poses(
  nao_sensor_msgs::msg::JointPositions &)
{
  RCLCPP_DEBUG(get_logger(), "generate_sole_poses called");
  if (twist.linear.z == 0.0) {
    if (abs(hiph - STAND_HIP_HEIGHT) < .0001) {
      walkOption = STAND;
      t = 0;
    } else {
      walkOption = STANDUP;
    }
  } else {
    if (abs(hiph - WALK_HIP_HEIGHT) < .0001) {
      if (twist.linear.x != 0.0 || twist.linear.y != 0.0 || twist.angular.z != 0.0) {
        walkOption = WALK;
        if (t >= BASE_WALK_PERIOD) {
          t = 0;
        }
      } else {
        walkOption = READY;
        t = 0;
      }
    } else {
      walkOption = CROUCH;
    }
  }
  RCLCPP_DEBUG(get_logger(), ("walkOption: " + std::to_string(walkOption)).c_str());

  t += dt;

  float forwardL = 0, forwardR = 0, leftL = 0, leftR = 0,
    foothL = 0, foothR = 0, turnRL = 0;

  if (walkOption == STAND) {
    hiph = STAND_HIP_HEIGHT;
  } else if (walkOption == READY) {
    hiph = WALK_HIP_HEIGHT;
  } else if (walkOption == CROUCH) {
    hiph = STAND_HIP_HEIGHT + (WALK_HIP_HEIGHT - STAND_HIP_HEIGHT) * parabolicStep(
      dt, t,
      CROUCH_STAND_PERIOD);
  } else if (walkOption == STANDUP) {
    hiph = WALK_HIP_HEIGHT + (STAND_HIP_HEIGHT - WALK_HIP_HEIGHT) * parabolicStep(
      dt, t,
      CROUCH_STAND_PERIOD);
  } else if (walkOption == WALK) {
    float forward = twist.linear.x;
    float left = twist.linear.y;
    float turn = twist.angular.z;

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
        forwardR = forwardR0 + ((forward) / 2 - forwardR0) * linearStep(t, BASE_WALK_PERIOD);
        // swing-foot follow-through
        forwardL = forwardL0 +
          parabolicStep(dt, t, BASE_WALK_PERIOD, 0) * (-(forward) / 2 - forwardL0);
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
        forwardL = forwardL0 + ((forward) / 2 - forwardL0) * linearStep(t, BASE_WALK_PERIOD);
        // swing-foot follow-through
        forwardR = forwardR0 +
          parabolicStep(dt, t, BASE_WALK_PERIOD, 0) * (-(forward) / 2 - forwardR0);
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
    }
  }

  biped_interfaces::msg::SolePoses command;
  command.l_sole.position.x = forwardL;
  command.l_sole.position.y = leftL + 0.050;
  command.l_sole.position.z = -hiph + foothL;
  command.r_sole.position.x = forwardR;
  command.r_sole.position.y = leftR - 0.050;
  command.r_sole.position.z = -hiph + foothR;

  return command;
}
