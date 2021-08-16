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

#include "walk_generator/walk_generator.hpp"
#include "walk_generator/maths_functions.hpp"

#define STAND_HIP_HEIGHT 0.248
#define WALK_HIP_HEIGHT 0.23
#define CROUCH_STAND_PERIOD 0.5
#define BASE_WALK_PERIOD 0.25
#define BASE_LEG_LIFT 0.012

WalkGenerator::WalkGenerator()
: Node("WalkGenerator"),
  hiph(STAND_HIP_HEIGHT)
{
  sub_joint_states =
    create_subscription<nao_sensor_msgs::msg::JointPositions>(
    "sensors/joint_positions", 1,
    [this](nao_sensor_msgs::msg::JointPositions::SharedPtr sensor_joints) {
      if (walk_goal_handle_) {
        if (walk_goal_handle_->is_canceling()) {
          RCLCPP_DEBUG(get_logger(), "Goal Cancelled");
          walk_goal_handle_->canceled(walk_result_);
          walk_goal_handle_ = nullptr;
          target_ = nullptr;
        } else {
          RCLCPP_DEBUG(get_logger(), "Generating joints");
          motion_interfaces::msg::IKCommand ikCommand = generate_ik_command(*sensor_joints);
          pub_ik_command->publish(ikCommand);
        }
      }
    });

  pub_ik_command = create_publisher<motion_interfaces::msg::IKCommand>("motion/ik_command", 1);

  this->action_server_ = rclcpp_action::create_server<motion_interfaces::action::Walk>(
    this,
    "walk",
    [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const WalkGoal>)
    {
      RCLCPP_DEBUG(get_logger(), "Received goal request");
      // Accept all goals
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [this](const std::shared_ptr<WalkGoalHandle>)
    {
      RCLCPP_INFO(get_logger(), "Received request to cancel goal");
      // Accept all cancel requests
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    std::bind(&WalkGenerator::handleAccepted, this, std::placeholders::_1));
}

motion_interfaces::msg::IKCommand WalkGenerator::generate_ik_command(
  nao_sensor_msgs::msg::JointPositions &)
{
  RCLCPP_DEBUG(get_logger(), "generate_ik_command called");
  if (target_->linear.z == 0.0) {
    if (abs(hiph - STAND_HIP_HEIGHT) < .0001) {
      walkOption = STAND;
      t = 0;
    } else {
      walkOption = STANDUP;
    }
  } else {
    if (abs(hiph - WALK_HIP_HEIGHT) < .0001) {
      if (target_->linear.x != 0.0 || target_->linear.y != 0.0 || target_->angular.z != 0.0) {
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
    float forward = target_->linear.x;
    float left = target_->linear.y;
    float turn = target_->angular.z;

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
        // 5.3.2L Jab walk with left foot - removed
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
        // 5.3.2R Jab-Walk with right foot - removed
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

  motion_interfaces::msg::IKCommand ikCommand;
  ikCommand.hiph = hiph;
  ikCommand.forward_l = forwardL;
  ikCommand.forward_r = forwardR;
  ikCommand.left_l = leftL;
  ikCommand.left_r = leftR;
  ikCommand.turn_rl = turnRL;
  ikCommand.footh_l = foothL;
  ikCommand.footh_r = foothR;

  return ikCommand;
}

void WalkGenerator::handleAccepted(
  const std::shared_ptr<WalkGoalHandle> goal_handle)
{
  // Abort any existing goal
  if (walk_goal_handle_) {
    RCLCPP_WARN(
      get_logger(),
      "Walk goal received before a previous goal finished. Aborting previous goal");
    walk_goal_handle_->abort(walk_result_);
  }
  walk_goal_handle_ = goal_handle;
  walk_feedback_ = std::make_shared<motion_interfaces::action::Walk::Feedback>();
  walk_result_ = std::make_shared<motion_interfaces::action::Walk::Result>();
  target_ = std::make_shared<geometry_msgs::msg::Twist>(
    goal_handle->get_goal()->target);
}
