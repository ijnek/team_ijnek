#ifndef BALANCED_MOTION_2__WALK_HPP_
#define BALANCED_MOTION_2__WALK_HPP_

#include "balanced_motion_2/motion.hpp"
#include "balanced_motion_2/inverse_kinematics.hpp"
#include "balanced_motion_2/motion_defs.hpp"
#include "balanced_motion_2/maths_functions.hpp"

class Walk : public Motion
{
public:
  Walk()
  : Motion("Walk") {}

  nao_interfaces::msg::Joints makeJoints(
    State & curr, State & aim,
    nao_interfaces::msg::Joints &) override
  {
    if (hasFinishedStep())
    {
      reset();
      leftFootIsSwingFoot = !leftFootIsSwingFoot;
    }

    t += MOTION_DT;

    if (first)
    {
      prev = curr.ik_command.value();
      first = false;

      if (curr.walking.value() == true) {
        twist = curr.twist.has_value() ? curr.twist.value() : geometry_msgs::msg::Twist{};
        if (aim.twist.value().linear.x > curr.twist.value().linear.x) {
          if (twist.linear.x < 0.05)
            twist.linear.x += 0.01;
        } else if (aim.twist.value().linear.x < curr.twist.value().linear.x) {
          if (twist.linear.x > -0.05)
            twist.linear.x -= 0.01;
        }
      } else {
        twist = geometry_msgs::msg::Twist{};
      }
    }

    motion_msgs::msg::IKCommand command = calculate_ik_command();

    curr_ik = command;

    return calculate_joints(command);
  }

  bool isAchievable(State & aim) override
  {
    if (aim.diving.has_value() && aim.diving.value() == true) {
      return false;
    }
    if (aim.standing.has_value() && aim.standing.value() == true) {
      return false;
    }
    return true;
  }

  State requirements() override
  {
    State requirements;
    requirements.on_feet = true;
    return requirements;
  }

  geometry_msgs::msg::Twist & getTwist()
  {
    return twist;
  }

  motion_msgs::msg::IKCommand & getIKCommand()
  {
    return curr_ik;
  }

  motion_msgs::msg::IKCommand calculate_ik_command()
  {
    float dt = MOTION_DT;

    float forwardL = 0, forwardR = 0, leftL = 0, leftR = 0,
    foothL = 0, foothR = 0, turnRL = 0;

    float forward = twist.linear.x;
    float left = twist.linear.y;
    float turn = twist.angular.z;

    // 5.1 Calculate the height to lift each swing foot
    float maxFootHeight = BASE_LEG_LIFT + abs(forward) * 0.01 + abs(left) * 0.03;
    float varFootHeight = maxFootHeight * parabolicReturnMod(t / BASE_WALK_PERIOD);     // 0.012 lift of swing foot from ground
    std::cout << "t: " << t << std::endl;
    std::cout << "varFootHeight: " << varFootHeight << std::endl;
    // 5.2 When walking in an arc, the outside foot needs to travel further than the inside one - void
    // 5.3L Calculate intra-walkphase forward, left and turn at time-step dt, for left swing foot
    if (leftFootIsSwingFoot) {
      std::cout << __FILE__ << ": " << __LINE__ << std::endl;
      if (weightHasShifted) {
        std::cout << __FILE__ << ": " << __LINE__ << std::endl;
        // 5.3.1L forward (the / by 2 is because the CoM moves as well and forwardL is wrt the CoM
        forwardL = parabolicStep(dt, t, BASE_WALK_PERIOD, 0, prev.forward_l, forward);
        forwardR = linearStep(t, BASE_WALK_PERIOD, prev.forward_r, -forward);
        // 5.3.2L Jab kick with left foot - removed
        // 5.3.3L Determine how much to lean from side to side - removed
        // 5.3.4L left
        if (left > 0) {
          leftL = parabolicStep(dt, t, BASE_WALK_PERIOD, 0.2, prev.left_l, left);
          leftR = -leftL;
        } else {
          leftL = prev.left_l * (1 - parabolicStep(dt, t, BASE_WALK_PERIOD, 0.0));
          leftR = -leftL;
        }
        // 5.3.5L turn (note, we achieve correct turn by splitting turn foot placement unevely over two steps, but 1.6 + 0.4 = 2 and adds up to two steps worth of turn)
        if (turn < 0) {
          turnRL = prev.turn_rl + (-1.6 * turn - prev.turn_rl) * parabolicStep(dt, t, BASE_WALK_PERIOD, 0.0);
        } else {
          turnRL = prev.turn_rl + (-0.4 * turn - prev.turn_rl) * parabolicStep(dt, t, BASE_WALK_PERIOD, 0.0);           //turn back to restore previous turn angle
        }
      }
      // 5.3.6L determine how high to lift the swing foot off the ground
      foothL = varFootHeight;                            // lift left swing foot
      foothR = 0;                                   // do not lift support foot;
    }
    // 5.3R Calculate intra-walkphase forward, left and turn at time-step dt, for right swing foot
    if (!leftFootIsSwingFoot) {              // if the support foot is left
      std::cout << __FILE__ << ": " << __LINE__ << std::endl;
      if (weightHasShifted) {
        std::cout << __FILE__ << ": " << __LINE__ << std::endl;
        // 5.3.1R forward
        forwardL = linearStep(t, BASE_WALK_PERIOD, prev.forward_l, -forward);
        forwardR = parabolicStep(dt, t, BASE_WALK_PERIOD, 0, prev.forward_r, forward);
        // 5.3.2R Jab-Kick with right foot - removed
        // 5.3.3R lean - not used
        // 5.3.4R left
        if (left < 0) {
          leftR = prev.left_r + (left - prev.left_r) * parabolicStep(dt, t, BASE_WALK_PERIOD, 0.2);
          leftL = -leftR;
        } else {
          leftR = prev.left_r * (1 - parabolicStep(dt, t, BASE_WALK_PERIOD, 0.0));
          leftL = -leftR;
        }
        // 5.3.5R turn
        if (turn < 0) {
          turnRL = prev.turn_rl + (0.4 * turn - prev.turn_rl) * parabolicStep(dt, t, BASE_WALK_PERIOD, 0.0);           //turn back to restore previous turn angle
        } else {
          turnRL = prev.turn_rl + (1.6 * turn - prev.turn_rl) * parabolicStep(dt, t, BASE_WALK_PERIOD, 0.0);
        }
        // 5.3.6R Foot height
      }
      foothR = varFootHeight;
      foothL = 0;
    }

    std::cout << "foothL: " << foothL
    << "foothR: " << foothR
    << "forwardL: " << forwardL
    << "forwardR: " << forwardR
    << "leftL: " << leftL
    << "leftR: " << leftR
    << "turnRL: " << turnRL << std::endl;

    motion_msgs::msg::IKCommand command;
    command.hiph = CROUCHING_HIPH;
    command.footh_l = foothL;
    command.footh_r = foothR;
    command.forward_l = forwardL;
    command.forward_r = forwardR;
    command.left_l = leftL;
    command.left_r = leftR;
    command.turn_rl = turnRL;

    std::cout << "foothL: " << foothL << std::endl;

    return command;
  }

  void reset() override
  {
    first = true;
    t = 0;
  }

  bool hasFinishedStep()
  {
    return t >= BASE_WALK_PERIOD;
  }

private:
  geometry_msgs::msg::Twist twist;
  motion_msgs::msg::IKCommand prev;
  bool leftFootIsSwingFoot = false;
  bool first = true;
  float t;
  bool weightHasShifted = true;
  motion_msgs::msg::IKCommand curr_ik;
};

#endif  // BALANCED_MOTION_2__WALK_HPP_
