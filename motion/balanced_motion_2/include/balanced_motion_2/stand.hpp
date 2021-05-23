#ifndef BALANCED_MOTION_2__STAND_HPP_
#define BALANCED_MOTION_2__STAND_HPP_

#include "balanced_motion_2/motion.hpp"
#include "balanced_motion_2/inverse_kinematics.hpp"

class Stand : public Motion
{
public:
  Stand()
  : Motion("Stand") {}

  nao_interfaces::msg::Joints makeJoints (State &, State &, nao_interfaces::msg::Joints &) override
  {
    motion_msgs::msg::IKCommand command;
    command.hiph = CROUCHING_HIPH;

    curr = command;

    return calculate_joints(command);
  }

  bool isAchievable(State & aim) override
  {
    if (aim.standing.has_value() && aim.standing.value() == true) {
      return true;
    }
    return false;
  }

  State requirements() override
  {
    State requirements;
    requirements.on_feet = true;
    requirements.twist = geometry_msgs::msg::Twist{};
    return requirements;
  }

  motion_msgs::msg::IKCommand & getIKCommand()
  {
    return curr;
  }

private:
  motion_msgs::msg::IKCommand curr;
};

#endif  // BALANCED_MOTION_2__STAND_HPP_
