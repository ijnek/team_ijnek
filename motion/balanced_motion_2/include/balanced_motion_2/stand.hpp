#ifndef BALANCED_MOTION_2__STAND_HPP_
#define BALANCED_MOTION_2__STAND_HPP_

#include "balanced_motion_2/motion.hpp"
#include "balanced_motion_2/inverse_kinematics.hpp"

#define STANDING_HIPH 0.248

class Stand : public Motion
{
public:
  Stand()
  : Motion("Stand") {}

  nao_interfaces::msg::Joints makeJoints (State &, nao_interfaces::msg::Joints &) override
  {
    motion_msgs::msg::IKCommand command;
    command.hiph = STANDING_HIPH;
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
    requirements.travelling = false;
    return requirements;
  }
};

#endif  // BALANCED_MOTION_2__STAND_HPP_
