#ifndef BALANCED_MOTION_2__STAND_HPP_
#define BALANCED_MOTION_2__STAND_HPP_

#include "balanced_motion_2/motion.hpp"

class Stand : public Motion
{
public:
  Stand()
  : Motion("Stand") {}

  nao_interfaces::msg::Joints makeJoints (State &, nao_interfaces::msg::Joints &) override
  {
    nao_interfaces::msg::Joints joints;
    joints.angles[nao_interfaces::msg::Joints::LSHOULDERPITCH] = 90 * 3.1415 / 180;
    joints.angles[nao_interfaces::msg::Joints::RSHOULDERPITCH] = 90 * 3.1415 / 180;
    return joints;
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
