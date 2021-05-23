#ifndef STAND__STAND_HPP_
#define STAND__STAND_HPP_

#include "motion_transitioner/motion.hpp"
#include "nao_interfaces/msg/joints.hpp"

class Stand : public Motion
{
public:
  Stand()
  : Motion("stand") {}
  nao_interfaces::msg::Joints makeJoints();
};

#endif  // STAND__STAND_HPP_
