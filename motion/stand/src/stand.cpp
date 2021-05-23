#include "stand/stand.hpp"
#include "motion_msgs/msg/ik_command.hpp"
#include "balanced_motion_2/inverse_kinematics.hpp"

#define CROUCHING_HIPH 0.23

nao_interfaces::msg::Joints Stand::makeJoints()
{
  motion_msgs::msg::IKCommand command;
  command.hiph = CROUCHING_HIPH;
  return calculate_joints(command);
}
