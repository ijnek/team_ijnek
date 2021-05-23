#ifndef BALANCED_MOTION_2__MOTION_HELPERS_HPP_
#define BALANCED_MOTION_2__MOTION_HELPERS_HPP_

#include "geometry_msgs/msg/twist.hpp"

bool twistIsZero(geometry_msgs::msg::Twist & twist)
{
  return
    twist.linear.x == 0 &&
    twist.linear.y == 0 &&
    twist.linear.z == 0 &&
    twist.angular.x == 0 &&
    twist.angular.y == 0 &&
    twist.angular.z == 0;
}

#endif  // BALANCED_MOTION_2__MOTION_HELPERS_HPP_
