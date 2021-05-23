#ifndef BALANCED_MOTION__STATE_HPP_
#define BALANCED_MOTION__STATE_HPP_

#include <optional>
#include "geometry_msgs/msg/twist.hpp"
#include "motion_msgs/msg/ik_command.hpp"

class State
{
public:
  std::optional<bool> standing;  // Robot is not moving feet, and is standing
  std::optional<bool> walking;  // Robot is moving weight across from left to right foot, not necessarily travelling about
  std::optional<bool> diving;
  std::optional<bool> on_ground;
  std::optional<bool> getting_up;
  std::optional<bool> on_feet;
  std::optional<geometry_msgs::msg::Twist> twist;
  std::optional<motion_msgs::msg::IKCommand> ik_command;

  State(bool initialise_as_false = false)
  {
    if (initialise_as_false) {
      standing = false;
      walking = false;
      diving = false;
      on_ground = false;
      getting_up = false;
      on_feet = false;
    }
  }

  bool satisfies(State & requirements)
  {
    if (requirements.standing.has_value()) {
      if (!this->standing.has_value()) {
        return false;
      }

      if (this->standing.value() != requirements.standing.value()) {
        return false;
      }
    }

    if (requirements.walking.has_value()) {
      if (!this->walking.has_value()) {
        return false;
      }

      if (this->walking.value() != requirements.walking.value()) {
        return false;
      }
    }

    if (requirements.diving.has_value()) {
      if (!this->diving.has_value()) {
        return false;
      }

      if (this->diving.value() != requirements.diving.value()) {
        return false;
      }
    }

    if (requirements.on_ground.has_value()) {
      if (!this->on_ground.has_value()) {
        return false;
      }

      if (this->on_ground.value() != requirements.on_ground.value()) {
        return false;
      }
    }

    if (requirements.getting_up.has_value()) {
      if (!this->getting_up.has_value()) {
        return false;
      }

      if (this->getting_up.value() != requirements.getting_up.value()) {
        return false;
      }
    }

    if (requirements.on_feet.has_value()) {
      if (!this->on_feet.has_value()) {
        return false;
      }

      if (this->on_feet.value() != requirements.on_feet.value()) {
        return false;
      }
    }

    if (requirements.twist.has_value()) {
      if (!this->twist.has_value()) {
        return false;
      }

      geometry_msgs::msg::Twist & twist1 = this->twist.value();
      geometry_msgs::msg::Twist & twist2 = requirements.twist.value();
      if (!close(twist1.linear.x, twist2.linear.x) ||
        !close(twist1.linear.y, twist2.linear.y) ||
        !close(twist1.linear.z, twist2.linear.z) ||
        !close(twist1.angular.x, twist2.angular.x) ||
        !close(twist1.angular.y, twist2.angular.y) ||
        !close(twist1.angular.z, twist2.angular.z))
      {
        return false;
      }
    }

    return true;
  }


  bool close(float a, float b)
  {
    return abs(a - b) < 0.000001;
  }

};

#endif  // BALANCED_MOTION__STATE_HPP_
