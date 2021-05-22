#ifndef BALANCED_MOTION__STATE_HPP_
#define BALANCED_MOTION__STATE_HPP_

#include <optional>
#include "geometry_msgs/msg/twist.hpp"

class State
{
public:
  std::optional<bool> standing;  // Robot is not moving feet, and is standing
  std::optional<bool> walking;  // Robot is moving weight across from left to right foot, not necessarily travelling about
  std::optional<bool> travelling;  // Robot is moving while walking around
  std::optional<bool> diving;
  std::optional<bool> on_ground;
  std::optional<bool> getting_up;

  bool satisfies(State &requirements)
  {
    if (requirements.standing.has_value())
    {
      if (!this->standing.has_value())
      {
        return false;
      }

      if (this->standing.value() != requirements.standing.value())
      {
        return false;
      }
    }

    if (requirements.walking.has_value())
    {
      if (!this->walking.has_value())
      {
        return false;
      }

      if (this->walking.value() != requirements.walking.value())
      {
        return false;
      }
    }

    if (requirements.diving.has_value())
    {
      if (!this->diving.has_value())
      {
        return false;
      }

      if (this->diving.value() != requirements.diving.value())
      {
        return false;
      }
    }

    if (requirements.on_ground.has_value())
    {
      if (!this->on_ground.has_value())
      {
        return false;
      }

      if (this->on_ground.value() != requirements.on_ground.value())
      {
        return false;
      }
    }

    if (requirements.getting_up.has_value())
    {
      if (!this->getting_up.has_value())
      {
        return false;
      }

      if (this->getting_up.value() != requirements.getting_up.value())
      {
        return false;
      }
    }

    if (requirements.travelling.has_value())
    {
      if (!this->travelling.has_value())
      {
        return false;
      }

      if (this->travelling.value() != requirements.travelling.value())
      {
        return false;
      }
    }

    return true;
  }

};

#endif  // BALANCED_MOTION__STATE_HPP_