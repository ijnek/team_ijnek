#ifndef BALANCED_MOTION__MOTION_HPP_
#define BALANCED_MOTION__MOTION_HPP_

#include "balanced_motion_2/state.hpp"
#include "nao_interfaces/msg/joints.hpp"

class Motion
{
public:
    Motion(std::string name) : name(name){}

    virtual nao_interfaces::msg::Joints makeJoints (State state, nao_interfaces::msg::Joints & sensor_joints)
    {
        return nao_interfaces::msg::Joints{};
    }

    // Can we achieve a certain state after the motion?
    virtual bool isAchievable(State &aim)
    {
        return false;
    }

    // what are the requirements, for starting the motion?
    virtual State requirements() {
        return State{};
    }

    // Called when motion transitions to another motion
    virtual void reset() {
    }

    const std::string name;
};

# endif  // BALANCED_MOTION__MOTION_HPP_