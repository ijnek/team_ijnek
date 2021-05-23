#ifndef BALANCED_MOTION__MOTION_HPP_
#define BALANCED_MOTION__MOTION_HPP_

#include "balanced_motion_2/state.hpp"
#include "nao_interfaces/msg/joints.hpp"

class Motion
{
public:
    Motion(std::string name) : name(name){}

    // First argument is the "aim" state, and the second argument is the sensor_joint angles
    virtual nao_interfaces::msg::Joints makeJoints (State &, State &, nao_interfaces::msg::Joints &)
    {
        return nao_interfaces::msg::Joints{};
    }

    // Can we achieve a certain state after the motion?
    virtual bool isAchievable(State &)
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