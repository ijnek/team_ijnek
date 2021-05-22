#ifndef BALANCED_MOTION_2__GETUP_HPP_
#define BALANCED_MOTION_2__GETUP_HPP_

#include "balanced_motion_2/motion.hpp"

class Getup : public Motion
{
public:
    Getup() : Motion("Getup") {}

    nao_interfaces::msg::Joints makeJoints (State state)
    {
        counter++;
        return nao_interfaces::msg::Joints{};
    }

    bool isAchievable(State &aim)
    {
        if (aim.diving.has_value() && aim.diving.value() == true){
            return false;
        }
        if (aim.walking.has_value() && aim.walking.value() == true){
            return false;
        }
        return true;
    }

    State requirements()
    {
        State requirements;
        requirements.on_ground = true;
        return requirements;
    }

    bool hasFinished()
    {
        return counter > 3;
    }

    void reset()
    {
        counter = 0;
    }

private:
    int counter;

};

#endif  // BALANCED_MOTION_2__GETUP_HPP_