#ifndef BALANCED_MOTION_2__GETUP_HPP_
#define BALANCED_MOTION_2__GETUP_HPP_

#include "balanced_motion_2/motion.hpp"
#include "balanced_motion_2/linear.hpp"

class Getup : public Linear
{
public:
    Getup() : Linear("Getup", "getupBack.pos") {}

    bool isAchievable(State &aim) override
    {
        if (aim.diving.has_value() && aim.diving.value() == true){
            return false;
        }
        if (aim.walking.has_value() && aim.walking.value() == true){
            return false;
        }
        return true;
    }

    State requirements() override
    {
        State requirements;
        requirements.on_ground = true;
        return requirements;
    }
};

#endif  // BALANCED_MOTION_2__GETUP_HPP_