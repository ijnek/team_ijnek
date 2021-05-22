#ifndef BALANCED_MOTION_2__GETUP_HPP_
#define BALANCED_MOTION_2__GETUP_HPP_

#include "balanced_motion_2/motion.hpp"

class Getup : public Motion
{
public:
    Getup() : Motion("Getup") {}

    bool isAchievable(State &state)
    {
        if (state.diving.has_value() && state.diving.value() == true){
            return false;
        }
        if (state.walking.has_value() && state.walking.value() == true){
            return false;
        }
        return true;
    }

    State requirements()
    {
        State requirements;
        requirements.getting_up = false;
        requirements.on_ground = true;
        return requirements;
    }
};

#endif  // BALANCED_MOTION_2__GETUP_HPP_