#ifndef BALANCED_MOTION_2__DIVE_HPP_
#define BALANCED_MOTION_2__DIVE_HPP_

#include "balanced_motion_2/motion.hpp"

class Dive : public Motion
{
public:
    Dive() : Motion("Dive") {}

    bool isAchievable(State &state)
    {
        if (state.diving.has_value() && state.diving.value() == true)
        {
            return true;
        }
        return false;
    }

    State requirements()
    {
        State requirements;
        requirements.diving = false;
        requirements.on_ground = false;
        requirements.getting_up = false;
        return requirements;
    }
};

#endif  // BALANCED_MOTION_2__DIVE_HPP_