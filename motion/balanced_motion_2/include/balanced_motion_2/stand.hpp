#ifndef BALANCED_MOTION_2__STAND_HPP_
#define BALANCED_MOTION_2__STAND_HPP_

#include "balanced_motion_2/motion.hpp"

class Stand : public Motion
{
public:
    Stand() : Motion("Stand") {}

    bool isAchievable(State &state)
    {
        if (state.standing.has_value() && state.standing.value() == true)
        {
            return true;
        }
        return false;
    }

    State requirements()
    {
        State requirements;
        requirements.getting_up = false;
        requirements.travelling = false;
        return requirements;
    }
};

#endif  // BALANCED_MOTION_2__STAND_HPP_