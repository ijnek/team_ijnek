#ifndef BALANCED_MOTION_2__WALK_HPP_
#define BALANCED_MOTION_2__WALK_HPP_

#include "balanced_motion_2/motion.hpp"

class Walk : public Motion
{
public:
    Walk() : Motion("Walk") {}

    bool isAchievable(State &state)
    {
        if (state.diving.has_value() && state.diving.value() == true){
            return false;
        }
        if (state.standing.has_value() && state.standing.value() == true){
            return false;
        }
        return true;
    }

    State requirements()
    {
        State requirements;
        requirements.getting_up = false;
        requirements.on_ground = false;
        requirements.diving = false;
        return requirements;
    }
};

#endif  // BALANCED_MOTION_2__WALK_HPP_