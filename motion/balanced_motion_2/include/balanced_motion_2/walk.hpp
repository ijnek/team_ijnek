#ifndef BALANCED_MOTION_2__WALK_HPP_
#define BALANCED_MOTION_2__WALK_HPP_

#include "balanced_motion_2/motion.hpp"

class Walk : public Motion
{
public:
    Walk() : Motion("Walk") {}

    bool isAchievable(State &aim) override
    {
        if (aim.diving.has_value() && aim.diving.value() == true){
            return false;
        }
        if (aim.standing.has_value() && aim.standing.value() == true){
            return false;
        }
        return true;
    }

    State requirements() override
    {
        State requirements;
        requirements.on_feet = true;
        return requirements;
    }

    bool isTravelling()
    {
        return false;
    }
};

#endif  // BALANCED_MOTION_2__WALK_HPP_