#ifndef BALANCED_MOTION_2__DIVE_HPP_
#define BALANCED_MOTION_2__DIVE_HPP_

#include "balanced_motion_2/motion.hpp"

class Dive : public Motion
{
public:
    Dive() : Motion("Dive") {}

    bool isAchievable(State &aim)
    {
        if (aim.diving.has_value() && aim.diving.value() == true)
        {
            return true;
        }
        return false;
    }

    State requirements()
    {
        State requirements;
        requirements.on_feet = true;
        return requirements;
    }
};

#endif  // BALANCED_MOTION_2__DIVE_HPP_