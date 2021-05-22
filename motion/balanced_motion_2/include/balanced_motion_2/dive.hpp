#ifndef BALANCED_MOTION_2__DIVE_HPP_
#define BALANCED_MOTION_2__DIVE_HPP_

#include "balanced_motion_2/motion.hpp"
#include "balanced_motion_2/linear.hpp"

class Dive : public Linear
{
public:
    Dive(int dt_ms) : Linear("Dive", "diveLeft.pos", dt_ms){}

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