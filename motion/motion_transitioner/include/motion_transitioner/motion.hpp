#ifndef BALANCED_MOTION__MOTION_HPP_
#define BALANCED_MOTION__MOTION_HPP_

#include <string>
#include <set>

class Motion
{
public:
    Motion(std::string name) : name(name){}

    const std::string name;
    std::set<std::shared_ptr<Motion>> canTransitionFrom;
};

# endif  // BALANCED_MOTION__MOTION_HPP_