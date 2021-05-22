#ifndef BALANCED_MOTION__TRANSITIONER_HPP_
#define BALANCED_MOTION__TRANSITIONER_HPP_

#include "balanced_motion_2/motion.hpp"
#include "nao_interfaces/msg/joints.hpp"

class MotionTransitioner
{
public:
  void addMotion(Motion * motion)
  {
    motions.push_back(motion);
  }

  std::tuple<Motion *, State> findNextMotion(State current, State aim)
  {
    std::vector<Motion *> motionsToSearch = motions;
    std::vector<Motion *> motionsLeadingToAim;
    std::vector<Motion *> motionsAddedInLastIteration;
    std::vector<std::tuple<Motion *, State>> motionsAddedInLastIterationWithAim;
    std::vector<Motion *> possibleNextMotions;

    do {
      motionsAddedInLastIteration = {};
      motionsAddedInLastIterationWithAim = {};

      std::cout << "motionsToSearch: ";
      for (Motion *motion : motionsToSearch)
      {
        std::cout << motion->name << ", ";
      }
      std::cout << std::endl;

      for (Motion * motion : motionsToSearch) {

        if (motionsLeadingToAim.size() == 0) {
          // We're in the first iteration
          if (motion->isAchievable(aim)) {
            motionsAddedInLastIteration.push_back(motion);
            motionsAddedInLastIterationWithAim.push_back(std::make_tuple(motion, aim));
          }
        } else {
          // We're in the second, third, fourth... iteration
          for (Motion * a : motionsLeadingToAim) {
            State aim = a->requirements();
            if (motion->isAchievable(aim)) {
              motionsAddedInLastIteration.push_back(motion);
              motionsAddedInLastIterationWithAim.push_back(std::make_tuple(motion, aim));
            }
          }
        }
      }

      std::cout << "motionsAddedInLastIteration: ";
      for (Motion *motion : motionsAddedInLastIteration)
      {
        std::cout << motion->name << ", ";
      }
      std::cout << std::endl;

      // If we've found a motion that we can execute right now, return!
      for (auto [motion, aim] : motionsAddedInLastIterationWithAim) {
        State requirements = motion->requirements();
        if (current.satisfies(requirements)) {
          return std::make_tuple(motion, aim);
        }
      }

      // Remove from motionsToSearch
      for (Motion * motion : motionsAddedInLastIteration) {
        auto position = std::find(motionsToSearch.begin(), motionsToSearch.end(), motion);
        if (position != motionsToSearch.end()) {
          motionsToSearch.erase(position);
        }
      }

      motionsLeadingToAim.insert(
        std::end(motionsLeadingToAim),
        std::begin(motionsAddedInLastIteration), std::end(motionsAddedInLastIteration));

      std::cout << "motionsLeadingToAim: ";
      for (Motion *motion : motionsLeadingToAim)
      {
        std::cout << motion->name << ", ";
      }
      std::cout << std::endl;

    } while (motionsAddedInLastIteration.size() != 0);

    std::cout << "COULDN'T FIND A VALID TRANSITION TO REACH AIM" << std::endl;
    return std::make_tuple(motions.at(0), State{});
  }

private:
  std::vector<Motion *> motions;
};


#endif  // BALANCED_MOTION__TRANSITIONER_HPP_
