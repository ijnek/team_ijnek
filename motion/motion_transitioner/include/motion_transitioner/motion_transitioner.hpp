#ifndef MOTION_TRANSITIONER__MOTION_TRANSITIONER_HPP_
#define MOTION_TRANSITIONER__MOTION_TRANSITIONER_HPP_

#include <set>
#include "motion_transitioner/motion.hpp"

std::shared_ptr<Motion> findNextMotion(std::shared_ptr<Motion> current, std::shared_ptr<Motion> aim)
{
  if (aim == current)
  {
    return nullptr;
  }

  std::set<std::shared_ptr<Motion>> toSearch({aim});    
  std::set<std::shared_ptr<Motion>> searched;

  while (toSearch.size() != 0)
  {
    searched.insert(toSearch.begin(), toSearch.end());

    std::set<std::shared_ptr<Motion>> newToSearch;
    for (std::shared_ptr<Motion> motion : toSearch) {
      for (std::shared_ptr<Motion> m : motion->canTransitionFrom)
      {
        if (m == current)
        {
          return motion;
        }

        if (searched.count(m) == 0)
        {
          newToSearch.insert(m);
        }
      }
    }

    toSearch = newToSearch;
  }

  return nullptr;
}

#endif  // MOTION_TRANSITIONER__MOTION_TRANSITIONER_HPP_
