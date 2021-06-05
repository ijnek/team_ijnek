#ifndef MOTION_TRANSITIONER__MOTION_TRANSITIONER_HPP_
#define MOTION_TRANSITIONER__MOTION_TRANSITIONER_HPP_

#include <functional>
#include "motion_transitioner/requests.hpp"

class MotionTransitioner
{
public:
  MotionTransitioner(
    std::function<void(GetupRequest)> startGetup,
    std::function<void(KickRequest)> startKick,
    std::function<void(CrouchRequest)> startCrouch);
  void request(GetupRequest req);
  void request(KickRequest req);
  void request(WalkRequest req);
  void request(CrouchRequest req);
  void notifyGetupDone();
  void notifyKickDone();

private:
  std::function<void(GetupRequest)> startGetup;
  std::function<void(KickRequest)> startKick;
  std::function<void(CrouchRequest)> startCrouch;

  bool duringGetup = false;
  bool duringKick = false;
};

#endif  // MOTION_TRANSITIONER__MOTION_TRANSITIONER_HPP_
