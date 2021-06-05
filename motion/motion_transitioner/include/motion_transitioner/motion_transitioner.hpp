#ifndef MOTION_TRANSITIONER__MOTION_TRANSITIONER_HPP_
#define MOTION_TRANSITIONER__MOTION_TRANSITIONER_HPP_

#include "motion_transitioner/requests.hpp"

class MotionTransitioner
{
public:
  MotionTransitioner(
    std::function<void(GetupRequest)> startGetup,
    std::function<void(KickRequest)> startKick)
  : startGetup(startGetup), startKick(startKick) {}
  void request(GetupRequest req) {duringGetup = true; startGetup(req);}
  void request(KickRequest req) {if (!duringGetup) {startKick(req);}}
  void notifyGetupDone() {duringGetup = false;}
  void notifyKickDone() {}

private:
  std::function<void(GetupRequest)> startGetup;
  std::function<void(KickRequest)> startKick;

  bool duringGetup = false;
};

#endif  // MOTION_TRANSITIONER__MOTION_TRANSITIONER_HPP_
