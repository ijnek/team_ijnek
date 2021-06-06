#ifndef MOTION_TRANSITIONER__MOTION_TRANSITIONER_HPP_
#define MOTION_TRANSITIONER__MOTION_TRANSITIONER_HPP_

#include <functional>
#include "motion_msgs/msg/kick.hpp"
#include "motion_msgs/msg/getup.hpp"
#include "motion_msgs/msg/crouch.hpp"

using namespace motion_msgs::msg;

class MotionTransitioner
{
public:
  MotionTransitioner(
    std::function<void(Getup)> startGetup,
    std::function<void(Kick)> startKick,
    std::function<void(Crouch)> startCrouch);
  void request(Getup req);
  void request(Kick req);
  // void request(WalkRequest req);
  void request(Crouch req);
  void notifyGetupDone();
  void notifyKickDone();

private:
  std::function<void(Getup)> startGetup;
  std::function<void(Kick)> startKick;
  std::function<void(Crouch)> startCrouch;

  bool duringGetup = false;
  bool duringKick = false;
};

#endif  // MOTION_TRANSITIONER__MOTION_TRANSITIONER_HPP_
