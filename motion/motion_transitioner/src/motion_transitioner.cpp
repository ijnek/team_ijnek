#include "motion_transitioner/motion_transitioner.hpp"

MotionTransitioner::MotionTransitioner(
    std::function<void(GetupRequest)> startGetup,
    std::function<void(KickRequest)> startKick,
    std::function<void(CrouchRequest)> startCrouch)
  : startGetup(startGetup), startKick(startKick), startCrouch(startCrouch)
{

}

void MotionTransitioner::request(GetupRequest req)
{
  duringGetup = true;
  startGetup(req);
}

void MotionTransitioner::request(KickRequest req)
{
  if (!duringGetup)
  {
    duringKick = true;
    startKick(req);
  }
}

void MotionTransitioner::notifyGetupDone()
{
  duringGetup = false;
  startCrouch(CrouchRequest{});
}

void MotionTransitioner::notifyKickDone()
{
  duringKick = false;
  startCrouch(CrouchRequest{});
}