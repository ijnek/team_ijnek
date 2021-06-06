#include "motion_transitioner/motion_transitioner.hpp"
#include <iostream>

MotionTransitioner::MotionTransitioner(
    std::function<void(Getup)> startGetup,
    std::function<void(Kick)> startKick,
    std::function<void(Crouch)> startCrouch)
  : startGetup(startGetup), startKick(startKick), startCrouch(startCrouch)
{

}

void MotionTransitioner::request(Getup req)
{
  duringGetup = true;
  startGetup(req);
}

void MotionTransitioner::request(Kick req)
{
  if (!duringGetup)
  {
    duringKick = true;
    startKick(req);
  }
}

void MotionTransitioner::request(Crouch)
{
  std::cout << "not impelmented" << std::endl;
}


void MotionTransitioner::notifyGetupDone()
{
  duringGetup = false;
  startCrouch(Crouch{});
}

void MotionTransitioner::notifyKickDone()
{
  duringKick = false;
  startCrouch(Crouch{});
}