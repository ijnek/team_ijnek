// This file is based on UNSW Sydney's codebase, but has been modified significantly.
// Both copyright notices are provided below.
//
// Copyright (c) 2018 UNSW Sydney.  All rights reserved.
//
// Licensed under Team rUNSWift's original license. See the "LICENSE-runswift"
// file to obtain a copy of the license.
//
// ---------------------------------------------------------------------------------
//
// Copyright 2021 Kenji Brameld
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "kick/kick.hpp"
#include <iostream>
#include <cmath>

#define SHIFT_PERIOD 2.6
#define SHIFT_END_PERIOD 2.5
#define BACK_PHASE 0.70
#define KICK_PHASE 0.10
#define THROUGH_PHASE 0.30
#define END_PHASE 0.0
#define TOTAL_PHASE (BACK_PHASE + KICK_PHASE + THROUGH_PHASE + END_PHASE)

#define KICK_STEP_HEIGHT 0.065  // how far to lift kicking foot

float parabolicReturn(float f);
float parabolicStep(float dt, float time, float period, float deadTimeFraction = 0);
float interpolateSmooth(float start, float end, float tCurrent, float tEnd);
inline static float RAD2DEG(const float x);
inline static float DEG2RAD(const float x);

Kick::Kick(
  std::function<void(void)> notifyKickDone,
  std::function<void(motion_interfaces::msg::IKCommand)> sendIKCommand)
: notifyKickDone(notifyKickDone), sendIKCommand(sendIKCommand)
{
}

void Kick::start(bool use_left_foot)
{
  if (!duringKick) {
    duringKick = true;
    // receivedMsg = req;
    this->use_left_foot = use_left_foot;
    kickT = 0;
  }
}

void Kick::notifyJoints(nao_sensor_msgs::msg::JointPositions)
{
  if (duringKick) {
    motion_interfaces::msg::IKCommand command;
    command.hiph = 0.23;

    kickT += 0.02;

    float sideAmp = 0;
    float kickAmp = -0.07;
    float power = 1.0;
    float kickPower = pow(power, 1.7);
    float kickBackAmp = -kickAmp * kickPower;

    float rock = 0;
    float kickingLean = 20.5;
    float footh = 0;
    float kickStepH = KICK_STEP_HEIGHT;

    float swingDelayFactor = 0.2;

    float & forwardDist = use_left_foot ? command.forward_l : command.forward_r;
    float & side = use_left_foot ? command.left_l : command.left_r;

    if (kickT < BACK_PHASE) {
      float totalShift = SHIFT_PERIOD / 4;
      float halfShift = totalShift / 2;

      // shift weight sideways
      if (kickT < totalShift) {
        rock = DEG2RAD(kickingLean) * parabolicStep(kickT, totalShift, 0);
      }

      // only start lifting the kicking at 1/3
      if (kickT >= totalShift / 3) {
        float t3 = kickT - totalShift / 3;
        float endT = BACK_PHASE - totalShift / 3;
        footh = kickStepH * parabolicStep(t3, endT, 0);
      }

      // Once we're halfway through shifting our weight, start moving the foot back.
      if (kickT >= halfShift) {
        float kickT2 = kickT - halfShift;
        float endT = BACK_PHASE - halfShift;
        forwardDist = interpolateSmooth(0, kickBackAmp, kickT2, endT);
        side = interpolateSmooth(0, sideAmp, kickT2, endT);
      }
    } else if (kickT < (BACK_PHASE + KICK_PHASE)) {
      if (kickT >= BACK_PHASE + swingDelayFactor * KICK_PHASE) {
        forwardDist = interpolateSmooth(
          kickBackAmp, kickAmp,
          kickT - BACK_PHASE - swingDelayFactor * KICK_PHASE,
          KICK_PHASE);
      }
      side = sideAmp;
      // Hold...
    } else if (kickT < (BACK_PHASE + KICK_PHASE + THROUGH_PHASE)) {
      // Keep hip balance
      forwardDist = kickAmp;
      side = sideAmp;
    } else if (kickT < (TOTAL_PHASE + SHIFT_END_PERIOD / 4)) {
      forwardDist = lastKickForward - lastKickForward * parabolicStep(
        kickT - TOTAL_PHASE,
        SHIFT_END_PERIOD / 8, 0);
      side = lastSide - lastSide * parabolicStep(kickT - TOTAL_PHASE, SHIFT_END_PERIOD / 8, 0);
      rock = lastRock - lastRock * parabolicStep(kickT - TOTAL_PHASE, SHIFT_END_PERIOD / 4, 0);
      footh = lastFooth - lastFooth * parabolicStep(kickT - TOTAL_PHASE, SHIFT_END_PERIOD / 6, 0);
    } else {
      kickT = 0;
      rock = 0;
      footh = 0;
      duringKick = false;
      notifyKickDone();
    }

    // aborting or ending a kick from these values
    if (kickT < (BACK_PHASE + KICK_PHASE + THROUGH_PHASE)) {
      lastKickForward = forwardDist;
      lastSide = side;
      lastFooth = footh;
      lastRock = rock;
    }

    // hack because rock isn't properly supported yet.
    if (use_left_foot) {
      command.left_l += rock / 8;
      command.left_r += rock / 8;
    } else {
      command.left_l -= rock / 8;
      command.left_r -= rock / 8;
    }

    sendIKCommand(command);
  }
}

float parabolicStep(float dt, float time, float period, float deadTimeFraction)
{
  // normalised [0,1] step up
  float deadTime = period * deadTimeFraction / 2;
  if (time < deadTime + dt / 2) {
    return 0;
  }
  if (time > period - deadTime - dt / 2) {
    return 1;
  }
  float timeFraction = (time - deadTime) / (period - 2 * deadTime);
  if (time < period / 2) {
    return 2.0 * timeFraction * timeFraction;
  }
  return 4 * timeFraction - 2 * timeFraction * timeFraction - 1;
}

float parabolicReturn(float f)   // normalised [0,1] up and down
{
  double x = 0;
  double y = 0;
  if (f < 0.25f) {
    y = 8 * f * f;
  }
  if (f >= 0.25f && f < 0.5f) {
    x = 0.5f - f;
    y = 8 * x * x;
    y = 1.0f - y;
  }
  if (f >= 0.5f && f < 0.75f) {
    x = f - 0.5f;
    y = 8 * x * x;
    y = 1.0f - y;
  }
  if (f >= 0.75f && f <= 1.0f) {
    x = 1.0f - f;
    y = 8 * x * x;
  }
  return y;
}

float interpolateSmooth(float start, float end, float tCurrent, float tEnd)
{
  return start + (end - start) * (1 + cos(M_PI * tCurrent / tEnd - M_PI)) / 2;
}

static const float DEG_OVER_RAD = 180 / M_PI;
static const float RAD_OVER_DEG = M_PI / 180;

inline static float RAD2DEG(const float x)
{
  return (x) * DEG_OVER_RAD;
}

inline static float DEG2RAD(const float x)
{
  return (x) * RAD_OVER_DEG;
}
