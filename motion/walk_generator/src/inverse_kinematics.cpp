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

#include "walk_generator/inverse_kinematics.hpp"

static const float thigh = 0.10000;
static const float tibia = 0.10290;
static const float ankle = 0.04519;

InverseKinematics::InverseKinematics()
: Node("InverseKinematics")
{
  sub_ik_command =
    create_subscription<motion_msgs::msg::IKCommand>(
    "motion/ik_command", 1,
    [this](motion_msgs::msg::IKCommand::SharedPtr ik_command) {
      nao_interfaces::msg::Joints joints = calculate_joints(*ik_command);
      pub_joints->publish(joints);
    });

  pub_joints = this->create_publisher<nao_interfaces::msg::Joints>("effectors/joints", 10);
}

nao_interfaces::msg::Joints InverseKinematics::calculate_joints(
  motion_msgs::msg::IKCommand & ik_command)
{
  // 9. Work out joint angles from walk variables above
  float & hiph = ik_command.hiph;
  float & foothL = ik_command.footh_l;
  float & foothR = ik_command.footh_r;
  float & forwardL = ik_command.forward_l;
  float & forwardR = ik_command.forward_r;
  float & leftL = ik_command.left_l;
  float & leftR = ik_command.left_r;
  float & turnRL = ik_command.turn_rl;
  float & comOffset = ik_command.com_offset;

  // 9.05 Sert hip and ankle values
  float HrL = atan(leftL / (hiph - ankle));
  float HrR = atan(leftR / (hiph - ankle));
  float ArL = -HrL;
  float ArR = -HrR;

  // 9.1 Left foot closed form inverse kinematics
  float leghL = hiph - foothL - ankle;   // vertical height between ankle and hip in meters
  float legX0L = leghL / cos(HrL);   // leg extension (eliminating knee) when forwardL = 0
  // leg extension at forwardL
  float legXL = sqrt(legX0L * legX0L + (forwardL + comOffset) * (forwardL + comOffset));
  if (legXL > thigh + tibia) {
    RCLCPP_ERROR(
      get_logger(), "(Walk2014Generator) cannot solve Inverse Kinematics for that foot position!");
  }
  // acute angles at hip in thigh-tibia triangle
  float beta1L = acos((thigh * thigh + legXL * legXL - tibia * tibia) / (2.0f * thigh * legXL));
  float beta2L = acos((tibia * tibia + legXL * legXL - thigh * thigh) / (2.0f * tibia * legXL));

  float tempL = legX0L / legXL;
  if (tempL > 1.0f) {
    // sin ratio to calculate leg extension pitch. If > 1 due to numerical error round down.
    tempL = 1.0f;
  }
  float deltaL = asin(tempL);                             // leg extension angle
  float dirL = 1.0f;
  if ((forwardL + comOffset) > 0.0f) {
    dirL = -1.0f;     // signum of position of foot
  }
  // Hip pitch is sum of leg-extension + hip acute angle above
  float HpL = beta1L + dirL * (M_PI / 2.0f - deltaL);
  // Ankle pitch is a similar calculation for the ankle joint
  float ApL = beta2L + dirL * (deltaL - M_PI / 2.0f);
  // to keep torso upright with both feet on the ground,
  // the knee pitch is always the sum of the hip pitch and the ankle pitch.
  float KpL = HpL + ApL;

  // 9.2 right foot closed form inverse kinematics (comments as above but for right leg)
  float leghR = hiph - foothR - ankle;
  float legX0R = leghR / cos(HrR);
  float legXR = sqrt(legX0R * legX0R + (forwardR + comOffset) * (forwardR + comOffset));
  float dirR = 1.0f;
  if ((forwardR + comOffset) > 0.0f) {
    dirR = -1.0f;
  }
  if (legXR > thigh + tibia) {
    RCLCPP_ERROR(
      get_logger(), "(Walk2014Generator) cannot solve Inverse Kinematics for that foot position!");
  }
  float beta1R = acos((thigh * thigh + legXR * legXR - tibia * tibia) / (2.0f * thigh * legXR));
  float beta2R = acos((tibia * tibia + legXR * legXR - thigh * thigh) / (2.0f * tibia * legXR));
  float tempR = legX0R / legXR;
  if (tempR > 1.0f) {
    tempR = 1.0f;
  }
  float deltaR = asin(tempR);
  float HpR = beta1R + dirR * (M_PI / 2.0f - deltaR);
  float ApR = beta2R + dirR * (deltaR - M_PI / 2.0f);
  float KpR = HpR + ApR;

  // 9.3 Sert hip and ankle values -> Moved to 9.05

  // 9.35 Add kick rocking
  // if (walk2014Option == KICK) {
  //     HrL += rock;
  //     HrR += rock;
  //     ArL -= rock;
  //     ArR -= rock;
  // }

  // 9.4 Adjust HpL, HrL, ApL, ArL LEFT based on Hyp turn to keep ankle in situ
  // Turning
  float z = 0;
  geometry_msgs::msg::Point tL = mf2b(z, -HpL, HrL, KpL, -ApL, ArL, z, z, z);
  geometry_msgs::msg::Point sL;
  float Hyp = -turnRL;
  for (int i = 0; i < 3; i++) {
    sL = mf2b(Hyp, -HpL, HrL, KpL, -ApL, ArL, z, z, z);
    geometry_msgs::msg::Point e;
    e.x = tL.x - sL.x;
    e.y = tL.y - sL.y;
    e.z = tL.z - sL.z;
    Hpr hpr = hipAngles(Hyp, -HpL, HrL, KpL, -ApL, ArL, z, z, z, e);
    HpL -= hpr.Hp;
    HrL += hpr.Hr;
  }
  // ApL and ArL to make sure LEFT foot is parallel to ground
  geometry_msgs::msg::Point up = mf2b(Hyp, -HpL, HrL, KpL, -ApL, ArL, 1.0f, 0.0f, 0.0f);
  geometry_msgs::msg::Point ur = mf2b(Hyp, -HpL, HrL, KpL, -ApL, ArL, 0.0f, 1.0f, 0.0f);
  ApL = ApL + asin(sL.z - up.z);
  ArL = ArL + asin(sL.z - ur.z);

  // 9.5 Adjust HpR, HrR, ApR, ArR (RIGHT) based on Hyp turn to keep ankle in situ
  // Map to LEFT - we reuse the left foot IK because of symmetry right foot
  float Hr = -HrR;
  float Ar = -ArR;
  // Target foot origin in body coords
  geometry_msgs::msg::Point t = mf2b(z, -HpR, Hr, KpR, -ApR, Ar, z, z, z);
  geometry_msgs::msg::Point s;
  Hyp = -turnRL;
  for (int i = 0; i < 3; i++) {
    s = mf2b(Hyp, -HpR, Hr, KpR, -ApR, Ar, z, z, z);
    geometry_msgs::msg::Point e;
    e.x = t.x - s.x;
    e.y = t.y - s.y;
    e.z = t.z - s.z;
    Hpr hpr = hipAngles(Hyp, -HpR, Hr, KpR, -ApR, Ar, z, z, z, e);
    HpR -= hpr.Hp;
    Hr += hpr.Hr;
  }
  // 9.6 Ap and Ar to make sure foot is parallel to ground
  geometry_msgs::msg::Point u1 = mf2b(Hyp, -HpR, Hr, KpR, -ApR, Ar, 1.0f, 0.0f, 0.0f);
  geometry_msgs::msg::Point u2 = mf2b(Hyp, -HpR, Hr, KpR, -ApR, Ar, 0.0f, 1.0f, 0.0f);
  ApR = ApR + asin(s.z - u1.z);
  Ar = Ar + asin(s.z - u2.z);
  // map back from left foot to right foot
  HrR = -Hr;
  ArR = -Ar;

  // 10. Set joint values
  nao_interfaces::msg::Joints j;
  // 10.2 Turn
  j.angles[nao_interfaces::msg::Joints::LHIPYAWPITCH] = -turnRL;

  // 10.3 Sagittal Joints
  j.angles[nao_interfaces::msg::Joints::LHIPPITCH] = -HpL;
  j.angles[nao_interfaces::msg::Joints::RHIPPITCH] = -HpR;
  j.angles[nao_interfaces::msg::Joints::LKNEEPITCH] = KpL;
  j.angles[nao_interfaces::msg::Joints::RKNEEPITCH] = KpR;

  // Only activate balance control if foot is on the ground
  j.angles[nao_interfaces::msg::Joints::LANKLEPITCH] = -ApL;
  j.angles[nao_interfaces::msg::Joints::RANKLEPITCH] = -ApR;

  // 10.4 Coronal Joints
  j.angles[nao_interfaces::msg::Joints::LHIPROLL] = HrL;
  j.angles[nao_interfaces::msg::Joints::RHIPROLL] = HrR;
  j.angles[nao_interfaces::msg::Joints::LANKLEROLL] = ArL;
  j.angles[nao_interfaces::msg::Joints::RANKLEROLL] = ArR;

  return j;
}

geometry_msgs::msg::Point InverseKinematics::mf2b(
  float Hyp, float Hp, float Hr, float Kp, float Ap,
  float Ar, float xf, float yf, float zf)
{
  // MFOOT2BODY Transform coords from foot to body.
  // This code originates from 2010 using symbolic equations in Matlab
  // to perform the coordinate transforms - see team report (BH)
  // In future this approach to IK for the Nao should be reworked in closed form,
  // significantly reducing the size of the code the
  // the computational complexity (BH)
  geometry_msgs::msg::Point result;
  float pi = M_PI;
  float tibia_mm = tibia * 1000;
  float thigh_mm = thigh * 1000;
  float k = sqrt(2.0);
  float c1 = cos(Ap);
  float c2 = cos(Hr + pi / 4.0);
  float c3 = cos(Hyp - pi / 2.0);
  float c4 = cos(Hp);
  float c5 = cos(Kp);
  float c6 = cos(Ar - pi / 2.0);
  float s1 = sin(Kp);
  float s2 = sin(Hp);
  float s3 = sin(Hyp - 1.0 / 2.0 * pi);
  float s4 = sin(Hr + 1.0 / 4.0 * pi);
  float s5 = sin(Ap);
  float s6 = sin(Ar - 1.0 / 2.0 * pi);
  result.x = thigh_mm * (s2 * s3 - c2 * c3 * c4) + tibia_mm *
    (s1 * (c4 * s3 + c2 * c3 * s2) + c5 * (s2 * s3 - c2 * c3 * c4)) -
    yf *
    (c6 *
    (c1 * (s1 * (c4 * s3 + c2 * c3 * s2) + c5 * (s2 * s3 - c2 * c3 * c4)) - s5 *
    (s1 * (s2 * s3 - c2 * c3 * c4) - c5 * (c4 * s3 + c2 * c3 * s2))) +
    c3 * s4 * s6) +
    zf *
    (s6 *
    (c1 * (s1 * (c4 * s3 + c2 * c3 * s2) + c5 * (s2 * s3 - c2 * c3 * c4)) - s5 *
    (s1 * (s2 * s3 - c2 * c3 * c4) - c5 * (c4 * s3 + c2 * c3 * s2))) -
    c3 * c6 * s4) +
    xf *
    (c1 * (s1 * (s2 * s3 - c2 * c3 * c4) - c5 * (c4 * s3 + c2 * c3 * s2)) + s5 *
    (s1 * (c4 * s3 + c2 * c3 * s2) + c5 * (s2 * s3 - c2 * c3 * c4)));
  result.y = xf *
    (c1 *
    (c5 * (s2 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) - (c3 * c4 * k) / 2.0f) + s1 *
    (c4 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) + (c3 * k * s2) / 2.0f)) +
    s5 *
    (c5 * (c4 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) + (c3 * k * s2) / 2.0f) -
    s1 * (s2 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) - (c3 * c4 * k) / 2.0f))) +
    tibia_mm *
    (c5 * (c4 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) + (c3 * k * s2) / 2.0f) - s1 *
    (s2 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) - (c3 * c4 * k) / 2.0f)) +
    thigh_mm * (c4 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) + (c3 * k * s2) / 2.0f) -
    yf *
    (s6 * ((c2 * k) / 2.0f - (k * s3 * s4) / 2.0f) +
    c6 *
    (c1 *
    (c5 * (c4 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) + (c3 * k * s2) / 2.0f) -
    s1 * (s2 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) - (c3 * c4 * k) / 2.0f)) -
    s5 *
    (c5 * (s2 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) - (c3 * c4 * k) / 2.0f) +
    s1 * (c4 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) + (c3 * k * s2) / 2.0f)))) -
    zf *
    (c6 * ((c2 * k) / 2.0f - (k * s3 * s4) / 2.0f) -
    s6 *
    (c1 *
    (c5 * (c4 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) + (c3 * k * s2) / 2.0f) -
    s1 * (s2 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) - (c3 * c4 * k) / 2.0f)) -
    s5 *
    (c5 * (s2 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) - (c3 * c4 * k) / 2.0f) +
    s1 * (c4 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) + (c3 * k * s2) / 2.0f))));
  result.z = yf *
    (s6 * ((c2 * k) / 2.0f + (k * s3 * s4) / 2.0f) +
    c6 *
    (c1 *
    (c5 * (c4 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) - (c3 * k * s2) / 2.0f) -
    s1 * (s2 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) + (c3 * c4 * k) / 2.0f)) -
    s5 *
    (c5 * (s2 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) + (c3 * c4 * k) / 2.0f) +
    s1 * (c4 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) - (c3 * k * s2) / 2.0f)))) -
    tibia_mm *
    (c5 * (c4 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) - (c3 * k * s2) / 2.0f) - s1 *
    (s2 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) + (c3 * c4 * k) / 2.0f)) -
    thigh_mm * (c4 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) - (c3 * k * s2) / 2.0f) -
    xf *
    (c1 *
    (c5 * (s2 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) + (c3 * c4 * k) / 2.0f) +
    s1 * (c4 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) - (c3 * k * s2) / 2.0f)) +
    s5 *
    (c5 * (c4 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) - (c3 * k * s2) / 2.0f) -
    s1 * (s2 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) + (c3 * c4 * k) / 2.0f))) +
    zf *
    (c6 * ((c2 * k) / 2.0f + (k * s3 * s4) / 2.0f) -
    s6 *
    (c1 *
    (c5 * (c4 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) - (c3 * k * s2) / 2.0f) -
    s1 * (s2 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) + (c3 * c4 * k) / 2.0f)) -
    s5 *
    (c5 * (s2 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) + (c3 * c4 * k) / 2.0f) +
    s1 * (c4 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) - (c3 * k * s2) / 2.0f))));
  return result;
}


Hpr InverseKinematics::hipAngles(
  float Hyp, float Hp, float Hr, float Kp, float Ap, float Ar,
  float xf, float yf, float zf, geometry_msgs::msg::Point e)
{
  // Code from 2010 to perform interative Inverse Kinematics.
  // Symbolic equations generated in Matlab - see 2010 team report for details and reference
  Hpr result;
  float pi = M_PI;
  float tibia_mm = tibia * 1000;
  float thigh_mm = thigh * 1000;
  float k = sqrt(2.0);
  float c1 = cos(Ap);
  float c2 = cos(Hr + pi / 4.0);
  float c3 = cos(Hyp - pi / 2.0);
  float c4 = cos(Hp);
  float c5 = cos(Kp);
  float c6 = cos(Ar - pi / 2.0);
  float s1 = sin(Kp);
  float s2 = sin(Hp);
  float s3 = sin(Hyp - 1.0 / 2.0 * pi);
  float s4 = sin(Hr + 1.0 / 4.0 * pi);
  float s5 = sin(Ap);
  float s6 = sin(Ar - 1.0 / 2.0 * pi);
  float j11 = thigh_mm * (c4 * s3 + c2 * c3 * s2) - tibia_mm *
    (s1 * (s2 * s3 - c2 * c3 * c4) - c5 * (c4 * s3 + c2 * c3 * s2)) +
    xf *
    (c1 * (s1 * (c4 * s3 + c2 * c3 * s2) + c5 * (s2 * s3 - c2 * c3 * c4)) - s5 *
    (s1 * (s2 * s3 - c2 * c3 * c4) - c5 * (c4 * s3 + c2 * c3 * s2))) +
    c6 * yf *
    (c1 * (s1 * (s2 * s3 - c2 * c3 * c4) - c5 * (c4 * s3 + c2 * c3 * s2)) + s5 *
    (s1 * (c4 * s3 + c2 * c3 * s2) + c5 * (s2 * s3 - c2 * c3 * c4))) -
    s6 * zf *
    (c1 * (s1 * (s2 * s3 - c2 * c3 * c4) - c5 * (c4 * s3 + c2 * c3 * s2)) + s5 *
    (s1 * (c4 * s3 + c2 * c3 * s2) + c5 * (s2 * s3 - c2 * c3 * c4)));
  float j12 = yf *
    (c6 *
    (c1 * (c3 * s1 * s2 * s4 - c3 * c4 * c5 * s4) + s5 * (c3 * c4 * s1 * s4 + c3 * c5 * s2 * s4)) -
    c2 * c3 * s6) -
    tibia_mm * (c3 * s1 * s2 * s4 - c3 * c4 * c5 * s4) -
    zf *
    (s6 *
    (c1 * (c3 * s1 * s2 * s4 - c3 * c4 * c5 * s4) + s5 * (c3 * c4 * s1 * s4 + c3 * c5 * s2 * s4)) +
    c2 * c3 * c6) +
    xf *
    (c1 * (c3 * c4 * s1 * s4 + c3 * c5 * s2 * s4) - s5 * (c3 * s1 * s2 * s4 - c3 * c4 * c5 * s4)) +
    c3 * c4 * s4 * thigh_mm;
  float j21 = xf *
    (c1 *
    (c5 * (c4 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) + (c3 * k * s2) / 2.0f) - s1 *
    (s2 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) - (c3 * c4 * k) / 2.0f)) -
    s5 *
    (c5 * (s2 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) - (c3 * c4 * k) / 2.0f) +
    s1 * (c4 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) + (c3 * k * s2) / 2.0f))) -
    tibia_mm *
    (c5 * (s2 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) - (c3 * c4 * k) / 2.0f) + s1 *
    (c4 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) + (c3 * k * s2) / 2.0f)) -
    thigh_mm * (s2 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) - (c3 * c4 * k) / 2.0f) +
    c6 * yf *
    (c1 *
    (c5 * (s2 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) - (c3 * c4 * k) / 2.0f) +
    s1 * (c4 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) + (c3 * k * s2) / 2.0f)) +
    s5 *
    (c5 * (c4 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) + (c3 * k * s2) / 2.0f) -
    s1 * (s2 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) - (c3 * c4 * k) / 2.0f))) -
    s6 * zf *
    (c1 *
    (c5 * (s2 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) - (c3 * c4 * k) / 2.0f) +
    s1 * (c4 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) + (c3 * k * s2) / 2.0f)) +
    s5 *
    (c5 * (c4 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) + (c3 * k * s2) / 2.0f) -
    s1 * (s2 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) - (c3 * c4 * k) / 2.0f)));
  float j22 = tibia_mm *
    (c4 * c5 * ((c2 * k) / 2.0f - (k * s3 * s4) / 2.0f) - s1 * s2 *
    ((c2 * k) / 2.0f - (k * s3 * s4) / 2.0f)) +
    xf *
    (c1 *
    (c4 * s1 * ((c2 * k) / 2.0f - (k * s3 * s4) / 2.0f) + c5 * s2 *
    ((c2 * k) / 2.0f - (k * s3 * s4) / 2.0f)) +
    s5 *
    (c4 * c5 * ((c2 * k) / 2.0f - (k * s3 * s4) / 2.0f) - s1 * s2 *
    ((c2 * k) / 2.0f - (k * s3 * s4) / 2.0f))) +
    yf *
    (s6 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) -
    c6 *
    (c1 *
    (c4 * c5 * ((c2 * k) / 2.0f - (k * s3 * s4) / 2.0f) - s1 * s2 *
    ((c2 * k) / 2.0f - (k * s3 * s4) / 2.0f)) -
    s5 *
    (c4 * s1 * ((c2 * k) / 2.0f - (k * s3 * s4) / 2.0f) + c5 * s2 *
    ((c2 * k) / 2.0f - (k * s3 * s4) / 2.0f)))) +
    zf *
    (c6 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) +
    s6 *
    (c1 *
    (c4 * c5 * ((c2 * k) / 2.0f - (k * s3 * s4) / 2.0f) - s1 * s2 *
    ((c2 * k) / 2.0f - (k * s3 * s4) / 2.0f)) -
    s5 *
    (c4 * s1 * ((c2 * k) / 2.0f - (k * s3 * s4) / 2.0f) + c5 * s2 *
    ((c2 * k) / 2.0f - (k * s3 * s4) / 2.0f)))) +
    c4 * thigh_mm * ((c2 * k) / 2.0f - (k * s3 * s4) / 2.0f);
  float j31 = tibia_mm *
    (c5 * (s2 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) + (c3 * c4 * k) / 2.0f) + s1 *
    (c4 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) - (c3 * k * s2) / 2.0f)) -
    xf *
    (c1 *
    (c5 * (c4 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) - (c3 * k * s2) / 2.0f) -
    s1 * (s2 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) + (c3 * c4 * k) / 2.0f)) -
    s5 *
    (c5 * (s2 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) + (c3 * c4 * k) / 2.0f) +
    s1 * (c4 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) - (c3 * k * s2) / 2.0f))) +
    thigh_mm * (s2 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) + (c3 * c4 * k) / 2.0f) -
    c6 * yf *
    (c1 *
    (c5 * (s2 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) + (c3 * c4 * k) / 2.0f) +
    s1 * (c4 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) - (c3 * k * s2) / 2.0f)) +
    s5 *
    (c5 * (c4 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) - (c3 * k * s2) / 2.0f) -
    s1 * (s2 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) + (c3 * c4 * k) / 2.0f))) +
    s6 * zf *
    (c1 *
    (c5 * (s2 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) + (c3 * c4 * k) / 2.0f) +
    s1 * (c4 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) - (c3 * k * s2) / 2.0f)) +
    s5 *
    (c5 * (c4 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) - (c3 * k * s2) / 2.0f) -
    s1 * (s2 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) + (c3 * c4 * k) / 2.0f)));
  float j32 = -tibia_mm *
    (c4 * c5 * ((c2 * k) / 2.0f + (k * s3 * s4) / 2.0f) - s1 * s2 *
    ((c2 * k) / 2.0f + (k * s3 * s4) / 2.0f)) -
    xf *
    (c1 *
    (c4 * s1 * ((c2 * k) / 2.0f + (k * s3 * s4) / 2.0f) + c5 * s2 *
    ((c2 * k) / 2.0f + (k * s3 * s4) / 2.0f)) +
    s5 *
    (c4 * c5 * ((c2 * k) / 2.0f + (k * s3 * s4) / 2.0f) - s1 * s2 *
    ((c2 * k) / 2.0f + (k * s3 * s4) / 2.0f))) -
    yf *
    (s6 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) -
    c6 *
    (c1 *
    (c4 * c5 * ((c2 * k) / 2.0f + (k * s3 * s4) / 2.0f) - s1 * s2 *
    ((c2 * k) / 2.0f + (k * s3 * s4) / 2.0f)) -
    s5 *
    (c4 * s1 * ((c2 * k) / 2.0f + (k * s3 * s4) / 2.0f) + c5 * s2 *
    ((c2 * k) / 2.0f + (k * s3 * s4) / 2.0f)))) -
    zf *
    (c6 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) +
    s6 *
    (c1 *
    (c4 * c5 * ((c2 * k) / 2.0f + (k * s3 * s4) / 2.0f) - s1 * s2 *
    ((c2 * k) / 2.0f + (k * s3 * s4) / 2.0f)) -
    s5 *
    (c4 * s1 * ((c2 * k) / 2.0f + (k * s3 * s4) / 2.0f) + c5 * s2 *
    ((c2 * k) / 2.0f + (k * s3 * s4) / 2.0f)))) -
    c4 * thigh_mm * ((c2 * k) / 2.0f + (k * s3 * s4) / 2.0f);
  float xbe = e.x;
  float ybe = e.y;
  float zbe = e.z;
  float lambda = 0.4f;
  float la2 = lambda * lambda;
  float la4 = la2 * la2;
  float j322 = j32 * j32;
  float j222 = j22 * j22;
  float j122 = j12 * j12;
  float j212 = j21 * j21;
  float j112 = j11 * j11;
  float j312 = j31 * j31;
  float sigma = 1.0f /
    (la4 + j112 * j222 + j122 * j212 + j112 * j322 + j122 * j312 + j212 * j322 + j222 * j312 +
    j112 * la2 + j122 * la2 + j212 * la2 + j222 * la2 + j312 * la2 + j322 * la2 -
    2.0f * j11 * j12 * j21 * j22 - 2.0f * j11 * j12 * j31 * j32 - 2.0f * j21 * j22 * j31 * j32);
  result.Hp = sigma * xbe *
    (j11 * j222 + j11 * j322 + j11 * la2 - j12 * j21 * j22 - j12 * j31 * j32) +
    sigma * ybe * (j122 * j21 + j21 * j322 + j21 * la2 - j11 * j12 * j22 - j22 * j31 * j32) +
    sigma * zbe * (j122 * j31 + j222 * j31 + j31 * la2 - j11 * j12 * j32 - j21 * j22 * j32);
  result.Hr = sigma * xbe *
    (j12 * j212 + j12 * j312 + j12 * la2 - j11 * j21 * j22 - j11 * j31 * j32) +
    sigma * ybe * (j112 * j22 + j22 * j312 + j22 * la2 - j11 * j12 * j21 - j21 * j31 * j32) +
    sigma * zbe * (j112 * j32 + j212 * j32 + j32 * la2 - j11 * j12 * j31 - j21 * j22 * j31);
  return result;
}
