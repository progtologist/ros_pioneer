/*********************************************************************
* arcos_sip.cpp
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, University of Patras
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of University of Patras nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Authors: Aris Synodinos
*********************************************************************/

#include <ros_arcos/arcos_sip.h>
#include <boost/math/constants/constants.hpp>
#include <limits>
#include <cmath>

using nav_msgs::Odometry;

namespace ros_arcos{

ArcosSIP::ArcosSIP(const ArcosConfig &config) :
  nh_("~"),
  config_(config)
{

}

void ArcosSIP::identify(const ArcosPacket &packet) const
{
  if (packet.at(1) >= STANDARDmpac && packet.at(1) <= STANDARDMpac)
    parseStandard(packet);
  else {
    switch (packet.at(1))
    {
    case CONFIGpac:
      parseConfig(packet);
      break;
    case SERAUXpac:
    case SERAUX2pac:
    case SERAUX3pac:
      parseSERAUX(packet);
      break;
    case ENCODERpac:
      parseEncoder(packet);
      break;
    case TCM2pac:
      parseTCM2(packet);
      break;
    case IOpac:
      parseIO(packet);
      break;
    case IMUpac:
      parseIMU(packet);
      break;
    case JOYSTICKpac:
      parseJoystick(packet);
      break;
    case GRIPPERpac:
      parseGripper(packet);
      break;
    case GYROpac:
      parseGyro(packet);
      break;
    case ARMINFOpac:
      parseArmInfo(packet);
      break;
    case ARMpac:
      parseArm(packet);
      break;
    default:
      ROS_ERROR("Could not identify packet type 0x%.2x", packet.at(1));
      break;
    }
  }
}

void ArcosSIP::parseStandard(const ArcosPacket& packet) const
{
  static int total_x    = 0;
  static int total_y    = 0;
  static int total_th   = 0;
  static int prev_x     = 0;
  static int prev_y     = 0;
  static double dist_conv_factor  = config_.getDouble("DistConvFactor");
  static double angle_conv_factor = config_.getDouble("AngleConvFactor");
  static double vel_conv_factor   = config_.getDouble("VelConvFactor");
  static double diff_conv_factor  = config_.getDouble("DiffConvFactor");
  static bool front_bumpers       = config_.getBool("FrontBumpers");
  static bool rear_bumpers        = config_.getBool("RearBumpers");
  static int num_front_bumpers    = config_.getInt("NumFrontBumpers");
  static int num_rear_bumpers     = config_.getInt("NumRearBumpers");

  int index = 2;
  int curr_x, curr_y, curr_th;
  int vel_l, vel_r;
  unsigned char batt;
  std::vector<bool> l_stall_bump;
  std::vector<bool> r_stall_bump;
  int control;
  std::vector<bool> flags_0_7;
  std::vector<bool> flags_8_15;
  unsigned char compass;
  unsigned char sonar_count;

  if (total_x == std::numeric_limits<int>::max())
    total_x = 0;
  if (total_y == std::numeric_limits<int>::max())
    total_y = 0;

  index = packet.getIntegerAt(index, curr_x);
  index = packet.getIntegerAt(index, curr_y);
  index = packet.getIntegerAt(index, curr_th);
  index = packet.getIntegerAt(index, vel_l);
  index = packet.getIntegerAt(index, vel_r);
  batt = packet.at(index++);
  index = packet.getBoolVectorAt(index, l_stall_bump);
  index = packet.getBoolVectorAt(index, r_stall_bump);
  index = packet.getIntegerAt(index, control);
  index = packet.getBoolVectorAt(index, flags_0_7);
  index = packet.getBoolVectorAt(index, flags_8_15);
  compass = packet.at(index++);
  sonar_count = packet.at(index++);

  curr_x %= 4096;
  curr_y %= 4096;

  int diff_x = this->encoderDifference(prev_x, curr_x);
  int diff_y = this->encoderDifference(prev_y, curr_y);
  prev_x = curr_x;
  prev_y = curr_y;
  int change_x = static_cast<int>(round(diff_x * dist_conv_factor));
  int change_y = static_cast<int>(round(diff_y * dist_conv_factor));
  total_x += change_x;
  total_y += change_y;
  total_th = static_cast<int>(round(degreesToRad(curr_th * angle_conv_factor)));

}

void ArcosSIP::parseConfig(const ArcosPacket& packet) const
{

}

void ArcosSIP::parseSERAUX(const ArcosPacket& packet) const
{

}

void ArcosSIP::parseEncoder(const ArcosPacket& packet) const
{

}

void ArcosSIP::parseTCM2(const ArcosPacket& packet) const
{

}

void ArcosSIP::parseIO(const ArcosPacket& packet) const
{

}

void ArcosSIP::parseIMU(const ArcosPacket& packet) const
{

}

void ArcosSIP::parseJoystick(const ArcosPacket& packet) const
{

}

void ArcosSIP::parseGripper(const ArcosPacket& packet) const
{

}

void ArcosSIP::parseGyro(const ArcosPacket& packet) const
{

}

void ArcosSIP::parseArmInfo(const ArcosPacket& packet) const
{

}

void ArcosSIP::parseArm(const ArcosPacket& packet) const
{

}

int ArcosSIP::encoderDifference(int previous, int current)
{
  int diff = std::abs(current - previous);
  int compl_diff = 4096 - diff;
  if (current >= previous)
    return (diff < compl_diff ? diff : compl_diff);
  else
    return (diff < compl_diff ? -diff : -compl_diff);
}

double ArcosSIP::radToDegrees(double radians)
{
  static const double PI = boost::math::constants::pi<double>();
  return (radians * 180.0 / PI);
}

double ArcosSIP::degreesToRad(double degrees)
{
  static const double PI = boost::math::constants::pi<double>();
  return (degrees * PI / 180.0);
}

}
