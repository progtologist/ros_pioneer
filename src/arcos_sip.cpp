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
  arcos_version_(1.0),
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
  typedef std::pair<unsigned char, int> sonar_t;
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
  std::vector<sonar_t> sonar_readings;
  unsigned char grip_state;
  unsigned char analog_port;
  unsigned char analog_reading;
  unsigned char digital_in;
  unsigned char digital_out;
  int battery; // Useful when batt > 255
  unsigned char charge_state;

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
  for (int i = 0; i < sonar_count; ++i) {
    sonar_t sonar_reading;
    sonar_reading.first = packet.at(index++);
    index = packet.getIntegerAt(index, sonar_reading.second);
    sonar_readings.push_back(sonar_reading);
  }
  grip_state = packet.at(index++);
  analog_port = packet.at(index++);
  analog_reading = packet.at(index++);
  digital_in = packet.at(index++);
  digital_out = packet.at(index++);
  index = packet.getIntegerAt(index, battery);
  charge_state = packet.at(index);

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
  int index = 2;
  std::string robot_type;
  std::string robot_subtype;
  std::string robot_serial_number;
  unsigned char four_motors;
  int top_rot_vel;
  int top_trans_vel;
  int top_rot_acc;
  int top_trans_acc;
  int max_pwm;
  std::string robot_name;
  unsigned char sip_cycle;
  unsigned char host_baud;
  unsigned char aux_baud;
  int gripper;
  int front_sonar;
  unsigned char rear_sonar;
  int low_battery;
  int rev_count;
  int watchdog;
  unsigned char p2mpacs;
  int stall_value, stall_count;
  int joy_vel, joy_rvel;
  int max_rot_vel, max_trans_vel;
  int rot_accel, rot_decel;
  int rot_kp, rot_kv, rot_ki;
  int trans_accel, trans_decel;
  int trans_kp, trans_kv, trans_ki;
  unsigned char front_bumpers, rear_bumpers;
  unsigned char charger;
  unsigned char sonar_cycle;
  unsigned char autobaud;
  unsigned char has_gyro;
  int drift_factor;
  unsigned char aux2_baud, aux3_baud;
  int ticks_mm;
  int shutdown_volts;
  std::string version;
  int gyro_cw, gyro_ccw;
  unsigned char kinematics_delay;

  index = packet.getStringAt(index, robot_type);
  index = packet.getStringAt(index, robot_subtype);
  index = packet.getStringAt(index, robot_serial_number);
  four_motors = packet.at(index++);
  index = packet.getIntegerAt(index, top_rot_vel);
  index = packet.getIntegerAt(index, top_trans_vel);
  index = packet.getIntegerAt(index, top_rot_acc);
  index = packet.getIntegerAt(index, top_trans_acc);
  index = packet.getIntegerAt(index, max_pwm);
  index = packet.getStringAt(index, robot_name);
  sip_cycle = packet.at(index++);
  host_baud = packet.at(index++);
  aux_baud  = packet.at(index++);
  index = packet.getIntegerAt(index, gripper);
  index = packet.getIntegerAt(index, front_sonar);
  rear_sonar = packet.at(index++);
  index = packet.getIntegerAt(index, low_battery);
  index = packet.getIntegerAt(index, rev_count);
  index = packet.getIntegerAt(index, watchdog);
  p2mpacs = packet.at(index++);
  index = packet.getIntegerAt(index, stall_value);
  index = packet.getIntegerAt(index, stall_count);
  index = packet.getIntegerAt(index, joy_vel);
  index = packet.getIntegerAt(index, joy_rvel);
  index = packet.getIntegerAt(index, max_rot_vel);
  index = packet.getIntegerAt(index, max_trans_vel);
  index = packet.getIntegerAt(index, rot_accel);
  index = packet.getIntegerAt(index, rot_decel);
  index = packet.getIntegerAt(index, rot_kp);
  index = packet.getIntegerAt(index, rot_kv);
  index = packet.getIntegerAt(index, rot_ki);
  index = packet.getIntegerAt(index, trans_accel);
  index = packet.getIntegerAt(index, trans_decel);
  index = packet.getIntegerAt(index, trans_kp);
  index = packet.getIntegerAt(index, trans_kv);
  index = packet.getIntegerAt(index, trans_ki);
  front_bumpers = packet.at(index++);
  rear_bumpers = packet.at(index++);
  charger = packet.at(index++);
  sonar_cycle = packet.at(index++);
  autobaud = packet.at(index++);
  has_gyro = packet.at(index++);
  index = packet.getIntegerAt(index, drift_factor);
  aux2_baud = packet.at(index++);
  aux3_baud = packet.at(index++);
  index = packet.getIntegerAt(index, ticks_mm);
  index = packet.getIntegerAt(index, shutdown_volts);
  // Check version
  if (packet.at(index) != '\0') {
    index = packet.getStringAt(index, version);
    index = packet.getIntegerAt(index, gyro_cw);
    index = packet.getIntegerAt(index, gyro_ccw);
    kinematics_delay = packet.at(index++);
  }
}

void ArcosSIP::parseSERAUX(const ArcosPacket& packet) const
{
  int size = static_cast<int>(packet.at(0)) - 2;
  std::string serial_message;

  int index = 2;
  index = packet.getStringAt(index, size, serial_message);
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
