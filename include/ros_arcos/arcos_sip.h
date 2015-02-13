/*********************************************************************
* arcos_sip.h
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

#ifndef ARCOS_SIP_H
#define ARCOS_SIP_H

#include <ros_arcos/arcos_config.h>
#include <ros_arcos/arcos_commands.h>
#include <ros_arcos/arcos_packet.h>
#include <nav_msgs/Odometry.h>

namespace ros_arcos{

class ArcosSIP
{
public:
  ArcosSIP(const ArcosConfig &config);
private:
  // Direct Methods
  void identify(const ArcosPacket &packet) const;
  void parseStandard(const ArcosPacket &packet) const;
  void parseConfig(const ArcosPacket &packet) const;
  void parseSERAUX(const ArcosPacket &packet) const;
  void parseEncoder(const ArcosPacket &packet) const;
  void parseTCM2(const ArcosPacket &packet) const;
  void parseIO(const ArcosPacket &packet) const;
  void parseIMU(const ArcosPacket &packet) const;
  void parseJoystick(const ArcosPacket &packet) const;
  void parseGripper(const ArcosPacket &packet) const;
  void parseGyro(const ArcosPacket &packet) const;
  void parseArmInfo(const ArcosPacket &packet) const;
  void parseArm(const ArcosPacket &packet) const;

  // Indirect Methods
  static int encoderDifference(int previous, int current);
  static double radToDegrees(double radians);
  static double degreesToRad(double degrees);

  ros::NodeHandle nh_;
  ArcosConfig config_;
};

}
#endif // ARCOS_SIP_H
