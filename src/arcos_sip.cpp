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

namespace ros_arcos{

ArcosSIP::ArcosSIP()
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

}
