/*********************************************************************
* packet.cpp
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

#include <ros_arcos/packet.h>

namespace ros_arcos{

/** FRIEND FUNCTIONS **/
std::ostream& operator<<(std::ostream &out, const ArcosPacket &packet)
{
  std::string temp_string(packet.packet_);
  out << temp_string;
  return out;
}

std::istream& operator>>(std::istream &in, ArcosPacket &packet)
{
  std::string temp_string;
  in >> temp_string;
  std::sprintf(packet.packet_, "%s", temp_string.c_str());
  return in;
}

/** CLASS FUNCTIONS **/
ArcosPacket::ArcosPacket(const char *s)
{
  std::strncpy(packet_, s, sizeof(packet_));
}

ArcosPacket& ArcosPacket::operator<<(const char *s)
{
  std::string temp_string(this->getPacket());
  temp_string.append(s);
  std::strncpy(this->packet_, temp_string.c_str(), sizeof(this->packet_));
  return (*this);
}

std::string ArcosPacket::getPacket()
{
  std::string temp_string(packet_);
  return (temp_string);
}


}
