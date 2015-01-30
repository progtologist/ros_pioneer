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

#include <ros_arcos/arcos_packet.h>

namespace ros_arcos{

/** FRIEND FUNCTIONS **/
std::ostream& operator<<(std::ostream &out, const ArcosPacket &packet)
{
  out << packet.packet_;
  return out;
}

std::istream& operator>>(std::istream &in, ArcosPacket &packet)
{
  in >> packet.packet_;
  return (in);
}

/** CLASS FUNCTIONS **/
ArcosPacket::ArcosPacket(const char *s)
{
  packet_.assign(s);
}

ArcosPacket& ArcosPacket::operator<<(const char *s)
{
  this->packet_.append(s);
  return (*this);
}

void ArcosPacket::packet(const std::string &packet)
{
  packet_ = packet;
}

std::string ArcosPacket::getPacket()
{
  return (packet_);
}

bool ArcosPacket::check(unsigned char* packet)
{
  int checksum = this->calculateChecksum(packet);
  size_t packet_size = packet[2] - 2;
  if ( (checksum == packet[packet_size-2] << 8) | packet[packet_size-1])
    return true;
  return false;
}

unsigned char* ArcosPacket::encode()
{
  unsigned char* char_packet = new unsigned char[packet_.size()+5];
  char_packet[0] = 0xFA;
  char_packet[1] = 0xFB;
  char_packet[2] = packet_.size() + 2;
  std::strncpy(reinterpret_cast<char*>(&char_packet[3]),
      packet_.c_str(),
      packet_.size());
  int checksum = this->calculateChecksum(char_packet);
  char_packet[packet_.size()+3] = checksum >> 8;
  char_packet[packet_.size()+4] = checksum & 0xFF;
  return (char_packet);
}

void ArcosPacket::decode(unsigned char *packet)
{
  size_t packet_size = packet[2] - 2;
  unsigned char* null_terminated_packet = new unsigned char[packet_size+1];
  std::strncpy(reinterpret_cast<char*>(null_terminated_packet),
               reinterpret_cast<char*>(&packet[3]), packet_size);
  null_terminated_packet[packet_size] = '\0';
  packet_.assign(reinterpret_cast<char*>(null_terminated_packet));
}

void ArcosPacket::send()
{
  unsigned char* packet = this->encode();
  ROS_INFO("HEXADECIMAL");
  for (size_t i = 0; i < (packet_.size()+5); ++i)
  {
    ROS_INFO("0x%.2x ", packet[i]);
  }
  ROS_INFO("DECIMAL");
  for (size_t i = 0; i < (packet_.size()+5); ++i)
  {
    ROS_INFO("%u ", packet[i]);
  }

  ROS_INFO("%s", this->check(packet) ? "True" : "False");

  delete[] packet;
}

int ArcosPacket::calculateChecksum(unsigned char* packet)
{
  int checksum = 0;
  size_t bytes_left = packet[2] - 2;
  size_t current_byte = 3;
  while (bytes_left > 1)
  {
    checksum += ((unsigned char)packet[current_byte]<<8) | (unsigned char)packet[current_byte+1];
    checksum = checksum & 0xFFFF;
    bytes_left -= 2;
    current_byte += 2;
  }
  if (current_byte > 0)
    checksum = checksum ^ (int)((unsigned char)packet[current_byte]);

  ROS_INFO("Checksum is %i", checksum);
  return (checksum);
}

}
