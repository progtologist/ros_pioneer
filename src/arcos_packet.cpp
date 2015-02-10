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
#include <cassert>
#include  <fcntl.h>

namespace ros_arcos{

void ArcosPacket::command(const Init_t &command)
{
  this->clear();
  this->setHeader();
  this->setSize(3);
  this->setCommand(static_cast<unsigned char>(command));
  this->setChecksum();
}

void ArcosPacket::command(const Void_t &command)
{
  this->clear();
  this->setHeader();
  this->setSize(3);
  this->setCommand(static_cast<unsigned char>(command));
  this->setChecksum();
}

void ArcosPacket::command(const Int_t &command, int value)
{
  this->clear();
  this->setHeader();
  this->setSize(6);
  this->setCommand(static_cast<unsigned char>(command));
  Types_t type = (value >= 0) ? ARGINT : ARGNINT;
  this->setType(type);
  this->setArgument(value);
  this->setChecksum();
}

void ArcosPacket::command(const TwoBytes_t &command,
                          unsigned char first_byte,
                          unsigned char second_byte)
{
  this->clear();
  this->setHeader();
  this->setSize(6);
  this->setCommand(static_cast<unsigned char>(command));
  this->setType(ARGINT);
  this->setArgument(first_byte,second_byte);
  this->setChecksum();
}

void ArcosPacket::command(const Str_t &command,
                          const std::string &msg)
{
  assert(msg.size() < 200);
  this->clear();
  this->setHeader();
  this->setSize(msg.size()+5);
  this->setCommand(static_cast<unsigned char>(command));
  this->setType(ARGSTR);
  this->setArgument(msg);
  this->setChecksum();
}

void ArcosPacket::clear()
{
  memset(buffer_, 0, sizeof(buffer_));
}

bool ArcosPacket::send(int file_descriptor)
{
  size_t buffer_size = buffer_[2] + 3;  // Send the entire buffer,
                                        // Including the header
  size_t written_bytes = write(file_descriptor, buffer_, buffer_size);
  if (written_bytes != buffer_size)
  {
    ROS_ERROR("Could not send the entire buffer");
    return false;
  }
  return true;
}

bool ArcosPacket::receive(int file_descriptor)
{
  unsigned char current_char = 0, previous_char = 0;
  bool header_flag = false;

  this->clear();

  while (!header_flag && ros::ok())
  {
    if ( (read(file_descriptor, &current_char, 1) < 0) )
    {
      ROS_ERROR("Error while trying to read serial port");
      return false;
    }
    if (previous_char == 0xFA && current_char == 0xFB)
    {
      header_flag = true;
    }
    previous_char = current_char;
  }
  this->setHeader();
  read(file_descriptor, &current_char, 1);
  this->setSize(current_char);
  if ( (read(file_descriptor, &buffer_[3], buffer_[2]) < buffer_[2]) )
  {
    ROS_ERROR("Error while trying to read the packet body");
    return false;
  }
  if (!this->check())
  {
    ROS_ERROR("Packet failed to pass checksum");
    return false;
  }
  return true;
}

unsigned char ArcosPacket::operator [](int i) const
{
  if (i < 0 || i > 206)
    ROS_ERROR("Packet out of bounds, requested %i", i);
  else
    return buffer_[i];
}

unsigned char ArcosPacket::at(int i) const
{
  if (i < 0 || i > buffer_[2] - 2)
    ROS_ERROR("Packet out of bounds, requested %i and size was %i", i, buffer_[2]-2);
  else
    return buffer_[i+2];
}

int ArcosPacket::getIntegerAt(int i) const
{
  /* 15 ls-bits */
  if (i < 0 || i > buffer_[2] - 2)
    ROS_ERROR("Packet out of bounds, requested %i and size was %i", i, buffer_[2]-2);
  else
  {
    int value = buffer_[i+2] | buffer_[i+3] << 8;
    return (value);
  }
}

std::string ArcosPacket::getStringAt(int i) const
{
  if (i < 0 || i > buffer_[2] - 2)
    ROS_ERROR("Packet out of bounds, requested %i and size was %i", i, buffer_[2]-2);
  else
  {
    std::string msg;
    while (buffer_[i+2] != '\0' && i < buffer_[2] - 1)
    {
      msg.append(reinterpret_cast<const char*>(&buffer_[i+2]),
          reinterpret_cast<const char*>(&buffer_[i+3]));
      i++;
    }
    return msg;
  }
}

std::string ArcosPacket::getStringAt(int i, size_t length) const
{
  if (i < 0 || i > buffer_[2] - 2 || length > buffer_[2] - 2)
    ROS_ERROR("Packet out of bounds, requested %i, length was %i, size was %i,", i, length, buffer_[2]-2);
  else
  {
    std::string msg;
    msg.append(reinterpret_cast<const char*>(&buffer_[i+2]),
        reinterpret_cast<const char*>(&buffer_[i+length+2]));
    return msg;
  }
}

bool ArcosPacket::check()
{
  int checksum = this->calculateChecksum();
  size_t buffer_size = buffer_[2] - 2;
  if ( (checksum == buffer_[buffer_size-2] << 8) | buffer_[buffer_size-1])
    return true;
  return false;
}

int ArcosPacket::calculateChecksum()
{
  int checksum = 0;
  size_t bytes_left = buffer_[2] - 2;
  size_t current_byte = 3;
  while (bytes_left > 1)
  {
    checksum += (buffer_[current_byte]<<8) | buffer_[current_byte+1];
    checksum = checksum & 0xFFFF;
    bytes_left -= 2;
    current_byte += 2;
  }
  if (current_byte > 0)
    checksum = checksum ^ buffer_[current_byte];

  ROS_DEBUG("Checksum is %i", checksum);
  return (checksum);
}

void ArcosPacket::printHex()
{
  size_t buffer_size = buffer_[2] + 3;  // Print the entire buffer,
                                        // Including the header
  ROS_INFO("HEX");
  for (size_t i = 0; i < buffer_size; ++i)
  {
    ROS_INFO("0x%.2x ", buffer_[i]);
  }
}

void ArcosPacket::printDec()
{
  size_t buffer_size = buffer_[2] + 3;  // Print the entire buffer,
                                        // Including the header
  ROS_INFO("DEC");
  for (size_t i = 0; i < buffer_size; ++i)
  {
    ROS_INFO("%u ", buffer_[i]);
  }
}

void ArcosPacket::printASCII()
{
  size_t buffer_size = buffer_[2] + 3;  // Print the entire buffer,
                                        // Including the header
  ROS_INFO("ASCII");
  for (size_t i = 0; i < buffer_size; ++i)
  {
    ROS_INFO("%c ", static_cast<unsigned char>(buffer_[i]));
  }
}

void ArcosPacket::setHeader()
{
  buffer_[0] = 0xFA;
  buffer_[1] = 0xFB;
}

void ArcosPacket::setSize(unsigned char size)
{
  buffer_[2] = size;
}

void ArcosPacket::setCommand(unsigned char command)
{
  buffer_[3] = command;
}

void ArcosPacket::setType(const Types_t &type)
{
  buffer_[4] = static_cast<unsigned char>(type);
}

void ArcosPacket::setArgument(int value)
{
  unsigned short argument = static_cast<unsigned short>(std::abs(value));
  buffer_[5] = argument & 0xFF;
  buffer_[6] = (argument >> 8) & 0xFF;
}

void ArcosPacket::setArgument(unsigned char first_byte,
                              unsigned char second_byte)
{
  buffer_[5] = first_byte;
  buffer_[6] = second_byte;
}

void ArcosPacket::setArgument(const std::string &argument)
{
  buffer_[5] = argument.size();
  for (size_t i = 0; i < argument.size(); ++i)
  {
    buffer_[i+6] = static_cast<unsigned char>(argument.at(i));
  }
}

void ArcosPacket::setChecksum()
{
  size_t buffer_size = buffer_[2];
  int checksum = this->calculateChecksum();
  buffer_[buffer_size+1] = checksum >> 8;
  buffer_[buffer_size+2] = checksum & 0xFF;
}

}
