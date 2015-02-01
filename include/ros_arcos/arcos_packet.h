/*********************************************************************
* arcos_packet.h
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


#ifndef ARCOS_PACKET_H
#define ARCOS_PACKET_H

#include <ros/ros.h>
#include <string>
#include <ros_arcos/arcos_commands.h>

namespace ros_arcos{

class ArcosPacket
{
public:
  void command(const cmd::Void_t &command);
  void command(const cmd::Int_t &command, int value);
  void command(const cmd::TwoBytes_t &command,
               unsigned char first_byte,
               unsigned char second_byte);
  void command(const cmd::Str_t &command,
               const std::string &msg);
  void send(int file_descriptor);
  void receive(int file_descriptor);
  unsigned char& operator [](int i);
  unsigned char operator [](int i) const;
  void printHex();
  void printDec();
protected:
  int calculateChecksum();
  bool check();
  void setHeader();
  void setSize(unsigned char size);
  void setCommand(unsigned char command);
  void setType(const cmd::Types_t &type);
  void setArgument(int value);
  void setArgument(unsigned char first_byte,
                   unsigned char second_byte);
  void setArgument(const std::string &argument);
  void setChecksum();

  // Variables
  unsigned char buffer_[207];
};

}

#endif // ARCOS_PACKET_H
