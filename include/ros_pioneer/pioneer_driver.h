/*********************************************************************
* pioneer_driver.h
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

#ifndef PIONEER_DRIVER_H
#define PIONEER_DRIVER_H

#include "pioneer_packet.h"
#include "pioneer_config.h"
#include <ros/ros.h>
#include <deque>
#include <termios.h>

namespace ros_pioneer {

class PioneerDriver
{
public:
  PioneerDriver();
  ~PioneerDriver();
private:
  bool initSerial(const std::string &serial_port);
  bool initTCP(const std::string &hostname,
               const int port);
  void loadConfig(const std::string &filename);
  bool getSerialSettings(termios &settings);
  bool setSerialSettings(const termios &settings);
  bool flushSerial();
  bool getSerialDescriptorFlags(int &flags);
  bool setSerialDescriptorFlags(int &flags);
  bool getStatusFlags(int &flags);
  bool setStatusFlags(const int &flags);
  void setSerialSpeed(termios &settings, int baudrate);
  bool synchronize(PioneerPacket &packet);
  void parseSynchronizationPacket(const PioneerPacket &packet);
  void closeConnection();

  // Pioneer Commands
  void sendOpen();
  void sendEnable(bool value);
  void sendPulse();

  std::deque<PioneerPacket> outgoing_packets_;
  PioneerConfig config_;
  int file_descriptor_;
  std::string name_;
  std::string type_;
  std::string subtype_;
};

}

#endif // PIONEER_DRIVER_H
