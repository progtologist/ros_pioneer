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

#include <ros_pioneer/pioneer_driver.h>
#include <ros/package.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/ip.h>
#include <netinet/tcp.h>

namespace ros_pioneer {

PioneerDriver::PioneerDriver()
{
  ros::NodeHandle nh("~");
  std::string serial_port, hostname;
  int tcp_port;
  nh.param("serial_port", serial_port, std::string("/dev/ttyS0"));
  nh.param("hostname", hostname, std::string("localhost"));
  nh.param("tcp_port", tcp_port, 8101);
  // First Initialize Network Connection
  if (!initTCP(hostname,tcp_port))
    initSerial(serial_port);
  sendOpen();
  sendEnable(true);
  sendPulse();
  // load configuration file
  std::string config_path = ros::package::getPath("ros_pioneer");
  config_path.append("/config/param/" + subtype_ + ".p");
  this->loadConfig(config_path);
}

PioneerDriver::~PioneerDriver()
{
  this->closeConnection();
}

bool PioneerDriver::initSerial(const std::string &serial_port)
{
  file_descriptor_ = -1;
  int flags;
  int baudrates[] = {B9600, B19200, B38400, B57600, B115200};
  int cur_br = 0;
  termios newtio;

  ROS_INFO("Openning serial port %s", serial_port.c_str());

  file_descriptor_ = open(serial_port.c_str(),
                          O_RDWR | O_SYNC | O_NONBLOCK, S_IRUSR | S_IWUSR);
  if (file_descriptor_ < 0)
  {
    ROS_ERROR("Failed to open serial port %s", serial_port.c_str());
    return false;
  }

  if (!getSerialSettings(newtio))
    return false;

  cfmakeraw(&newtio);
  setSerialSpeed(newtio, baudrates[cur_br]);

  if (!setSerialSettings(newtio))
    return false;

  if (!getSerialDescriptorFlags(flags))
    return false;

  PioneerPacket packet;
  while (!synchronize(packet))
  {
    cur_br++;
    if (cur_br == 5)
    {
      ROS_ERROR("Could not initialize connection");
      return false;
    }
    setSerialSpeed(newtio,baudrates[cur_br]);
    setSerialSettings(newtio);
  }
  ROS_INFO("Synchronized");
  this->parseSynchronizationPacket(packet);
  ROS_INFO("Connected to a %s %s named %s",
           type_.c_str(),
           subtype_.c_str(),
           name_.c_str());
  return true;
}

bool PioneerDriver::initTCP(const std::string &hostname,
                          const int port)
{
  file_descriptor_ = -1;
  int flags;
  sockaddr_in sockaddress;
  hostent hostinfo;

  bzero(&sockaddress, sizeof(sockaddress));
  sockaddress.sin_family = AF_INET;
  hostinfo = *gethostbyname(hostname.c_str());

  // set port number
  sockaddress.sin_port = htons(port);

  file_descriptor_ = socket(AF_INET, SOCK_STREAM, 0);
  if (file_descriptor_ < 0)
  {
    ROS_ERROR("Failed to open TCP port %hu to %s", port, hostname.c_str());
    return false;
  }

  if (!getStatusFlags(flags))
    return false;

  flags |= FD_CLOEXEC | IPPROTO_TCP | TCP_NODELAY;

  if (!setStatusFlags(flags))
    return false;

  int status = connect(file_descriptor_,
                       reinterpret_cast<const sockaddr*>(&sockaddress),
                       sizeof(sockaddress));
  if (status < 0)
  {
    switch (errno)
    {
    case ECONNREFUSED:
      ROS_ERROR("Connection refused");
      break;
    case ENETUNREACH:
      ROS_ERROR("No route to host");
      break;
    default:
      ROS_ERROR("NetFail");
      break;
    }
    closeConnection();
  }

  PioneerPacket packet;
  ROS_INFO("Starting synchronization");
  while (!synchronize(packet))
    ROS_ERROR("Could not synchronize, retrying");
  ROS_INFO("Synchronized");
  this->parseSynchronizationPacket(packet);
  ROS_INFO("Connected to a %s %s named %s",
           type_.c_str(),
           subtype_.c_str(),
           name_.c_str());
  return true;
}

void PioneerDriver::loadConfig(const std::string &filename)
{
  config_.loadFile(filename);
}

bool PioneerDriver::getSerialSettings(termios &settings)
{
  // Get the serial connection attributes
  int status;
  status = tcgetattr(file_descriptor_, &settings);
  if (status < 0)
  {
    ROS_ERROR("Failed to get connection attributes");
    return false;
  }
  return true;
}

bool PioneerDriver::setSerialSettings(const termios &settings)
{
  // Set the serial connection attributes
  int status;
  status = tcsetattr(file_descriptor_, TCSAFLUSH, &settings);
  if (status < 0)
  {
    ROS_ERROR("Failed to set connection attributes");
    return false;
  }

  if (!flushSerial())
    return false;
  return true;
}

bool PioneerDriver::flushSerial()
{
  int status;
  // Flush the input and output streams
  status = tcflush(file_descriptor_, TCIOFLUSH);
  if (status < 0)
  {
    ROS_ERROR("Failed to flush the input and output streams");
    return false;
  }
  return true;
}

bool PioneerDriver::getSerialDescriptorFlags(int &flags)
{
  // Get the flags
  flags = fcntl(file_descriptor_, F_GETFL);
  if (flags < 0)
  {
    ROS_ERROR("Failed to get the descriptor flags");
    return false;
  }
  return true;
}

bool PioneerDriver::setSerialDescriptorFlags(int &flags)
{
  int status;
  status = fcntl(file_descriptor_, F_SETFL, flags);
  if (status < 0)
  {
    ROS_ERROR("Failed to set the descriptor flags");
    return false;
  }
}

bool PioneerDriver::getStatusFlags(int &flags)
{
  // Get the flags
  flags = fcntl(file_descriptor_, F_GETFD);
  if (flags < 0)
  {
    ROS_ERROR("Failed to get the status flags");
    return false;
  }
  return true;
}

bool PioneerDriver::setStatusFlags(const int &flags)
{
  int status;
  status = fcntl(file_descriptor_, F_SETFD, flags);
  if (status < 0)
  {
    ROS_ERROR("Failed to set the status flags");
    return false;
  }
  return true;
}

void PioneerDriver::setSerialSpeed(termios &settings, int baudrate)
{
  cfsetispeed(&settings, baudrate);
  cfsetospeed(&settings, baudrate);
}

bool PioneerDriver::synchronize(PioneerPacket &packet)
{
  int flags;
  this->getSerialDescriptorFlags(flags);

  enum {
    IDLE,
    SYNCED_ONCE,
    SYNCED_TWICE,
    READY
  } sync_state;
  sync_state = IDLE;

  while (sync_state != READY) {
    switch (sync_state) {
    case IDLE:
      packet.command(SYNC0);
      packet.send(file_descriptor_);
      ROS_INFO("Sent SYNC0");
      break;
    case SYNCED_ONCE:
      ROS_INFO("Turning off NONBLOCK mode");
      fcntl(file_descriptor_, F_SETFL, flags ^ O_NONBLOCK) ;
      packet.command(SYNC1);
      packet.send(file_descriptor_);
      ROS_INFO("Sent SYNC1");
      break;
    case SYNCED_TWICE:
      packet.command(SYNC2);
      packet.send(file_descriptor_);
      ROS_INFO("Sent SYNC2");
      break;
    default:
      return false;
    }
    ros::Duration(0.2).sleep();
    if (!packet.receive(file_descriptor_))
      return false;
    switch (packet.at(1)) {
    case SYNC0:
      sync_state = SYNCED_ONCE;
      ROS_INFO("Received SYNC0");
      break;
    case SYNC1:
      sync_state = SYNCED_TWICE;
      ROS_INFO("Received SYNC1");
      break;
    case SYNC2:
      sync_state = READY;
      ROS_INFO("Received SYNC2");
      break;
    default:
      ROS_ERROR("Could not synchronize");
      return false;
    }
    ros::Duration(0.2).sleep();
  }
  return true;
}

void PioneerDriver::parseSynchronizationPacket(const PioneerPacket &packet)
{
  int index = 2;
  index = packet.getStringAt(index, name_);
  index = packet.getStringAt(index, type_);
  index = packet.getStringAt(index, subtype_);
}

void PioneerDriver::closeConnection()
{
  ROS_INFO("Closing connection");
  PioneerPacket packet;
  // Send STOP
  packet.command(STOP);
  packet.send(file_descriptor_);
  ros::Duration(0.5).sleep();
  // Send CLOSE
  packet.command(CLOSE);
  packet.send(file_descriptor_);
  ros::Duration(0.5).sleep();

  close(file_descriptor_);
  file_descriptor_ = -1;
}

void PioneerDriver::sendOpen()
{
  PioneerPacket packet;
  packet.command(OPEN);
  packet.send(file_descriptor_);
}

void PioneerDriver::sendEnable(bool value)
{
  PioneerPacket packet;
  packet.command(ENABLE, static_cast<int>(value));
  packet.send(file_descriptor_);
}

void PioneerDriver::sendPulse()
{
  PioneerPacket packet;
  packet.command(PULSE);
  packet.send(file_descriptor_);
}

}
