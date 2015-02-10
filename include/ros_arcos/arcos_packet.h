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
/**
 * @brief The ArcosPacket class
 */
class ArcosPacket
{
public:
  /**
   * @brief command Create a packet with an initialization command
   * @param command The initialization that the packet contains.
   */
  void command(const Init_t &command);
  /**
   * @brief command Create a packet with a command that takes no arguments
   * @param command The command that the packet contains.
   */
  void command(const Void_t &command);
  /**
   * @brief command Create a packet with a command that takes an integer argument
   * @param command The command that the packet contains.
   * @param value The argument of the command.
   */
  void command(const Int_t &command, int value);
  /**
   * @brief command Create a packet with a command that takes a two byte argument
   * @param command The command that the packet contains.
   * @param first_byte The first byte of the argument.
   * @param second_byte The second byte of the argument.
   */
  void command(const TwoBytes_t &command,
               unsigned char first_byte,
               unsigned char second_byte);
  /**
   * @brief command Create a packet with a command that takes a string argument
   * @param command The command that the packet contains.
   * @param msg The string argument of the command.
   */
  void command(const Str_t &command,
               const std::string &msg);
  /**
   * @brief clear Empties the packet contents
   */
  void clear();
  /**
   * @brief send Writes the packet to the given file descriptor
   * @param file_descriptor The file descriptor to write to.
   * @return True if successfull, false otherwise.
   */
  bool send(int file_descriptor);
  /**
   * @brief receive Reads the packet from the given file descriptor
   * @param file_descriptor The file descriptor to read from.
   * @return True if successfull, false otherwise.
   */
  bool receive(int file_descriptor);
  /**
   * @brief operator [] The accessor to the packet data. All the data are accessible
   * @param i The byte to be read.
   * @return The unsigned character of the requested byte.
   */
  unsigned char operator [](int i) const;
  /**
   * @brief at The accessor to the packet data. The header and checksum data are
   * skipped from this interface.
   * @param i The byte to be read.
   * @return The unsigned character of the requested byte.
   */
  unsigned char at(int i) const;
  /**
   * @brief getIntegerAt Returns the integer stored at the current and next byte.
   * @param i The first byte of the integer.
   * @return The integer stored.
   */
  int getIntegerAt(int i) const;
  /**
   * @brief getStringAt Returns the string stored starting at the current byte
   * ending with a null character.
   * @param i The first byte of the string.
   * @return The string stored.
   */
  std::string getStringAt(int i) const;
  /**
   * @brief getStringAt Returns the string stored starting at the current byte
   * with a fixed length.
   * @param i The first byte of the string.
   * @param length The length of the string.
   * @return The string stored.
   */
  std::string getStringAt(int i, size_t length) const;
  /**
   * @brief printHex Print to the ros console the Hexadecimal bytes of the package.
   */
  void printHex();
  /**
   * @brief printDec Print to the ros console the Decimal bytes of the package.
   */
  void printDec();
  /**
   * @brief printASCII Print to the ros console the ASCII characters of the package.
   */
  void printASCII();
protected:
  int calculateChecksum();
  bool check();
  void setHeader();
  void setSize(unsigned char size);
  void setCommand(unsigned char command);
  void setType(const Types_t &type);
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
