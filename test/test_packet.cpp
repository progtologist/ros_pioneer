/*********************************************************************
* test_packet.cpp
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

#include <gtest/gtest.h>
#include <ros_arcos/arcos_packet.h>

using namespace ros_arcos;
using namespace std;

TEST(ArcosPacket, Init_Commands)
{
  {
    ArcosPacket pac;
    pac.command(SYNC0);
    EXPECT_EQ(pac[0],250);
    EXPECT_EQ(pac[1],251);
    EXPECT_EQ(pac[2],3);
    EXPECT_EQ(pac[3],0);
    EXPECT_EQ(pac[4],0);
    EXPECT_EQ(pac[5],0);
  }

  {
    ArcosPacket pac;
    pac.command(SYNC1);
    EXPECT_EQ(pac[0],250);
    EXPECT_EQ(pac[1],251);
    EXPECT_EQ(pac[2],3);
    EXPECT_EQ(pac[3],1);
    EXPECT_EQ(pac[4],0);
    EXPECT_EQ(pac[5],1);
  }

  {
    ArcosPacket pac;
    pac.command(SYNC2);
    EXPECT_EQ(pac[0],250);
    EXPECT_EQ(pac[1],251);
    EXPECT_EQ(pac[2],3);
    EXPECT_EQ(pac[3],2);
    EXPECT_EQ(pac[4],0);
    EXPECT_EQ(pac[5],2);
  }
}

TEST(ArcosPacket, Void_Commands)
{
  ArcosPacket pac;
  pac.command(PULSE);

  EXPECT_EQ(pac[0],0xFA);
  EXPECT_EQ(pac[1],0xFB);
  EXPECT_EQ(pac[2],0x03);
  EXPECT_EQ(pac[3],0x00);
  EXPECT_EQ(pac[4],0x00);
  EXPECT_EQ(pac[5],0x00);
}

TEST(ArcosPacket, Int_Commands)
{
  ArcosPacket pac;
  pac.command(HOSTBAUD, 3);
  EXPECT_EQ(pac[0],0xFA);
  EXPECT_EQ(pac[1],0xFB);
  EXPECT_EQ(pac[2],0x06);
  EXPECT_EQ(pac[3],0x32);
  EXPECT_EQ(pac[4],0x3B);
  EXPECT_EQ(pac[5],0x03);
  EXPECT_EQ(pac[6],0x00);
  EXPECT_EQ(pac[7],0x35);
  EXPECT_EQ(pac[8],0x3B);
}

TEST(ArcosPacket, TwoBytes_Char_Commands)
{
  ArcosPacket pac;
  pac.command(ARM_POS, 25, 3);
  EXPECT_EQ(pac[0],0xFA);
  EXPECT_EQ(pac[1],0xFB);
  EXPECT_EQ(pac[2],0x06);
  EXPECT_EQ(pac[3],0x4D);
  EXPECT_EQ(pac[4],0x3B);
  EXPECT_EQ(pac[5],0x19);
  EXPECT_EQ(pac[6],0x03);
  EXPECT_EQ(pac[7],0x66);
  EXPECT_EQ(pac[8],0x3E);
}

TEST(ArcosPacket, TwoBytes_Vector_Commands)
{
  ArcosPacket pac;
  std::vector<bool> f_byte(8,0);
  f_byte[0] = 1;
  f_byte[2] = 1;
  std::vector<bool> s_byte(4,0);
  s_byte[1] = 1;
  s_byte[3] = 1;
  pac.command(DIGOUT, f_byte, s_byte);
  EXPECT_EQ(pac[0],0xFA);
  EXPECT_EQ(pac[1],0xFB);
  EXPECT_EQ(pac[2],0x06);
  EXPECT_EQ(pac[3],0x1E);
  EXPECT_EQ(pac[4],0x3B);
  EXPECT_EQ(pac[5],0x05);
  EXPECT_EQ(pac[6],0x0A);
  EXPECT_EQ(pac[7],0x23);
  EXPECT_EQ(pac[8],0x45);
}

TEST(ArcosPacket, String_Commands)
{
  ArcosPacket pac;
  pac.command(TTY2, "TEST");
  EXPECT_EQ(pac[0],0xFA);
  EXPECT_EQ(pac[1],0xFB);
  EXPECT_EQ(pac[2],0x09);
  EXPECT_EQ(pac[3],0x2A);
  EXPECT_EQ(pac[4],0x2B);
  EXPECT_EQ(pac[5],0x04);
  EXPECT_EQ(pac[6],0x54);
  EXPECT_EQ(pac[7],0x45);
  EXPECT_EQ(pac[8],0x53);
  EXPECT_EQ(pac[9],0x54);
  EXPECT_EQ(pac[10],0x73);
  EXPECT_EQ(pac[11],0x86);
}

TEST(ArcosPacket, At_Accessor)
{
  ArcosPacket pac;
  pac.command(PULSE);
  EXPECT_EQ(pac.at(0),0x03);
  EXPECT_EQ(pac.at(1),0x00);
}

TEST(ArcosPacket, Get_Unsigned_Integer_At_Accessor)
{
  ArcosPacket pac;
  pac.command(HOSTBAUD, 40000);     // Value larger than 32767
  EXPECT_EQ(pac.at(0), 6);          // Size of packet
  EXPECT_EQ(pac.at(1), HOSTBAUD);   // The command
  EXPECT_EQ(pac.at(2), ARGINT);     // The data type
  int value, index;
  index = pac.getUnsignedAt(3, value);
  EXPECT_EQ(value, 40000);          // The Integer
  EXPECT_EQ(index, 5);              // The index of the next values
}

TEST(ArcosPacket, Get_Integer_At_Accessor)
{
  ArcosPacket pac;
  pac.command(HOSTBAUD, 300);
  EXPECT_EQ(pac.at(0), 6);          // Size of packet
  EXPECT_EQ(pac.at(1), HOSTBAUD);   // The command
  EXPECT_EQ(pac.at(2), ARGINT);     // The data type
  int value, index;
  index = pac.getIntegerAt(3, value);
  EXPECT_EQ(value, 300);            // The Integer
  EXPECT_EQ(index, 5);              // The index of the next values
}

TEST(ArcosPacket, Get_String_At_Length_Accessor)
{
  ArcosPacket pac;
  pac.command(TTY2, "TEST");
  EXPECT_EQ(pac.at(0), 9);                            // Size of packet
  EXPECT_EQ(pac.at(1), TTY2);                    // The command
  EXPECT_EQ(pac.at(2), ARGSTR);                  // The data type
  EXPECT_EQ(pac.at(3), 4);                            // The size of the string
  std::string value;
  int index;
  index = pac.getStringAt(4, 4, value);
  EXPECT_STREQ(value.c_str(),"TEST");  // The String
  EXPECT_EQ(index, 9);
}

TEST(ArcosPacket, Get_String_At_Accessor)
{
  ArcosPacket pac;
  pac.command(TTY2, "NULL TERMINATED");
  EXPECT_EQ(pac.at(0), 20);
  EXPECT_EQ(pac.at(1), TTY2);
  EXPECT_EQ(pac.at(2), ARGSTR);
  EXPECT_EQ(pac.at(3), 15);
  std::string value;
  int index;
  index = pac.getStringAt(4, value);
  EXPECT_STREQ(value.c_str(), "NULL TERMINATED");
  EXPECT_EQ(index, 20);
}

TEST(ArcosPacket, Get_Bool_Vector_At_Accessor)
{
  ArcosPacket pac;
  pac.command(ENABLE, 21); // Just for testing
  std::vector<bool> value;
  int index;
  index = pac.getBoolVectorAt(3, value);
  EXPECT_EQ(index, 4);
  EXPECT_EQ(value.size(), 8);
  EXPECT_EQ(value[0], 1);
  EXPECT_EQ(value[1], 0);
  EXPECT_EQ(value[2], 1);
  EXPECT_EQ(value[3], 0);
  EXPECT_EQ(value[4], 1);
  EXPECT_EQ(value[5], 0);
  EXPECT_EQ(value[6], 0);
  EXPECT_EQ(value[7], 0);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "google_test_packet");
  return RUN_ALL_TESTS();
}
