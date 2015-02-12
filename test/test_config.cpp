/*********************************************************************
* test_config.cpp
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
#include <ros_arcos/arcos_config.h>
#include <ros_arcos/config.h>
#include <boost/exception/diagnostic_information.hpp>
#include <boost/exception_ptr.hpp>

using namespace ros_arcos;
using namespace std;
using boost::any;
using boost::any_cast;

TEST(ArcosConfig, FileParser)
{
  ArcosConfig con;
  string package_path(PACKAGE_PATH);
  string open_path(package_path);
  open_path.append("/config/params/p3dx.p");
  con.loadFile(open_path);
  con.parseFile();

  map<string,any> config;
  config = con.configuration();
  bool holonomic = any_cast<bool>(config["General/Holonomic"]);
  EXPECT_EQ(holonomic, true);
  int max_vel = any_cast<int>(config["General/MaxVelocity"]);
  EXPECT_EQ(max_vel, 2200);
  double grlr = any_cast<double>(config["General/RobotLengthRear"]);
  EXPECT_FLOAT_EQ(grlr, 301.0);
  string v7va = any_cast<string>(config["Video 7/VideoAddress"]);
  EXPECT_STREQ(v7va.c_str(), "192.168.0.90");
  vector<int> sonar_0 = any_cast<vector<int> >(config["Sonar/SonarUnit 0"]);
  EXPECT_EQ(sonar_0.size(), 9);
  EXPECT_EQ(sonar_0[0], 69);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "google_test_config");
  return RUN_ALL_TESTS();
}