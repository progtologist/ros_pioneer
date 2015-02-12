/*********************************************************************
* arcos_config.cpp
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

#include <ros_arcos/arcos_config.h>

namespace ros_arcos{

ArcosConfig::~ArcosConfig()
{
  if (file_.is_open())
    file_.close();
}

bool ArcosConfig::loadFile(const std::string &filename)
{
  try{
    file_.open(filename.c_str());
  }
  catch (std::exception &e) {
    ROS_ERROR("%s", e.what());
  }
}

void ArcosConfig::parseFile()
{

}

void ArcosConfig::parseLine(const std::string &line)
{
  std::stringstream iss(line);
  std::string data;
  std::string variable_name;
  std::string variable_value;
  std::getline(iss, data, ';');
  if (data.size() != 0) {
    std::stringstream ss(data);
    std::getline(ss, variable_name, ' ');
    std::getline(ss, variable_value);
  }
  ROS_INFO("Name is: %s", variable_name.c_str());
  ROS_INFO("Value is: %s", variable_value.c_str());
}

bool ArcosConfig::isBool(const std::string &input)
{
  std::string temp_in = input;
  std::transform(temp_in.begin(), temp_in.end(), temp_in.begin(), ::tolower);
  if (temp_in.compare("true") == 0)
    return true;
  if (temp_in.compare("false")== 0)
    return true;
  return false;
}

bool ArcosConfig::toBool(const std::string &input)
{
  std::string temp_in = input;
  std::transform(temp_in.begin(), temp_in.end(), temp_in.begin(), ::tolower);
  std::istringstream is(temp_in);
  bool out;
  is >> std::boolalpha >> out;
  return out;
}

bool ArcosConfig::isInteger(const std::string &input)
{
  bool decimal = false;
  for (size_t i = 0; i < input.size(); ++i) {
    if (input[i] == '.')
      return false;
    else if (!std::isdigit(input[i]))
      return false;
  }
  return true;
}

int ArcosConfig::toInteger(const std::string &input)
{
  std::istringstream is(input);
  int out;
  is >> out;
  return out;
}

bool ArcosConfig::isDouble(const std::string &input)
{
  bool decimal = false;
  for (size_t i = 0; i < input.size(); ++i) {
    if (input[i] == '.' && !decimal)
      decimal = true;
    else if (!std::isdigit(input[i]))
      return false;
  }
  return true;
}

double ArcosConfig::toDouble(const std::string &input)
{
  std::istringstream is(input);
  double out;
  is >> out;
  return out;
}

}
