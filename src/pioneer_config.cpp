/*********************************************************************
* pioneer_config.cpp
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

#include <ros_pioneer/pioneer_config.h>
#include <boost/exception/diagnostic_information.hpp>

namespace ros_pioneer {

bool PioneerConfig::loadFile(const std::string &filename)
{
  std::ifstream file;
  try{
    file.open(filename.c_str());
  }
  catch (std::exception &e) {
    ROS_ERROR("%s", e.what());
  }
  this->parseFile(file);
  file.close();
}

void PioneerConfig::parseFile(std::ifstream &file)
{
  std::string line;
  while (std::getline(file, line))
    this->parseLine(line);
}

void PioneerConfig::parseLine(const std::string &line)
{
  static std::string section;
  std::stringstream iss(line);
  std::string data;
  std::getline(iss, data, ';');
  if (data.size() != 0 && !boost::starts_with(data, " ")) {
    boost::trim(data);
    std::vector<std::string> strings;
    boost::split(strings,
                 data,
                 boost::is_any_of("\t "),
                 boost::token_compress_on);
    if (strings[0].compare("Section") == 0)
      section = this->getSection(strings);
    else {
      if (strings.size() == 1) {
        // No data to be added!
      }
      else if (strings.size() == 2) {
        this->addToMap(section + strings[0], strings[1]);
      }
      else {
        this->addToMap(section + strings[0] + " " + strings[1], strings);
      }
    }
  }
}

bool PioneerConfig::isBool(const std::string &input)
{
  std::string temp_in = boost::to_lower_copy(input);
  if (temp_in.compare("true") == 0)
    return true;
  if (temp_in.compare("false")== 0)
    return true;
  return false;
}

bool PioneerConfig::toBool(const std::string &input)
{
  std::string temp_in = boost::to_lower_copy(input);
  std::istringstream is(temp_in);
  bool out;
  is >> std::boolalpha >> out;
  return out;
}

bool PioneerConfig::isInteger(const std::string &input)
{
  bool decimal = false;
  bool sign = false;
  for (size_t i = 0; i < input.size(); ++i) {
    if (input[i] == '.')
      return false;
    else if (input[i] == '-' && !sign)
      sign = true;
    else if (!std::isdigit(input[i]))
      return false;
  }
  return true;
}

int PioneerConfig::toInteger(const std::string &input)
{
  std::istringstream is(input);
  int out;
  is >> out;
  return out;
}

bool PioneerConfig::isDouble(const std::string &input)
{
  bool decimal = false;
  bool sign = false;
  for (size_t i = 0; i < input.size(); ++i) {
    if (input[i] == '.' && !decimal)
      decimal = true;
    else if (input[i] == '-' && !sign)
      sign = true;
    else if (!std::isdigit(input[i]))
      return false;
  }
  return true;
}

double PioneerConfig::toDouble(const std::string &input)
{
  std::istringstream is(input);
  double out;
  is >> out;
  return out;
}

std::vector<int> PioneerConfig::toVector(const std::vector<std::string> &input)
{
  std::vector<int> out;
  for (int i = 2; i < input.size(); ++i)
    out.push_back(this->toInteger(input[i]));
  return out;
}

void PioneerConfig::addToMap(const std::string &key, const std::string &value)
{
  if (isBool(value))
    configuration_[key] = toBool(value);
  else if (isInteger(value))
    configuration_[key] = toInteger(value);
  else if (isDouble(value))
    configuration_[key] = toDouble(value);
  else
    configuration_[key] = value;
}

void PioneerConfig::addToMap(const std::string &key,
                           const std::vector<std::string> &value)
{
  configuration_[key] = toVector(value);
}

std::string PioneerConfig::getSection(std::vector<std::string> strings)
{
  std::string section(strings[1]);
  if (strings.size() != 2) {
    if (isInteger(strings[2])) {
      section += " " + strings[2];
    }
  }
  section.append("/");
  return section;
}

bool PioneerConfig::getBool(const std::string &key) const
{
  bool value;
  std::map<std::string, boost::any>::const_iterator iter;
  iter = configuration_.find(key);
  try {
    if (iter != configuration_.end())
      value = boost::any_cast<bool>(iter->second);
    else
      ROS_ERROR("Key requested '%s' was not found", key.c_str());
  }
  catch (boost::exception &e) {
    ROS_ERROR("Key requested is not bool, %s",
              boost::diagnostic_information(e).c_str());
  }
  return (value);
}

void PioneerConfig::setBool(const std::string &key, bool value)
{
  configuration_[key] = value;
}

int PioneerConfig::getInt(const std::string &key) const
{
  int value;
  std::map<std::string, boost::any>::const_iterator iter;
  iter = configuration_.find(key);
  try {
    if (iter != configuration_.end())
      value = boost::any_cast<int>(iter->second);
    else
      ROS_ERROR("Key requested '%s' was not found", key.c_str());
  }
  catch (boost::exception &e) {
    ROS_ERROR("Key requested is not integer, %s",
              boost::diagnostic_information(e).c_str());
  }
  return (value);
}

void PioneerConfig::setInt(const std::string &key, int value)
{
  configuration_[key] = value;
}

double PioneerConfig::getDouble(const std::string &key) const
{
  double value;
  std::map<std::string, boost::any>::const_iterator iter;
  iter = configuration_.find(key);
  try {
    if (iter != configuration_.end())
      value = boost::any_cast<double>(iter->second);
    else
      ROS_ERROR("Key requested '%s' was not found", key.c_str());
  }
  catch (boost::exception &e) {
    ROS_ERROR("Key requested is not double, %s",
              boost::diagnostic_information(e).c_str());
  }
  return (value);
}

void PioneerConfig::setDouble(const std::string &key, double value)
{
  configuration_[key] = value;
}

std::string PioneerConfig::getString(const std::string &key) const
{
  std::string value;
  std::map<std::string, boost::any>::const_iterator iter;
  iter = configuration_.find(key);
  try {
    if (iter != configuration_.end())
      value = boost::any_cast<std::string>(iter->second);
    else
      ROS_ERROR("Key requested '%s' was not found", key.c_str());
  }
  catch (boost::exception &e) {
    ROS_ERROR("Key requested is not string, %s",
              boost::diagnostic_information(e).c_str());
  }
  return (value);
}

void PioneerConfig::setString(const std::string &key, const std::string &value)
{
  configuration_[key] = value;
}

std::vector<int> PioneerConfig::getIntVector(const std::string &key) const
{
  std::vector<int> value;
  std::map<std::string, boost::any>::const_iterator iter;
  iter = configuration_.find(key);
  try {
    if (iter != configuration_.end())
      value = boost::any_cast<std::vector<int> >(iter->second);
    else
      ROS_ERROR("Key requested '%s' was not found", key.c_str());
  }
  catch (boost::exception &e) {
    ROS_ERROR("Key requested is not a vector of integers, %s",
              boost::diagnostic_information(e).c_str());
  }
  return (value);
}

void PioneerConfig::setIntVector(const std::string &key, const std::vector<int> &value)
{
  configuration_[key] = value;
}

}
