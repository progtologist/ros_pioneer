/*********************************************************************
* pioneer_config.h
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

#ifndef PIONEER_CONFIG_H
#define PIONEER_CONFIG_H

#include <ros/ros.h>
#include <string>
#include <fstream>
#include <map>
#include <boost/any.hpp>
#include <boost/algorithm/string.hpp>

namespace ros_pioneer {

class PioneerConfig
{
public:
  bool loadFile(const std::string &filename);
  bool getBool(const std::string &key) const;
  void setBool(const std::string &key, bool value);
  int getInt(const std::string &key) const;
  void setInt(const std::string &key, int value);
  double getDouble(const std::string &key) const;
  void setDouble(const std::string &key, double value);
  std::string getString(const std::string &key) const;
  void setString(const std::string &key, const std::string &value);
  std::vector<int> getIntVector(const std::string &key) const;
  void setIntVector(const std::string &key, const std::vector<int> &value);
private:
  void parseFile(std::ifstream &file);
  void parseLine(const std::string &line);
  std::string getSection(std::vector<std::string> strings);
  bool isBool(const std::string &input);
  bool toBool(const std::string &input);
  bool isInteger(const std::string &input);
  int toInteger(const std::string &input);
  bool isDouble(const std::string &input);
  double toDouble(const std::string &input);
  std::vector<int> toVector(const std::vector<std::string> &input);
  void addToMap(const std::string &key, const std::string &value);
  void addToMap(const std::string &key, const std::vector<std::string> &value);

  std::map<std::string, boost::any> configuration_;
};

}

#endif // PIONEER_CONFIG_H
