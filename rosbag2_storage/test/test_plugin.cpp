// Copyright 2018,  Open Source Robotics Foundation, Inc.
// Copyright 2018,  Bosch Software Innovations GmbH.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <iostream>
#include <string>

#include "pluginlib/class_list_macros.hpp"

#include "test_plugin.hpp"

TestPlugin::~TestPlugin()
{
  std::cout << "\nclosing bag\n";
}

void TestPlugin::write(std::string message)
{
  (void) message;
  std::cout << "\nwriting\n";
}

std::string TestPlugin::read_next()
{
  std::cout << "\nreading\n";
  return "";
}

void TestPlugin::open(const std::string & uri)
{
  std::cout << "\nopened " << uri << ".\n";
}

void TestPlugin::open_readonly(const std::string & uri)
{
  std::cout << "\nopened readonly" << uri << ".\n";
}

rosbag2_storage::BagInfo TestPlugin::info()
{
  return rosbag2_storage::BagInfo();
}

void TestPlugin::create_topic()
{
  std::cout << "Created topic.\n";
}

bool TestPlugin::has_next()
{
  return true;
}

PLUGINLIB_EXPORT_CLASS(TestPlugin, rosbag2_storage::ReadWriteStorage)
