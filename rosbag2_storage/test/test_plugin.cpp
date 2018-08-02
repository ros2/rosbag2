// Copyright 2018 Open Source Robotics Foundation, Inc.
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

bool TestPlugin::write(const char * data, size_t size)
{
  (void) data;
  (void) size;
  std::cout << "\nwriting\n";
  return true;
}

bool TestPlugin::read_next(const char * buffer, size_t & size)
{
  (void) buffer;
  (void) size;
  std::cout << "\nreading\n";
  return true;
}

void TestPlugin::open_for_reading(const std::string & uri)
{
  std::cout << "\nopened " << uri << " for reading.\n";
}

void TestPlugin::open_for_writing(const std::string & uri)
{
  std::cout << "\nopened " << uri << " for writing.\n";
}

rosbag2_storage::BagInfo TestPlugin::info()
{
  return rosbag2_storage::BagInfo();
}

PLUGINLIB_EXPORT_CLASS(TestPlugin, rosbag2_storage::WritableStorage)
PLUGINLIB_EXPORT_CLASS(TestPlugin, rosbag2_storage::ReadableStorage)
