// Copyright 2018, Bosch Software Innovations GmbH.
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
#include <memory>
#include <string>

#include "pluginlib/class_list_macros.hpp"

#include "test_read_only_plugin.hpp"

TestReadOnlyPlugin::~TestReadOnlyPlugin()
{
  close();
}

void TestReadOnlyPlugin::open(
  const std::string & uri, rosbag2_storage::storage_interfaces::IOFlag flag)
{
  if (flag == rosbag2_storage::storage_interfaces::IOFlag::READ_ONLY) {
    std::cout << "opening testplugin read only: ";
  } else if (flag == rosbag2_storage::storage_interfaces::IOFlag::READ_WRITE) {
    std::cout << "opening testplugin read write: ";
  }
  std::cout << uri << ".\n";
  is_open_ = true;
}

bool TestReadOnlyPlugin::is_open() const {return is_open_;}

void TestReadOnlyPlugin::close()
{
  std::cout << "\nclosing.\n";
  is_open_ = false;
}

rosbag2_storage::BagInfo TestReadOnlyPlugin::info()
{
  return rosbag2_storage::BagInfo();
}

std::shared_ptr<rosbag2_storage::SerializedBagMessage> TestReadOnlyPlugin::read_next()
{
  std::cout << "\nreading\n";
  return std::shared_ptr<rosbag2_storage::SerializedBagMessage>();
}

PLUGINLIB_EXPORT_CLASS(TestReadOnlyPlugin, rosbag2_storage::storage_interfaces::ReadOnlyInterface)
