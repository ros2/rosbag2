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
#include <string>

#include "pluginlib/class_list_macros.hpp"

#include "test_read_only_plugin.hpp"

TestReadOnlyPlugin::~TestReadOnlyPlugin()
{
  std::cout << "\nclosing bag\n";
}

void TestReadOnlyPlugin::open(const std::string & uri)
{
  std::cout << "\nopened " << uri << ".\n";
}

rosbag2_storage::BagInfo TestReadOnlyPlugin::info()
{
  return rosbag2_storage::BagInfo();
}

rosbag2_storage::SerializedBagMessage TestReadOnlyPlugin::read_next()
{
  std::cout << "\nreading\n";
  return rosbag2_storage::SerializedBagMessage();
}

PLUGINLIB_EXPORT_CLASS(TestReadOnlyPlugin, rosbag2_storage::storage_interfaces::ReadOnlyInterface)
