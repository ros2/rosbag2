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
#include <vector>

#include "pluginlib/class_list_macros.hpp"

#include "test_constants.hpp"
#include "test_read_only_plugin.hpp"

TestReadOnlyPlugin::~TestReadOnlyPlugin()
{
  std::cout << "\nclosing.\n";
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
}

bool TestReadOnlyPlugin::has_next()
{
  return true;
}

std::shared_ptr<rosbag2_storage::SerializedBagMessage> TestReadOnlyPlugin::read_next()
{
  std::cout << "\nreading\n";
  return std::shared_ptr<rosbag2_storage::SerializedBagMessage>();
}

std::vector<rosbag2_storage::TopicMetadata> TestReadOnlyPlugin::get_all_topics_and_types()
{
  std::cout << "\nreading topics and types\n";
  return std::vector<rosbag2_storage::TopicMetadata>();
}

std::string TestReadOnlyPlugin::get_relative_file_path() const
{
  std::cout << "\nreturning relative path\n";
  return test_constants::DUMMY_FILEPATH;
}

rosbag2_storage::BagMetadata TestReadOnlyPlugin::get_metadata()
{
  std::cout << "\nreturning bag metadata\n";
  return rosbag2_storage::BagMetadata();
}

uint64_t TestReadOnlyPlugin::get_bagfile_size() const
{
  std::cout << "\nreturning bagfile size\n";
  return test_constants::MAX_BAGFILE_SIZE;
}

std::string TestReadOnlyPlugin::get_storage_identifier() const
{
  std::cout << "\nreturning storage identifier\n";
  return test_constants::READ_ONLY_PLUGIN_IDENTIFIER;
}

void TestReadOnlyPlugin::set_filter(
  const std::shared_ptr<rosbag2_storage::StorageFilter> & /*storage_filter*/)
{
  std::cout << "\nsetting storage filter\n";
}

PLUGINLIB_EXPORT_CLASS(TestReadOnlyPlugin, rosbag2_storage::storage_interfaces::ReadOnlyInterface)
