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
#include <memory>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

#include "rosbag2_storage/serialized_bag_message.hpp"

#include "test_plugin.hpp"

TestPlugin::~TestPlugin()
{
  std::cout << "\nclosing.\n";
}

void TestPlugin::open(
  const std::string & uri, rosbag2_storage::storage_interfaces::IOFlag flag)
{
  if (flag == rosbag2_storage::storage_interfaces::IOFlag::READ_ONLY) {
    std::cout << "opening testplugin read only: ";
  } else if (flag == rosbag2_storage::storage_interfaces::IOFlag::READ_WRITE) {
    std::cout << "opening testplugin read write: ";
  }
  std::cout << uri << ".\n";
}

bool TestPlugin::has_next()
{
  return true;
}

std::shared_ptr<rosbag2_storage::SerializedBagMessage> TestPlugin::read_next()
{
  std::cout << "\nreading\n";
  return std::shared_ptr<rosbag2_storage::SerializedBagMessage>();
}

void TestPlugin::create_topic(const rosbag2_storage::TopicMetadata & topic)
{
  std::cout << "Created topic with name =" << topic.name << " and type =" << topic.type << ".\n";
}

void TestPlugin::remove_topic(const rosbag2_storage::TopicMetadata & topic)
{
  std::cout << "Removed topic with name =" << topic.name << " and type =" << topic.type << ".\n";
}

void TestPlugin::write(const std::shared_ptr<const rosbag2_storage::SerializedBagMessage> msg)
{
  (void) msg;
  std::cout << "\nwriting\n";
}

std::vector<rosbag2_storage::TopicMetadata> TestPlugin::get_all_topics_and_types()
{
  std::cout << "\nreading topics and types\n";
  return std::vector<rosbag2_storage::TopicMetadata>();
}

rosbag2_storage::BagMetadata TestPlugin::get_metadata()
{
  std::cout << "\nreturning metadata\n";
  return rosbag2_storage::BagMetadata();
}

uint64_t TestPlugin::get_bagfile_size() const
{
  std::cout << "\nreturning bagfile size\n";
  return default_max_bagfile_size;
}

std::string TestPlugin::get_identifier() const
{
  return "TestPlugin";
}

PLUGINLIB_EXPORT_CLASS(TestPlugin, rosbag2_storage::storage_interfaces::ReadWriteInterface)
