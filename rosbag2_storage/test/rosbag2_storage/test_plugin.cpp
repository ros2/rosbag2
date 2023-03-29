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

#include "test_constants.hpp"
#include "test_plugin.hpp"

TestPlugin::~TestPlugin()
{
  std::cout << "\nclosing.\n";
}

void TestPlugin::open(
  const rosbag2_storage::StorageOptions & storage_options,
  rosbag2_storage::storage_interfaces::IOFlag flag)
{
  if (storage_options.storage_id != test_constants::READ_WRITE_PLUGIN_IDENTIFIER) {
    throw std::runtime_error{"storage_id did not match. TestReadOnlyPlugin won't open."};
  }
  if (flag == rosbag2_storage::storage_interfaces::IOFlag::READ_ONLY) {
    std::cout << "opening testplugin read only: ";
  } else if (flag == rosbag2_storage::storage_interfaces::IOFlag::READ_WRITE) {
    std::cout << "opening testplugin read write: ";
  }
  std::cout << "storage uri: " << storage_options.uri << ".\n";
  std::cout << "config file uri: " << storage_options.storage_config_uri << ".\n";
}

void TestPlugin::update_metadata(const rosbag2_storage::BagMetadata & metadata)
{
  std::cout << "Set metadata" << std::endl;
  (void)metadata;
}

bool TestPlugin::set_read_order(const rosbag2_storage::ReadOrder & order)
{
  std::cout << "Set read order " << order.sort_by << " " << order.reverse << std::endl;
  return true;
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

void TestPlugin::create_topic(
  const rosbag2_storage::TopicMetadata & topic,
  const rosbag2_storage::MessageDefinition & message_definition)
{
  std::cout << "Created topic with name =" << topic.name << ", type =" << topic.type <<
    "and message definition encoding " << message_definition.encoding << ".\n";
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

void TestPlugin::write(
  const std::vector<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>> & msg)
{
  (void) msg;
  std::cout << "\nwriting multiple\n";
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

std::string TestPlugin::get_relative_file_path() const
{
  std::cout << "\nreturning relative path\n";
  return test_constants::DUMMY_FILEPATH;
}

uint64_t TestPlugin::get_bagfile_size() const
{
  std::cout << "\nreturning bagfile size\n";
  return test_constants::MAX_BAGFILE_SIZE;
}

std::string TestPlugin::get_storage_identifier() const
{
  std::cout << "\nreturning storage identifier\n";
  return test_constants::READ_WRITE_PLUGIN_IDENTIFIER;
}

uint64_t TestPlugin::get_minimum_split_file_size() const
{
  std::cout << "\nreturning minimum split file size\n";
  return test_constants::MIN_SPLIT_FILE_SIZE;
}

void TestPlugin::set_filter(
  const rosbag2_storage::StorageFilter & /*storage_filter*/)
{
  std::cout << "\nsetting storage filter\n";
}

void TestPlugin::reset_filter()
{
  std::cout << "\nresetting storage filter\n";
}

void TestPlugin::seek(const rcutils_time_point_value_t & /*timestamp*/)
{
  std::cout << "\nseeking\n";
}

PLUGINLIB_EXPORT_CLASS(TestPlugin, rosbag2_storage::storage_interfaces::ReadWriteInterface)
