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

#include <gtest/gtest.h>

#include <memory>
#include <string>

#include "rosbag2_storage/storage_interfaces/read_only_interface.hpp"
#include "rosbag2_storage/storage_interfaces/read_write_interface.hpp"

#include "rosbag2_storage/storage_factory.hpp"

#include "plugin_constants.hpp"

using rosbag2_storage::storage_interfaces::ReadWriteInterface;
using rosbag2_storage::storage_interfaces::ReadOnlyInterface;

// The fixture for testing class Foo.
class StorageFactoryTest : public ::testing::Test
{
public:
  rosbag2_storage::StorageFactory factory;

  std::string bag_file_path = "file/to/be/loaded.bag";
  std::string test_plugin_id = "my_test_plugin";
  std::string test_read_only_plugin_id = "my_read_only_test_plugin";
  std::string test_unavailable_plugin_id = "my_unavailable_plugin";
};

TEST_F(StorageFactoryTest, load_test_plugin) {
  // Load plugin for read and write
  auto read_write_storage = factory.open_read_write(
    bag_file_path, test_plugin_id);
  ASSERT_NE(nullptr, read_write_storage);

  EXPECT_EQ(
    plugin_constants::DUMMY_FILEPATH,
    read_write_storage->get_relative_path());

  EXPECT_EQ(
    plugin_constants::READ_WRITE_PLUGIN_IDENTIFIER,
    read_write_storage->get_storage_identifier());

  auto msg = read_write_storage->read_next();
  read_write_storage->write(msg);

  // Load plugin for read only even though it provides read and write interfaces
  auto read_only_storage = factory.open_read_only(
    bag_file_path, test_plugin_id);
  ASSERT_NE(nullptr, read_only_storage);
  msg = read_only_storage->read_next();
}

TEST_F(StorageFactoryTest, loads_readonly_plugin_only_for_read_only_storage) {
  auto storage_for_reading = factory.open_read_only(
    bag_file_path, test_read_only_plugin_id);
  ASSERT_NE(nullptr, storage_for_reading);

  EXPECT_EQ(
    plugin_constants::DUMMY_FILEPATH,
    storage_for_reading->get_relative_path());

  EXPECT_EQ(
    plugin_constants::READ_ONLY_PLUGIN_IDENTIFIER,
    storage_for_reading->get_storage_identifier());

  storage_for_reading->read_next();

  auto storage_for_reading_and_writing = factory.open_read_write(
    bag_file_path, test_read_only_plugin_id);
  ASSERT_EQ(nullptr, storage_for_reading_and_writing);
}

TEST_F(StorageFactoryTest, load_unavailable_plugin) {
  auto instance_rw = factory.open_read_write(
    bag_file_path, test_unavailable_plugin_id);
  EXPECT_EQ(nullptr, instance_rw);

  auto instance_ro = factory.open_read_only(
    bag_file_path, test_unavailable_plugin_id);
  EXPECT_EQ(nullptr, instance_ro);
}
