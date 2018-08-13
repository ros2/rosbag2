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

#include "rosbag2_storage/storage_factory.hpp"


TEST(StorageFactoryTest, load_test_plugin) {
  rosbag2_storage::StorageFactory factory;

  auto storage_for_reading = factory.get_read_only_storage(
    "my_test_plugin", "file/to/be/read.bag");
  std::string message;
  storage_for_reading->read_next(message);
  storage_for_reading->read_next(message);
  storage_for_reading->read_next(message);
  storage_for_reading.reset();

  auto read_write_storage = factory.get_read_write_storage(
    "my_test_plugin", "file/to/be/read_and_written.bag");
  read_write_storage->write("");
  read_write_storage->write("");
  read_write_storage->write("");
  read_write_storage->read_next(message);
  read_write_storage->read_next(message);
  read_write_storage->read_next(message);
  read_write_storage.reset();
}

TEST(StorageFactoryTest, loads_readonly_plugin_only_for_read_only_storage) {
  rosbag2_storage::StorageFactory factory;

  std::string message;
  auto storage_for_reading = factory.get_read_only_storage(
    "my_read_only_test_plugin", "file/to/be/read.bag");
  storage_for_reading->read_next(message);
  storage_for_reading->read_next(message);
  storage_for_reading->read_next(message);
  storage_for_reading.reset();

  auto storage_for_reading_and_writing = factory.get_read_write_storage(
    "my_read_only_test_plugin", "file/to/be/read.bag");
  EXPECT_FALSE(storage_for_reading_and_writing);
}
