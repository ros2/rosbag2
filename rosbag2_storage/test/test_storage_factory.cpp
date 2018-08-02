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

  auto storage_for_writing = factory.get_for_writing("my_test_plugin", "file/to/be/written.bag");
  size_t size = 0;
  storage_for_writing->write(nullptr, size);
  storage_for_writing->write(nullptr, size);
  storage_for_writing->write(nullptr, size);
  storage_for_writing.reset();

  auto storage_for_reading = factory.get_for_reading("my_test_plugin", "file/to/be/read.bag");
  storage_for_reading->read_next(nullptr, size);
  storage_for_reading->read_next(nullptr, size);
  storage_for_reading->read_next(nullptr, size);
}
