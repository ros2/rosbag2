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

class TestStorageFactory : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
  }

  void SetUp()
  {
  }
};

TEST_F(TestStorageFactory, load_test_plugin) {
  rosbag2_storage::StorageFactory factory("my_test_plugin");

  auto instance = factory.get_storage_interface();
  EXPECT_STREQ("my_test_plugin", instance->get_storage_identifier().c_str());
  SUCCEED();
}
