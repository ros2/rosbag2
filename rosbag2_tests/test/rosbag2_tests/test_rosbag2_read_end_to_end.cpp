// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#include <gmock/gmock.h>

#include <iostream>
#include <memory>
#include <string>

// rclcpp must be included before process_execution_helpers.hpp
#include "rclcpp/rclcpp.hpp"
#include "rcpputils/filesystem_helper.hpp"

#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_cpp/storage_options.hpp"
#include "rosbag2_cpp/converter_options.hpp"

#include "rosbag2_storage/storage_filter.hpp"

#include "rosbag2_storage_default_plugins/sqlite/sqlite_storage.hpp"

#include "test_msgs/msg/arrays.hpp"
#include "test_msgs/msg/basic_types.hpp"

using namespace ::testing;  // NOLINT

class ReadEndToEndTestFixture : public Test
{
public:
  ReadEndToEndTestFixture()
  {
    database_path_ = _SRC_RESOURCES_DIR_PATH;  // variable defined in CMakeLists.txt
    reader_ = std::make_unique<rosbag2_cpp::readers::SequentialReader>();
  }

  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  std::string database_path_;
  std::unique_ptr<rosbag2_cpp::readers::SequentialReader> reader_;
};

TEST_F(ReadEndToEndTestFixture, read_end_to_end_test) {
  rosbag2_cpp::StorageOptions storage_options;
  storage_options.uri = (rcpputils::fs::path(database_path_) / "talker").string();
  storage_options.storage_id = "sqlite3";

  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";

  rosbag2_storage::StorageFilter storage_filter;
  storage_filter.topics.push_back("/topic");

  reader_->open(storage_options, converter_options);
  reader_->set_filter(storage_filter);

  std::shared_ptr<rosbag2_storage::SerializedBagMessage> msg;

  EXPECT_TRUE(reader_->has_next());
  msg = reader_->read_next();
  EXPECT_EQ(msg->topic_name, "/topic");
  EXPECT_TRUE(reader_->has_next());
  msg = reader_->read_next();
  EXPECT_EQ(msg->topic_name, "/topic");

  // Add a topic and re-open file
  reader_->reset();
  reader_->open(storage_options, converter_options);
  storage_filter.topics.push_back("/rosout");
  reader_->set_filter(storage_filter);

  EXPECT_TRUE(reader_->has_next());
  msg = reader_->read_next();
  EXPECT_EQ(msg->topic_name, "/rosout");

  EXPECT_TRUE(reader_->has_next());
  msg = reader_->read_next();
  EXPECT_EQ(msg->topic_name, "/topic");

  // Clear filter object
  reader_->reset();
  reader_->open(storage_options, converter_options);
  storage_filter.topics.clear();
  reader_->set_filter(storage_filter);

  EXPECT_TRUE(reader_->has_next());
  msg = reader_->read_next();
  EXPECT_EQ(msg->topic_name, "/rosout");

  EXPECT_TRUE(reader_->has_next());
  msg = reader_->read_next();
  EXPECT_EQ(msg->topic_name, "/topic");

  // Set and reset reader filter
  reader_->reset();
  reader_->open(storage_options, converter_options);
  storage_filter.topics.push_back("/rosout");
  reader_->set_filter(storage_filter);
  reader_->reset_filter();

  EXPECT_TRUE(reader_->has_next());
  msg = reader_->read_next();
  EXPECT_EQ(msg->topic_name, "/rosout");

  EXPECT_TRUE(reader_->has_next());
  msg = reader_->read_next();
  EXPECT_EQ(msg->topic_name, "/topic");
}
