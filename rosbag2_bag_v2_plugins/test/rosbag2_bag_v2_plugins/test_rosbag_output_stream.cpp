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

#include <gmock/gmock.h>

#include <memory>
#include <string>

#include "../../src/rosbag2_bag_v2_plugins/storage/rosbag_output_stream.hpp"

using namespace ::testing;  // NOLINT
using RosbagOutputStream = rosbag2_bag_v2_plugins::RosbagOutputStream;

TEST(RosbagOutputStream, constructor_correctly_initializes_ros_message_type)
{
  auto rosbag_output_stream = RosbagOutputStream("std_msgs/String");

  auto data_type = std::string(
    reinterpret_cast<char *>(rosbag_output_stream.get_content()->buffer));
  EXPECT_THAT(data_type, StrEq("std_msgs/String"));
}

TEST(RosbagOutputStream, advance_correctly_make_space_for_message)
{
  std::string expected_data_type = "std_msgs/String";
  auto rosbag_output_stream = RosbagOutputStream(expected_data_type);

  auto data_pointer = rosbag_output_stream.advance(10);

  auto serialized_message = rosbag_output_stream.get_content();
  auto data_type = std::string(reinterpret_cast<char *>(serialized_message->buffer));
  EXPECT_THAT(data_type, StrEq(expected_data_type));
  EXPECT_THAT(serialized_message->buffer_capacity, Eq(10 + expected_data_type.length() + 1));
  EXPECT_THAT(serialized_message->buffer_length, Eq(10 + expected_data_type.length() + 1));
  EXPECT_THAT(data_pointer, Eq(serialized_message->buffer + expected_data_type.length() + 1));
}

TEST(RosbagOutputStream, adding_data_to_message_works_correctly)
{
  std::string added_data = "some_added_data";
  std::string expected_data_type = "std_msgs/String";
  auto rosbag_output_stream = RosbagOutputStream(expected_data_type);

  memcpy(
    rosbag_output_stream.advance(added_data.length() + 1),
    added_data.c_str(),
    added_data.size() + 1);

  auto serialized_message = rosbag_output_stream.get_content();
  auto data_type = std::string(reinterpret_cast<char *>(serialized_message->buffer));
  EXPECT_THAT(data_type, StrEq(expected_data_type));
  EXPECT_THAT(
    serialized_message->buffer_capacity, Eq(added_data.length() + expected_data_type.length() + 2));
  EXPECT_THAT(
    serialized_message->buffer_length, Eq(added_data.length() + expected_data_type.length() + 2));
  auto additional_data_pointer = reinterpret_cast<char *>(
    serialized_message->buffer + expected_data_type.length() + 1);
  EXPECT_THAT(std::string(additional_data_pointer), StrEq(added_data));
}
