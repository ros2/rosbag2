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

#include <gmock/gmock.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rosbag2/sequential_reader.hpp"
#include "rosbag2_storage/bag_metadata.hpp"
#include "rosbag2_storage/topic_with_type.hpp"
#include "rosbag2/typesupport_helpers.hpp"
#include "rosbag2/types/ros2_message.hpp"
#include "test_msgs/msg/bounded_array_nested.hpp"
#include "test_msgs/msg/bounded_array_primitives.hpp"
#include "test_msgs/msg/dynamic_array_nested.hpp"
#include "test_msgs/msg/dynamic_array_primitives.hpp"
#include "test_msgs/msg/nested.hpp"
#include "test_msgs/msg/primitives.hpp"
#include "test_msgs/msg/static_array_nested.hpp"
#include "test_msgs/msg/static_array_primitives.hpp"
#include "test_msgs/message_fixtures.hpp"

using namespace testing;  // NOLINT

// NOTE: These tests should be run with valgrind (or leak_sanitizers) and there should be *no*
// memory leaks or problems with free!

class Ros2MessageTest : public Test
{
public:
  auto get_allocated_message(const std::string & message_type)
  {
    auto introspection_ts =
      rosbag2::get_typesupport(message_type, "rosidl_typesupport_introspection_cpp");

    return rosbag2::allocate_ros2_message(introspection_ts);
  }
};

TEST_F(Ros2MessageTest, allocate_ros2_message_allocates_primitive_message) {
  auto message = get_allocated_message("test_msgs/Primitives");

  auto data = static_cast<test_msgs::msg::Primitives *>(message->message);

  data->bool_value = true;
  data->string_value = "content";
  data->int16_value = 144;
}

TEST_F(Ros2MessageTest, allocate_ros2_message_allocates_primitive_message_with_empty_string) {
  auto message = get_allocated_message("test_msgs/Primitives");

  auto data = static_cast<test_msgs::msg::Primitives *>(message->message);

  data->bool_value = true;
  data->string_value = "";
  data->int16_value = 144;
}

TEST_F(Ros2MessageTest, allocate_ros2_message_allocates_nested_message) {
  auto message = get_allocated_message("test_msgs/Nested");

  auto data = static_cast<test_msgs::msg::Nested *>(message->message);

  data->primitive_values.bool_value = true;
  data->primitive_values.string_value = "content";
  data->primitive_values.int16_value = 143;
}

TEST_F(Ros2MessageTest, allocate_ros2_message_allocates_static_array_message) {
  auto message = get_allocated_message("test_msgs/StaticArrayPrimitives");

  auto data = static_cast<test_msgs::msg::StaticArrayPrimitives *>(message->message);

  data->bool_values = {true, false, true};
  data->string_values = {"eins", "zwei", "drei"};
  data->int32_values = {11, 22, 33};
}

TEST_F(Ros2MessageTest, allocate_ros2_message_allocates_static_array_message_with_empty_string) {
  auto message = get_allocated_message("test_msgs/StaticArrayPrimitives");

  auto data = static_cast<test_msgs::msg::StaticArrayPrimitives *>(message->message);

  data->bool_values = {true, false, true};
  data->string_values = {"", "zwei", ""};
  data->int32_values = {11, 22, 33};
}

TEST_F(Ros2MessageTest, allocate_ros2_message_allocates_dynamic_array_message) {
  auto message = get_allocated_message("test_msgs/DynamicArrayPrimitives");

  auto data = static_cast<test_msgs::msg::DynamicArrayPrimitives *>(message->message);

  data->bool_values = {true, false, true, false};
  data->string_values = {"eins", "zwei"};
  data->int32_values = {11, 22, 33, 44, 55};
}

TEST_F(Ros2MessageTest, allocate_ros2_message_allocates_bounded_array_message) {
  auto message = get_allocated_message("test_msgs/BoundedArrayPrimitives");

  auto data = static_cast<test_msgs::msg::BoundedArrayPrimitives *>(message->message);

  data->bool_values = {true, false, true};
  data->string_values = {"eins", "zwei"};
  data->int32_values = {11};
}

TEST_F(Ros2MessageTest, allocate_ros2_message_allocates_static_array_nested_message) {
  auto message = get_allocated_message("test_msgs/StaticArrayNested");

  auto data = static_cast<test_msgs::msg::StaticArrayNested *>(message->message);

  data->primitive_values[0].bool_value = true;
  data->primitive_values[0].string_value = "eins";
  data->primitive_values[0].int64_value = 123;
  data->primitive_values[1].bool_value = false;
  data->primitive_values[1].string_value = "zwei";
  data->primitive_values[1].int64_value = 123;
}

TEST_F(Ros2MessageTest, allocate_ros2_message_allocates_dynamic_array_nested_message) {
  auto message = get_allocated_message("test_msgs/DynamicArrayNested");

  auto data = static_cast<test_msgs::msg::DynamicArrayNested *>(message->message);

  data->primitive_values.push_back(*get_messages_primitives()[1]);
  data->primitive_values.push_back(*get_messages_primitives()[2]);
}

TEST_F(Ros2MessageTest, allocate_ros2_message_allocates_bounded_array_nested_message) {
  auto message = get_allocated_message("test_msgs/BoundedArrayNested");

  auto data = static_cast<test_msgs::msg::BoundedArrayNested *>(message->message);

  data->primitive_values.push_back(*get_messages_primitives()[1]);
  data->primitive_values.push_back(*get_messages_primitives()[2]);
}
