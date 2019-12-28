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

#include "rosbag2_cpp/typesupport_helpers.hpp"
#include "rosbag2_cpp/types/introspection_message.hpp"

#include "test_msgs/message_fixtures.hpp"
#include "test_msgs/msg/arrays.hpp"
#include "test_msgs/msg/basic_types.hpp"
#include "test_msgs/msg/bounded_sequences.hpp"
#include "test_msgs/msg/multi_nested.hpp"
#include "test_msgs/msg/nested.hpp"
#include "test_msgs/msg/strings.hpp"
#include "test_msgs/msg/unbounded_sequences.hpp"

using namespace testing;  // NOLINT

// NOTE: These tests should be run with valgrind (or leak_sanitizers) and there should be *no*
// memory leaks or problems with free!

class Ros2MessageTest : public Test
{
public:
  Ros2MessageTest()
  {
    allocator_ = rcutils_get_default_allocator();
  }

  auto get_allocated_message(const std::string & message_type)
  {
    auto introspection_ts =
      rosbag2_cpp::get_typesupport(message_type, "rosidl_typesupport_introspection_cpp");

    return rosbag2_cpp::allocate_introspection_message(introspection_ts, &allocator_);
  }

  rcutils_allocator_t allocator_;
};

TEST_F(Ros2MessageTest, allocate_ros2_message_allocates_basic_types_message) {
  auto message = get_allocated_message("test_msgs/BasicTypes");

  auto data = static_cast<test_msgs::msg::BasicTypes *>(message->message);

  data->bool_value = true;
  data->int16_value = 144;
}

TEST_F(Ros2MessageTest, allocate_ros2_message_allocates_strings_message_with_empty_string) {
  auto message = get_allocated_message("test_msgs/Strings");

  if (!message) {
    fprintf(stderr, "could not get string message\n");
  }
  auto data = static_cast<test_msgs::msg::Strings *>(message->message);
  if (!data) {
    fprintf(stderr, "allocation failed for string\n");
  }

  data->string_value = "";
}

TEST_F(Ros2MessageTest, allocate_ros2_message_allocates_strings_message_with_big_string_no_SSO) {
  auto message = get_allocated_message("test_msgs/Strings");

  auto data = static_cast<test_msgs::msg::Strings *>(message->message);

  data->string_value = std::string(1000, 's');
}

TEST_F(Ros2MessageTest, allocate_ros2_message_allocates_nested_message) {
  auto message = get_allocated_message("test_msgs/Nested");

  auto data = static_cast<test_msgs::msg::Nested *>(message->message);

  data->basic_types_value.bool_value = true;
  data->basic_types_value.int16_value = 143;
}

TEST_F(Ros2MessageTest, allocate_ros2_message_allocates_arrays_message) {
  auto message = get_allocated_message("test_msgs/Arrays");

  auto data = static_cast<test_msgs::msg::Arrays *>(message->message);

  data->bool_values = {{true, false, true}};
  data->string_values = {{"eins", "zwei", std::string(1000, 'd')}};
  data->int32_values = {{11, 22, 33}};
}

TEST_F(Ros2MessageTest, allocate_ros2_message_allocates_arrays_message_with_empty_string) {
  auto message = get_allocated_message("test_msgs/Arrays");

  auto data = static_cast<test_msgs::msg::Arrays *>(message->message);

  data->bool_values = {{true, false, true}};
  data->string_values = {{"", "zwei", std::string(1000, 'a')}};
  data->int32_values = {{11, 22, 33}};

  data->basic_types_values[0].bool_value = true;
  data->basic_types_values[0].int64_value = 123;
  data->basic_types_values[1].bool_value = false;
  data->basic_types_values[1].int64_value = 123;
}

TEST_F(Ros2MessageTest, allocate_ros2_message_allocates_unbounded_sequences_message) {
  auto message = get_allocated_message("test_msgs/UnboundedSequences");

  auto data = static_cast<test_msgs::msg::UnboundedSequences *>(message->message);

  data->bool_values = {{true, false, true, false}};
  data->string_values = {{"eins", std::string(1000, 'b'), ""}};
  data->int32_values = {{11, 22, 33, 44, 55}};

  data->basic_types_values.push_back(*get_messages_basic_types()[1]);
  data->basic_types_values.push_back(*get_messages_basic_types()[2]);
}

TEST_F(Ros2MessageTest, allocate_ros2_message_allocates_bounded_sequences_message) {
  auto message = get_allocated_message("test_msgs/BoundedSequences");

  auto data = static_cast<test_msgs::msg::BoundedSequences *>(message->message);

  data->bool_values = {{true, false, true}};
  data->string_values = {{"eins", std::string(1000, 'z'), ""}};
  data->int32_values = {11};

  data->basic_types_values.push_back(*get_messages_basic_types()[1]);
  data->basic_types_values.push_back(*get_messages_basic_types()[2]);
}

TEST_F(Ros2MessageTest, allocate_ros2_message_allocates_multi_nested_message) {
  auto message = get_allocated_message("test_msgs/MultiNested");

  auto data = static_cast<test_msgs::msg::MultiNested *>(message->message);

  data->array_of_arrays[0] = *get_messages_arrays()[0];

  data->bounded_sequence_of_bounded_sequences.push_back(
    *get_messages_bounded_sequences()[0]);
  data->bounded_sequence_of_bounded_sequences.push_back(
    *get_messages_bounded_sequences()[1]);

  data->unbounded_sequence_of_unbounded_sequences.push_back(
    *get_messages_unbounded_sequences()[0]);
  data->unbounded_sequence_of_unbounded_sequences.push_back(
    *get_messages_unbounded_sequences()[0]);
}

TEST_F(Ros2MessageTest, allocate_ros2_message_cleans_up_topic_name_on_shutdown) {
  auto message = get_allocated_message("test_msgs/BoundedSequences");

  rosbag2_cpp::introspection_message_set_topic_name(message.get(), "Topic name");

  EXPECT_THAT(message->topic_name, StrEq("Topic name"));
}

TEST_F(Ros2MessageTest, duplicate_set_topic_does_not_leak) {
  auto message = get_allocated_message("test_msgs/BoundedSequences");

  rosbag2_cpp::introspection_message_set_topic_name(message.get(), "Topic name");
  rosbag2_cpp::introspection_message_set_topic_name(message.get(), "Topic name");

  EXPECT_THAT(message->topic_name, StrEq("Topic name"));
}
