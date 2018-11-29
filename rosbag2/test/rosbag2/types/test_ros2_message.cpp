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

#include "rosbag2/typesupport_helpers.hpp"
#include "rosbag2/types/introspection_message.hpp"
#include "test_msgs/msg/bounded_array_nested.hpp"
#include "test_msgs/msg/bounded_array_primitives_nested.hpp"
#include "test_msgs/msg/bounded_array_primitives.hpp"
#include "test_msgs/msg/dynamic_array_primitives_nested.hpp"
#include "test_msgs/msg/dynamic_array_nested.hpp"
#include "test_msgs/msg/dynamic_array_primitives.hpp"
#include "test_msgs/msg/dynamic_array_static_array_primitives_nested.hpp"
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
  Ros2MessageTest()
  {
    allocator_ = rcutils_get_default_allocator();
  }

  auto get_allocated_message(const std::string & message_type)
  {
    auto introspection_ts =
      rosbag2::get_typesupport(message_type, "rosidl_typesupport_introspection_cpp");

    return rosbag2::allocate_introspection_message(introspection_ts, &allocator_);
  }

  rcutils_allocator_t allocator_;
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

TEST_F(Ros2MessageTest, allocate_ros2_message_allocates_primitive_message_with_big_string_no_SSO) {
  auto message = get_allocated_message("test_msgs/Primitives");

  auto data = static_cast<test_msgs::msg::Primitives *>(message->message);

  data->bool_value = true;
  data->string_value = std::string(1000, 's');
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

  data->bool_values = {{true, false, true}};
  data->string_values = {{"eins", "zwei", std::string(1000, 'd')}};
  data->int32_values = {{11, 22, 33}};
}

TEST_F(Ros2MessageTest, allocate_ros2_message_allocates_static_array_message_with_empty_string) {
  auto message = get_allocated_message("test_msgs/StaticArrayPrimitives");

  auto data = static_cast<test_msgs::msg::StaticArrayPrimitives *>(message->message);

  data->bool_values = {{true, false, true}};
  data->string_values = {{"", "zwei", std::string(1000, 'a')}};
  data->int32_values = {{11, 22, 33}};
}

TEST_F(Ros2MessageTest, allocate_ros2_message_allocates_dynamic_array_message) {
  auto message = get_allocated_message("test_msgs/DynamicArrayPrimitives");

  auto data = static_cast<test_msgs::msg::DynamicArrayPrimitives *>(message->message);

  data->bool_values = {{true, false, true, false}};
  data->string_values = {{"eins", std::string(1000, 'b'), ""}};
  data->int32_values = {{11, 22, 33, 44, 55}};
}

TEST_F(Ros2MessageTest, allocate_ros2_message_allocates_bounded_array_message) {
  auto message = get_allocated_message("test_msgs/BoundedArrayPrimitives");

  auto data = static_cast<test_msgs::msg::BoundedArrayPrimitives *>(message->message);

  data->bool_values = {{true, false, true}};
  data->string_values = {{"eins", std::string(1000, 'z'), ""}};
  data->int32_values = {11};
}

TEST_F(Ros2MessageTest, allocate_ros2_message_allocates_static_array_nested_message) {
  auto message = get_allocated_message("test_msgs/StaticArrayNested");

  auto data = static_cast<test_msgs::msg::StaticArrayNested *>(message->message);

  data->primitive_values[0].bool_value = true;
  data->primitive_values[0].string_value = "eins";
  data->primitive_values[0].int64_value = 123;
  data->primitive_values[1].bool_value = false;
  data->primitive_values[1].string_value = std::string(1000, '2');
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

TEST_F(Ros2MessageTest, allocate_ros2_message_allocates_nested_static_array_nested_message) {
  auto message = get_allocated_message("test_msgs/DynamicArrayStaticArrayPrimitivesNested");

  auto data =
    static_cast<test_msgs::msg::DynamicArrayStaticArrayPrimitivesNested *>(message->message);

  data->dynamic_array_static_array_primitive_values.push_back(
    *get_messages_static_array_primitives()[0]);
  data->dynamic_array_static_array_primitive_values.push_back(
    *get_messages_static_array_primitives()[0]);
}

TEST_F(Ros2MessageTest, allocate_ros2_message_allocates_nested_dynamic_array_nested_message) {
  auto message = get_allocated_message("test_msgs/DynamicArrayPrimitivesNested");

  auto data =
    static_cast<test_msgs::msg::DynamicArrayPrimitivesNested *>(message->message);

  data->dynamic_array_primitive_values.push_back(
    *get_messages_dynamic_array_primitives()[1]);
  data->dynamic_array_primitive_values.push_back(
    *get_messages_dynamic_array_primitives()[2]);
}

TEST_F(Ros2MessageTest, allocate_ros2_message_allocates_nested_bounded_array_nested_message) {
  auto message = get_allocated_message("test_msgs/BoundedArrayPrimitivesNested");

  auto data =
    static_cast<test_msgs::msg::BoundedArrayPrimitivesNested *>(message->message);

  data->bounded_array_primitive_values.push_back(
    *get_messages_bounded_array_primitives()[0]);
  data->bounded_array_primitive_values.push_back(
    *get_messages_bounded_array_primitives()[1]);
}

TEST_F(Ros2MessageTest, allocate_ros2_message_cleans_up_topic_name_on_shutdown) {
  auto message = get_allocated_message("test_msgs/BoundedArrayNested");

  rosbag2::introspection_message_set_topic_name(message.get(), "Topic name");

  EXPECT_THAT(message->topic_name, StrEq("Topic name"));
}

TEST_F(Ros2MessageTest, duplicate_set_topic_does_not_leak) {
  auto message = get_allocated_message("test_msgs/BoundedArrayNested");

  rosbag2::introspection_message_set_topic_name(message.get(), "Topic name");
  rosbag2::introspection_message_set_topic_name(message.get(), "Topic name");

  EXPECT_THAT(message->topic_name, StrEq("Topic name"));
}
