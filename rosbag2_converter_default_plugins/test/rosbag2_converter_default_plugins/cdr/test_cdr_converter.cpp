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

#include "../../../src/rosbag2_converter_default_plugins/cdr/cdr_converter.hpp"
#include "../../../src/rosbag2_converter_default_plugins/logging.hpp"
#include "rcutils/strdup.h"
#include "rosbag2/serialization_format_converter_interface.hpp"
#include "rosbag2/typesupport_helpers.hpp"
#include "rosbag2_test_common/memory_management.hpp"
#include "test_msgs/message_fixtures.hpp"

using rosbag2::SerializationFormatConverterInterface;
using namespace ::testing;  // NOLINT
using namespace rosbag2_test_common;  // NOLINT

class CdrConverterTestFixture : public Test
{
public:
  CdrConverterTestFixture()
  {
    converter_ = std::make_unique<rosbag2_converter_default_plugins::CdrConverter>();
    memory_management_ = std::make_unique<MemoryManagement>();
    topic_name_ = "test_topic";
    allocator_ = rcutils_get_default_allocator();
  }

  std::shared_ptr<rosbag2_ros2_message_t> make_shared_ros_message(
    const std::string & topic_name = "")
  {
    auto ros_message = std::make_shared<rosbag2_ros2_message_t>();
    ros_message->time_stamp = 0;
    ros_message->message = nullptr;
    ros_message->allocator = allocator_;

    if (!topic_name.empty()) {
      rosbag2::ros2_message_set_topic_name(ros_message.get(), topic_name.c_str());
    }

    return ros_message;
  }

  std::unique_ptr<rosbag2::SerializationFormatConverterInterface> converter_;
  std::unique_ptr<MemoryManagement> memory_management_;
  std::string topic_name_;
  rcutils_allocator_t allocator_;
};

TEST_F(CdrConverterTestFixture, deserialize_converts_cdr_into_ros_message_for_primitives) {
  auto message = get_messages_primitives()[0];
  message->string_value = "test_deserialize";
  message->float64_value = 102.34;
  message->int32_value = 10101010;
  auto serialized_data = memory_management_->serialize_message(message);
  auto serialized_message = std::make_shared<rosbag2::SerializedBagMessage>();
  serialized_message->serialized_data = serialized_data;
  serialized_message->topic_name = topic_name_;
  serialized_message->time_stamp = 1;

  auto ros_message = make_shared_ros_message();
  test_msgs::msg::Primitives primitive_test_msg;
  ros_message->message = &primitive_test_msg;
  auto type_support = rosbag2::get_typesupport("test_msgs/Primitives", "rosidl_typesupport_cpp");

  converter_->deserialize(serialized_message, type_support, ros_message);

  auto cast_message = static_cast<test_msgs::msg::Primitives *>(ros_message->message);
  EXPECT_THAT(*cast_message, Eq(*message));
  EXPECT_THAT(ros_message->time_stamp, Eq(serialized_message->time_stamp));
  EXPECT_THAT(ros_message->topic_name, StrEq(serialized_message->topic_name));
}

TEST_F(CdrConverterTestFixture, serialize_converts_ros_message_into_cdr_for_primitives) {
  auto ros_message = make_shared_ros_message(topic_name_);
  ros_message->time_stamp = 1;
  auto message = get_messages_primitives()[0];
  message->string_value = "test_serialize";
  message->float64_value = 102.34;
  message->int32_value = 10101010;
  ros_message->message = message.get();

  auto serialized_message = std::make_shared<rosbag2::SerializedBagMessage>();
  serialized_message->serialized_data = memory_management_->make_initialized_message();
  auto type_support = rosbag2::get_typesupport("test_msgs/Primitives", "rosidl_typesupport_cpp");

  converter_->serialize(ros_message, type_support, serialized_message);

  auto deserialized_msg = memory_management_->deserialize_message<test_msgs::msg::Primitives>(
    serialized_message->serialized_data);
  EXPECT_THAT(*deserialized_msg, Eq(*message));
  EXPECT_THAT(serialized_message->topic_name, StrEq(topic_name_));
  EXPECT_THAT(serialized_message->time_stamp, Eq(ros_message->time_stamp));
}

TEST_F(CdrConverterTestFixture, deserialize_converts_cdr_into_ros_message_for_static_array) {
  auto message = get_messages_static_array_primitives()[0];
  message->string_values = {{"test_deserialize", "another string", "the third one"}};
  message->float64_values = {{102.34, 1.9, 1236.011}};
  message->int32_values = {{11, 36, 219}};
  auto serialized_data = memory_management_->serialize_message(message);
  auto serialized_message = std::make_shared<rosbag2::SerializedBagMessage>();
  serialized_message->serialized_data = serialized_data;
  serialized_message->topic_name = topic_name_;
  serialized_message->time_stamp = 1;

  auto ros_message = make_shared_ros_message();
  test_msgs::msg::StaticArrayPrimitives primitive_test_msg;
  ros_message->message = &primitive_test_msg;
  auto type_support = rosbag2::get_typesupport(
    "test_msgs/StaticArrayPrimitives", "rosidl_typesupport_cpp");

  converter_->deserialize(serialized_message, type_support, ros_message);

  auto cast_message = static_cast<test_msgs::msg::StaticArrayPrimitives *>(ros_message->message);
  EXPECT_THAT(*cast_message, Eq(*message));
  EXPECT_THAT(ros_message->time_stamp, Eq(serialized_message->time_stamp));
  EXPECT_THAT(ros_message->topic_name, StrEq(serialized_message->topic_name));
}

TEST_F(CdrConverterTestFixture, serialize_converts_ros_message_into_cdr_for_static_array) {
  auto ros_message = make_shared_ros_message(topic_name_);
  ros_message->time_stamp = 1;
  auto message = get_messages_static_array_primitives()[0];
  message->string_values = {{"test_deserialize", "another string", "the third one"}};
  message->float64_values = {{102.34, 1.9, 1236.011}};
  message->int32_values = {{11, 36, 219}};
  ros_message->message = message.get();

  auto serialized_message = std::make_shared<rosbag2::SerializedBagMessage>();
  serialized_message->serialized_data = memory_management_->make_initialized_message();
  auto type_support = rosbag2::get_typesupport(
    "test_msgs/StaticArrayPrimitives", "rosidl_typesupport_cpp");

  converter_->serialize(ros_message, type_support, serialized_message);

  auto deserialized_msg = memory_management_->
    deserialize_message<test_msgs::msg::StaticArrayPrimitives>(serialized_message->serialized_data);
  EXPECT_THAT(*deserialized_msg, Eq(*message));
  EXPECT_THAT(serialized_message->topic_name, StrEq(topic_name_));
  EXPECT_THAT(serialized_message->time_stamp, Eq(ros_message->time_stamp));
}

TEST_F(CdrConverterTestFixture, deserialize_converts_cdr_into_ros_message_for_dynamic_array_nest) {
  auto message = get_messages_dynamic_array_nested()[0];
  test_msgs::msg::Primitives first_primitive_message;
  first_primitive_message.string_value = "I am the first";
  first_primitive_message.float32_value = 35.7f;
  test_msgs::msg::Primitives second_primitive_message;
  second_primitive_message.string_value = "I am the second";
  second_primitive_message.float32_value = 135.72f;
  message->primitive_values.push_back(first_primitive_message);
  message->primitive_values.push_back(second_primitive_message);
  auto serialized_data = memory_management_->serialize_message(message);
  auto serialized_message = std::make_shared<rosbag2::SerializedBagMessage>();
  serialized_message->serialized_data = serialized_data;
  serialized_message->topic_name = topic_name_;
  serialized_message->time_stamp = 1;

  auto ros_message = make_shared_ros_message();
  test_msgs::msg::DynamicArrayNested dynamic_nested_message;
  ros_message->message = &dynamic_nested_message;
  auto type_support = rosbag2::get_typesupport(
    "test_msgs/DynamicArrayNested", "rosidl_typesupport_cpp");

  converter_->deserialize(serialized_message, type_support, ros_message);

  auto cast_message = static_cast<test_msgs::msg::DynamicArrayNested *>(ros_message->message);
  EXPECT_THAT(*cast_message, Eq(*message));
  EXPECT_THAT(ros_message->time_stamp, Eq(serialized_message->time_stamp));
  EXPECT_THAT(ros_message->topic_name, StrEq(serialized_message->topic_name));
}

TEST_F(CdrConverterTestFixture, serialize_converts_ros_message_into_cdr_for_dynamic_array_nest) {
  auto ros_message = make_shared_ros_message(topic_name_);
  ros_message->time_stamp = 1;
  auto message = get_messages_dynamic_array_nested()[0];
  test_msgs::msg::Primitives first_primitive_message;
  first_primitive_message.string_value = "I am the first";
  first_primitive_message.float32_value = 35.7f;
  test_msgs::msg::Primitives second_primitive_message;
  second_primitive_message.string_value = "I am the second";
  second_primitive_message.float32_value = 135.72f;
  message->primitive_values.push_back(first_primitive_message);
  message->primitive_values.push_back(second_primitive_message);
  ros_message->message = message.get();

  auto serialized_message = std::make_shared<rosbag2::SerializedBagMessage>();
  serialized_message->serialized_data = memory_management_->make_initialized_message();
  auto type_support = rosbag2::get_typesupport(
    "test_msgs/DynamicArrayNested", "rosidl_typesupport_cpp");

  converter_->serialize(ros_message, type_support, serialized_message);

  auto deserialized_msg = memory_management_->
    deserialize_message<test_msgs::msg::DynamicArrayNested>(serialized_message->serialized_data);
  EXPECT_THAT(*deserialized_msg, Eq(*message));
  EXPECT_THAT(serialized_message->topic_name, StrEq(topic_name_));
  EXPECT_THAT(serialized_message->time_stamp, Eq(ros_message->time_stamp));
}
