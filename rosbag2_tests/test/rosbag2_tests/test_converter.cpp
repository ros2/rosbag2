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

#include "rcutils/strdup.h"
#include "rosbag2/serialization_format_converter_factory.hpp"
#include "rosbag2/typesupport_helpers.hpp"
#include "rosbag2/types/ros2_message.hpp"
#include "rosbag2_test_common/memory_management.hpp"
#include "test_msgs/message_fixtures.hpp"

using rosbag2::SerializationFormatConverterFactory;
using namespace ::testing;  // NOLINT
using namespace rosbag2_test_common;  // NOLINT

class ConverterTestFixture : public Test
{
public:
  ConverterTestFixture()
  {
    factory_ = std::make_unique<rosbag2::SerializationFormatConverterFactory>();
    memory_management_ = std::make_unique<MemoryManagement>();
    topic_name_ = "test_topic";
    allocator_ = rcutils_get_default_allocator();
    cdr_converter_ = factory_->load_converter("cdr");
  }
  std::shared_ptr<rosbag2_ros2_message_t>

  allocate_empty_dynamic_array_message()
  {
    auto introspection_type_support = rosbag2::get_typesupport(
      "test_msgs/DynamicArrayNested", "rosidl_typesupport_introspection_cpp");
    auto ros_message = rosbag2::allocate_ros2_message(introspection_type_support, &allocator_);
    ros_message->timestamp = 1;
    ros_message->topic_name = topic_name_.c_str();
    return ros_message;
  }

  std::unique_ptr<rosbag2::SerializationFormatConverterFactory> factory_;
  std::unique_ptr<rosbag2::SerializationFormatConverterInterface> cdr_converter_;
  std::unique_ptr<MemoryManagement> memory_management_;
  std::string topic_name_;
  rcutils_allocator_t allocator_;
};

TEST_F(ConverterTestFixture, serialize_converts_ros_message_into_cdr_for_dynamic_array_nested) {
  auto ros_message = allocate_empty_dynamic_array_message();

  auto correctly_typed_ros_message = reinterpret_cast<test_msgs::msg::DynamicArrayNested *>(
    ros_message->message);
  auto primitive_msgs = get_messages_primitives();
  for (const auto & primitive_msg : primitive_msgs) {
    correctly_typed_ros_message->primitive_values.push_back(*primitive_msg);
  }
  // The message should be equivalent to the following message:
  auto message = get_messages_dynamic_array_nested()[0];
  EXPECT_THAT(*correctly_typed_ros_message, Eq(*message));

  auto serialized_message = std::make_shared<rosbag2::SerializedBagMessage>();
  serialized_message->serialized_data = memory_management_->make_initialized_message();
  auto type_support = rosbag2::get_typesupport(
    "test_msgs/DynamicArrayNested", "rosidl_typesupport_cpp");

  cdr_converter_->serialize(ros_message, type_support, serialized_message);

  auto deserialized_msg = memory_management_->
    deserialize_message<test_msgs::msg::DynamicArrayNested>(serialized_message->serialized_data);
  EXPECT_THAT(*deserialized_msg, Eq(*message));
  EXPECT_THAT(serialized_message->topic_name, StrEq(topic_name_));
  EXPECT_THAT(serialized_message->time_stamp, Eq(ros_message->timestamp));
}

TEST_F(ConverterTestFixture, deserialize_converts_ros_message_into_cdr_for_dynamic_array_nested) {
  auto ros_message = allocate_empty_dynamic_array_message();

  auto message = get_messages_dynamic_array_nested()[0];
  auto bag_message = std::make_shared<rosbag2::SerializedBagMessage>();
  bag_message->topic_name = topic_name_;
  bag_message->time_stamp = 1;
  bag_message->serialized_data = memory_management_->serialize_message(message);

  auto type_support = rosbag2::get_typesupport(
    "test_msgs/DynamicArrayNested", "rosidl_typesupport_cpp");

  cdr_converter_->deserialize(bag_message, type_support, ros_message);

  auto deserialized_message =
    static_cast<test_msgs::msg::DynamicArrayNested *>(ros_message->message);
  EXPECT_THAT(*deserialized_message, Eq(*message));
  EXPECT_THAT(ros_message->topic_name, StrEq(topic_name_));
}
