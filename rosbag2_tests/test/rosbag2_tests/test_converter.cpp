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
#include "rosbag2/types/introspection_message.hpp"
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
    cdr_serializer_ = factory_->load_serializer("cdr");
    cdr_deserializer_ = factory_->load_deserializer("cdr");
    type_support_ =
      rosbag2::get_typesupport("test_msgs/MultiNested", "rosidl_typesupport_cpp");
  }

  std::shared_ptr<rosbag2_introspection_message_t> allocate_empty_dynamic_array_message()
  {
    auto introspection_type_support = rosbag2::get_typesupport(
      "test_msgs/MultiNested", "rosidl_typesupport_introspection_cpp");
    auto introspection_message =
      rosbag2::allocate_introspection_message(introspection_type_support, &allocator_);
    introspection_message->time_stamp = 1;
    rosbag2::introspection_message_set_topic_name(introspection_message.get(), topic_name_.c_str());
    return introspection_message;
  }

  void fill_dynamic_array_message(std::shared_ptr<rosbag2_introspection_message_t> message)
  {
    auto correctly_typed_ros_message =
      reinterpret_cast<test_msgs::msg::MultiNested *>(message->message);
    auto multi_nested = get_messages_multi_nested()[0];
    correctly_typed_ros_message->array_of_arrays = multi_nested->array_of_arrays;
  }

  std::unique_ptr<rosbag2::SerializationFormatConverterFactory> factory_;
  std::unique_ptr<rosbag2::converter_interfaces::SerializationFormatSerializer>
  cdr_serializer_;
  std::unique_ptr<rosbag2::converter_interfaces::SerializationFormatDeserializer>
  cdr_deserializer_;
  std::unique_ptr<MemoryManagement> memory_management_;
  std::string topic_name_;
  rcutils_allocator_t allocator_;
  const rosidl_message_type_support_t * type_support_;
};

TEST_F(ConverterTestFixture, cdr_converter_plugin_can_serialize_and_deserialize_messages) {
  auto initial_ros_message = allocate_empty_dynamic_array_message();
  fill_dynamic_array_message(initial_ros_message);

  auto serialized_message = std::make_shared<rosbag2::SerializedBagMessage>();
  serialized_message->serialized_data = memory_management_->make_initialized_message();

  cdr_serializer_->serialize(initial_ros_message, type_support_, serialized_message);

  auto final_roundtrip_ros_message = allocate_empty_dynamic_array_message();

  cdr_deserializer_->deserialize(serialized_message, type_support_, final_roundtrip_ros_message);
  serialized_message.reset();

  EXPECT_THAT(
    *static_cast<test_msgs::msg::MultiNested *>(final_roundtrip_ros_message->message),
    Eq(*static_cast<test_msgs::msg::MultiNested *>(initial_ros_message->message)));
  EXPECT_THAT(final_roundtrip_ros_message->topic_name, StrEq(initial_ros_message->topic_name));
  EXPECT_THAT(final_roundtrip_ros_message->time_stamp, Eq(initial_ros_message->time_stamp));
}
