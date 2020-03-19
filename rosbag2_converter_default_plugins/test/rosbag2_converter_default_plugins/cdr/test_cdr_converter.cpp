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

#include "rcpputils/shared_library.hpp"
#include "rcutils/strdup.h"

#include "rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp"
#include "rosbag2_cpp/typesupport_helpers.hpp"
#include "rosbag2_cpp/types.hpp"

#include "rosbag2_test_common/memory_management.hpp"

#include "test_msgs/message_fixtures.hpp"

#include "../../../src/rosbag2_converter_default_plugins/cdr/cdr_converter.hpp"
#include "../../../src/rosbag2_converter_default_plugins/logging.hpp"

using rosbag2_cpp::converter_interfaces::SerializationFormatConverter;
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

  std::shared_ptr<rosbag2_cpp::rosbag2_introspection_message_t> make_shared_ros_message(
    const std::string & topic_name = "")
  {
    auto ros_message = std::make_shared<rosbag2_cpp::rosbag2_introspection_message_t>();
    ros_message->time_stamp = 0;
    ros_message->message = nullptr;
    ros_message->allocator = allocator_;

    if (!topic_name.empty()) {
      rosbag2_cpp::introspection_message_set_topic_name(ros_message.get(), topic_name.c_str());
    }

    return ros_message;
  }

  std::unique_ptr<SerializationFormatConverter> converter_;
  std::unique_ptr<MemoryManagement> memory_management_;
  std::string topic_name_;
  rcutils_allocator_t allocator_;
};

TEST_F(CdrConverterTestFixture, deserialize_converts_cdr_into_ros_message_for_primitives) {
  auto message = get_messages_basic_types()[1];
  auto serialized_data = memory_management_->serialize_message(message);
  auto serialized_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  serialized_message->serialized_data = serialized_data;
  serialized_message->topic_name = topic_name_;
  serialized_message->time_stamp = 1;

  auto ros_message = make_shared_ros_message();
  test_msgs::msg::BasicTypes primitive_test_msg;
  ros_message->message = &primitive_test_msg;
  std::shared_ptr<rcpputils::SharedLibrary> library;
  auto type_support =
    rosbag2_cpp::get_typesupport("test_msgs/BasicTypes", "rosidl_typesupport_cpp", library);

  converter_->deserialize(serialized_message, type_support, ros_message);

  auto cast_message = static_cast<test_msgs::msg::BasicTypes *>(ros_message->message);
  EXPECT_THAT(*cast_message, Eq(*message));
  EXPECT_THAT(ros_message->time_stamp, Eq(serialized_message->time_stamp));
  EXPECT_THAT(ros_message->topic_name, StrEq(serialized_message->topic_name));
}

TEST_F(CdrConverterTestFixture, serialize_converts_ros_message_into_cdr_for_primitives) {
  auto ros_message = make_shared_ros_message(topic_name_);
  ros_message->time_stamp = 1;
  auto message = get_messages_basic_types()[1];
  ros_message->message = message.get();

  auto serialized_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  serialized_message->serialized_data = memory_management_->make_initialized_message();
  std::shared_ptr<rcpputils::SharedLibrary> library;
  auto type_support =
    rosbag2_cpp::get_typesupport("test_msgs/BasicTypes", "rosidl_typesupport_cpp", library);

  converter_->serialize(ros_message, type_support, serialized_message);

  auto deserialized_msg = memory_management_->deserialize_message<test_msgs::msg::BasicTypes>(
    serialized_message->serialized_data);
  EXPECT_THAT(*deserialized_msg, Eq(*message));
  EXPECT_THAT(serialized_message->topic_name, StrEq(topic_name_));
  EXPECT_THAT(serialized_message->time_stamp, Eq(ros_message->time_stamp));
}

TEST_F(CdrConverterTestFixture, deserialize_converts_cdr_into_ros_message_for_strings) {
  auto message = get_messages_strings()[2];
  auto serialized_data = memory_management_->serialize_message(message);
  auto serialized_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  serialized_message->serialized_data = serialized_data;
  serialized_message->topic_name = topic_name_;
  serialized_message->time_stamp = 1;

  auto ros_message = make_shared_ros_message();
  test_msgs::msg::Strings string_test_msg;
  ros_message->message = &string_test_msg;
  std::shared_ptr<rcpputils::SharedLibrary> library;
  auto type_support =
    rosbag2_cpp::get_typesupport("test_msgs/Strings", "rosidl_typesupport_cpp", library);

  converter_->deserialize(serialized_message, type_support, ros_message);

  auto cast_message = static_cast<test_msgs::msg::Strings *>(ros_message->message);
  EXPECT_THAT(*cast_message, Eq(*message));
  EXPECT_THAT(ros_message->time_stamp, Eq(serialized_message->time_stamp));
  EXPECT_THAT(ros_message->topic_name, StrEq(serialized_message->topic_name));
}

TEST_F(CdrConverterTestFixture, serialize_converts_ros_message_into_cdr_for_strings) {
  auto ros_message = make_shared_ros_message(topic_name_);
  ros_message->time_stamp = 1;
  auto message = get_messages_strings()[2];
  ros_message->message = message.get();

  auto serialized_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  serialized_message->serialized_data = memory_management_->make_initialized_message();
  std::shared_ptr<rcpputils::SharedLibrary> library;
  auto type_support =
    rosbag2_cpp::get_typesupport("test_msgs/Strings", "rosidl_typesupport_cpp", library);

  converter_->serialize(ros_message, type_support, serialized_message);

  auto deserialized_msg = memory_management_->deserialize_message<test_msgs::msg::Strings>(
    serialized_message->serialized_data);
  EXPECT_THAT(*deserialized_msg, Eq(*message));
  EXPECT_THAT(serialized_message->topic_name, StrEq(topic_name_));
  EXPECT_THAT(serialized_message->time_stamp, Eq(ros_message->time_stamp));
}

TEST_F(CdrConverterTestFixture, deserialize_converts_cdr_into_ros_message_for_wstrings) {
  auto message = get_messages_wstrings()[0];
  auto serialized_data = memory_management_->serialize_message(message);
  auto serialized_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  serialized_message->serialized_data = serialized_data;
  serialized_message->topic_name = topic_name_;
  serialized_message->time_stamp = 1;

  auto ros_message = make_shared_ros_message();
  test_msgs::msg::WStrings wstring_test_msg;
  ros_message->message = &wstring_test_msg;
  std::shared_ptr<rcpputils::SharedLibrary> library;
  auto type_support =
    rosbag2_cpp::get_typesupport("test_msgs/WStrings", "rosidl_typesupport_cpp", library);

  converter_->deserialize(serialized_message, type_support, ros_message);

  auto cast_message = static_cast<test_msgs::msg::WStrings *>(ros_message->message);
  EXPECT_THAT(*cast_message, Eq(*message));
  EXPECT_THAT(ros_message->time_stamp, Eq(serialized_message->time_stamp));
  EXPECT_THAT(ros_message->topic_name, StrEq(serialized_message->topic_name));
}

TEST_F(CdrConverterTestFixture, serialize_converts_ros_message_into_cdr_for_wstrings) {
  auto ros_message = make_shared_ros_message(topic_name_);
  ros_message->time_stamp = 1;
  auto message = get_messages_wstrings()[0];
  ros_message->message = message.get();

  auto serialized_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  serialized_message->serialized_data = memory_management_->make_initialized_message();
  std::shared_ptr<rcpputils::SharedLibrary> library;
  auto type_support =
    rosbag2_cpp::get_typesupport("test_msgs/WStrings", "rosidl_typesupport_cpp", library);

  converter_->serialize(ros_message, type_support, serialized_message);

  auto deserialized_msg = memory_management_->deserialize_message<test_msgs::msg::WStrings>(
    serialized_message->serialized_data);
  EXPECT_THAT(*deserialized_msg, Eq(*message));
  EXPECT_THAT(serialized_message->topic_name, StrEq(topic_name_));
  EXPECT_THAT(serialized_message->time_stamp, Eq(ros_message->time_stamp));
}

TEST_F(CdrConverterTestFixture, deserialize_converts_cdr_into_ros_message_for_static_array) {
  auto message = get_messages_arrays()[0];
  auto serialized_data = memory_management_->serialize_message(message);
  auto serialized_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  serialized_message->serialized_data = serialized_data;
  serialized_message->topic_name = topic_name_;
  serialized_message->time_stamp = 1;

  auto ros_message = make_shared_ros_message();
  test_msgs::msg::Arrays primitive_test_msg;
  ros_message->message = &primitive_test_msg;
  std::shared_ptr<rcpputils::SharedLibrary> library;
  auto type_support =
    rosbag2_cpp::get_typesupport("test_msgs/Arrays", "rosidl_typesupport_cpp", library);

  converter_->deserialize(serialized_message, type_support, ros_message);

  auto cast_message = static_cast<test_msgs::msg::Arrays *>(ros_message->message);
  EXPECT_THAT(*cast_message, Eq(*message));
  EXPECT_THAT(ros_message->time_stamp, Eq(serialized_message->time_stamp));
  EXPECT_THAT(ros_message->topic_name, StrEq(serialized_message->topic_name));
}

TEST_F(CdrConverterTestFixture, serialize_converts_ros_message_into_cdr_for_static_array) {
  auto ros_message = make_shared_ros_message(topic_name_);
  ros_message->time_stamp = 1;
  auto message = get_messages_arrays()[0];
  ros_message->message = message.get();

  auto serialized_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  serialized_message->serialized_data = memory_management_->make_initialized_message();
  std::shared_ptr<rcpputils::SharedLibrary> library;
  auto type_support =
    rosbag2_cpp::get_typesupport("test_msgs/Arrays", "rosidl_typesupport_cpp", library);

  converter_->serialize(ros_message, type_support, serialized_message);

  auto deserialized_msg = memory_management_->
    deserialize_message<test_msgs::msg::Arrays>(serialized_message->serialized_data);
  EXPECT_THAT(*deserialized_msg, Eq(*message));
  EXPECT_THAT(serialized_message->topic_name, StrEq(topic_name_));
  EXPECT_THAT(serialized_message->time_stamp, Eq(ros_message->time_stamp));
}

TEST_F(CdrConverterTestFixture, deserialize_converts_cdr_into_ros_message_for_unbounded_sequence) {
  auto message = get_messages_unbounded_sequences()[1];
  auto serialized_data = memory_management_->serialize_message(message);
  auto serialized_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  serialized_message->serialized_data = serialized_data;
  serialized_message->topic_name = topic_name_;
  serialized_message->time_stamp = 1;

  auto ros_message = make_shared_ros_message();
  test_msgs::msg::UnboundedSequences dynamic_nested_message;
  ros_message->message = &dynamic_nested_message;
  std::shared_ptr<rcpputils::SharedLibrary> library;
  auto type_support =
    rosbag2_cpp::get_typesupport("test_msgs/UnboundedSequences", "rosidl_typesupport_cpp", library);

  converter_->deserialize(serialized_message, type_support, ros_message);

  auto cast_message = static_cast<test_msgs::msg::UnboundedSequences *>(ros_message->message);
  EXPECT_THAT(*cast_message, Eq(*message));
  EXPECT_THAT(ros_message->time_stamp, Eq(serialized_message->time_stamp));
  EXPECT_THAT(ros_message->topic_name, StrEq(serialized_message->topic_name));
}

TEST_F(CdrConverterTestFixture, serialize_converts_ros_message_into_cdr_for_unbounded_sequence) {
  auto ros_message = make_shared_ros_message(topic_name_);
  ros_message->time_stamp = 1;
  auto message = get_messages_unbounded_sequences()[1];
  test_msgs::msg::BasicTypes first_primitive_message;
  ros_message->message = message.get();

  auto serialized_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  serialized_message->serialized_data = memory_management_->make_initialized_message();
  std::shared_ptr<rcpputils::SharedLibrary> library;
  auto type_support =
    rosbag2_cpp::get_typesupport("test_msgs/UnboundedSequences", "rosidl_typesupport_cpp", library);

  converter_->serialize(ros_message, type_support, serialized_message);

  auto deserialized_msg = memory_management_->
    deserialize_message<test_msgs::msg::UnboundedSequences>(serialized_message->serialized_data);
  EXPECT_THAT(*deserialized_msg, Eq(*message));
  EXPECT_THAT(serialized_message->topic_name, StrEq(topic_name_));
  EXPECT_THAT(serialized_message->time_stamp, Eq(ros_message->time_stamp));
}

TEST_F(CdrConverterTestFixture, deserialize_converts_cdr_into_ros_message_for_multi_nested) {
  auto message = get_messages_multi_nested()[0];
  auto serialized_data = memory_management_->serialize_message(message);
  auto serialized_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  serialized_message->serialized_data = serialized_data;
  serialized_message->topic_name = topic_name_;
  serialized_message->time_stamp = 1;

  auto ros_message = make_shared_ros_message();
  test_msgs::msg::MultiNested multi_nested_message;
  ros_message->message = &multi_nested_message;
  std::shared_ptr<rcpputils::SharedLibrary> library;
  auto type_support =
    rosbag2_cpp::get_typesupport("test_msgs/MultiNested", "rosidl_typesupport_cpp", library);

  converter_->deserialize(serialized_message, type_support, ros_message);

  auto cast_message = static_cast<test_msgs::msg::MultiNested *>(ros_message->message);
  EXPECT_THAT(*cast_message, Eq(*message));
  EXPECT_THAT(ros_message->time_stamp, Eq(serialized_message->time_stamp));
  EXPECT_THAT(ros_message->topic_name, StrEq(serialized_message->topic_name));
}

TEST_F(CdrConverterTestFixture, serialize_converts_ros_message_into_cdr_for_multi_nested) {
  auto ros_message = make_shared_ros_message(topic_name_);
  ros_message->time_stamp = 1;
  auto message = get_messages_multi_nested()[0];
  ros_message->message = message.get();

  auto serialized_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  serialized_message->serialized_data = memory_management_->make_initialized_message();
  std::shared_ptr<rcpputils::SharedLibrary> library;
  auto type_support =
    rosbag2_cpp::get_typesupport("test_msgs/MultiNested", "rosidl_typesupport_cpp", library);

  converter_->serialize(ros_message, type_support, serialized_message);

  auto deserialized_msg = memory_management_->
    deserialize_message<test_msgs::msg::MultiNested>(serialized_message->serialized_data);
  EXPECT_THAT(*deserialized_msg, Eq(*message));
  EXPECT_THAT(serialized_message->topic_name, StrEq(topic_name_));
  EXPECT_THAT(serialized_message->time_stamp, Eq(ros_message->time_stamp));
}
