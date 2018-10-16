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
#include "rosbag2/format_converter_interface.hpp"
#include "rosbag2/typesupport_helpers.hpp"
#include "rosbag2_test_common/memory_management.hpp"
#include "test_msgs/message_fixtures.hpp"

using rosbag2::FormatConverterInterface;
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

  std::shared_ptr<rosbag2::Ros2Message> make_shared_ros_message(const std::string & topic_name = "")
  {
    auto raw_msg = new rosbag2::Ros2Message;
    auto size = topic_name.empty() ? 0 : strlen(topic_name.c_str()) + 1;
    auto ros_message = std::shared_ptr<rosbag2::Ros2Message>(raw_msg,
        [](rosbag2::Ros2Message * msg) {
          int error = rcutils_char_array_fini(&msg->topic_name);
          delete msg;
          if (error != RCUTILS_RET_OK) {
            ROSBAG2_CONVERTER_DEFAULT_PLUGINS_LOG_ERROR(
              "rosbag2_converter_default_plugins", "Leaking memory %i", error);
          }
        });
    ros_message->topic_name = rcutils_get_zero_initialized_char_array();
    auto ret = rcutils_char_array_init(&ros_message->topic_name, size, &allocator_);
    if (ret != RCUTILS_RET_OK) {
      ROSBAG2_CONVERTER_DEFAULT_PLUGINS_LOG_ERROR(
        "rosbag2_converter_default_plugins", "Initialization of char array failed %i", ret);
    }
    ros_message->timestamp = 0;
    ros_message->message = nullptr;
    ros_message->allocator = allocator_;

    if (!topic_name_.empty()) {
      memcpy(ros_message->topic_name.buffer, topic_name.c_str(), size);
    }

    return ros_message;
  }

  std::unique_ptr<rosbag2::FormatConverterInterface> converter_;
  std::unique_ptr<MemoryManagement> memory_management_;
  std::string topic_name_;
  rcutils_allocator_t allocator_;
};

TEST_F(CdrConverterTestFixture, deserialize_converts_cdr_into_ros_message) {
  auto message = get_messages_primitives()[0];
  message->string_value = "test_deserialize";
  auto serialized_data = memory_management_->serialize_message(message);
  auto serialized_message = std::make_shared<SerializedBagMessage>();
  serialized_message->serialized_data = serialized_data;
  serialized_message->topic_name = topic_name_;
  serialized_message->time_stamp = 1;

  auto ros_message = make_shared_ros_message();
  test_msgs::msg::Primitives primitive_test_msg;
  ros_message->message = &primitive_test_msg;
  auto type_support = rosbag2::get_typesupport("test_msgs/Primitives");

  converter_->deserialize(ros_message, serialized_message, type_support);

  auto cast_message = static_cast<test_msgs::msg::Primitives *>(ros_message->message);
  EXPECT_THAT(cast_message->string_value, StrEq("test_deserialize"));
  EXPECT_THAT(ros_message->timestamp, Eq(serialized_message->time_stamp));
  EXPECT_THAT(ros_message->topic_name.buffer, StrEq(serialized_message->topic_name));
}

TEST_F(CdrConverterTestFixture, serialize_converts_ros_message_into_cdr) {
  auto ros_message = make_shared_ros_message(topic_name_);
  ros_message->timestamp = 1;
  auto message = get_messages_primitives()[0];
  message->string_value = "test_serialize";
  ros_message->message = message.get();

  auto serialized_message = std::make_shared<SerializedBagMessage>();
  serialized_message->serialized_data = memory_management_->make_initialized_message();
  auto type_support = rosbag2::get_typesupport("test_msgs/Primitives");

  converter_->serialize(serialized_message, ros_message, type_support);

  auto deserialized_msg = memory_management_->deserialize_message<test_msgs::msg::Primitives>(
    serialized_message->serialized_data);
  EXPECT_THAT(deserialized_msg->string_value, StrEq("test_serialize"));
  EXPECT_THAT(serialized_message->topic_name, StrEq(topic_name_));
  EXPECT_THAT(serialized_message->time_stamp, Eq(ros_message->timestamp));
}
