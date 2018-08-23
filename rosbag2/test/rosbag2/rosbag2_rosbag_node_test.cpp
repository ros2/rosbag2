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

#include <future>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "../../src/rosbag2/rosbag2_node.hpp"
#include "../../src/rosbag2/typesupport_helpers.hpp"

using namespace ::testing;  // NOLINT

class RosBag2NodeFixture : public Test
{
public:
  RosBag2NodeFixture()
  {
    node_ = std::make_shared<rosbag2::Rosbag2Node>("rosbag2");
  }

  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  std::vector<std::string> subscribe_raw_messages(
    size_t expected_messages_number, const std::string & topic_name, const std::string & type)
  {
    std::vector<std::string> messages;
    size_t counter = 0;
    auto subscription = node_->create_raw_subscription(topic_name, type,
        [this, &counter, &messages](std::shared_ptr<rcutils_char_array_t> message) {
          messages.push_back(deserialize_string_message(message));
          counter++;
        });

    while (counter < expected_messages_number) {
      rclcpp::spin_some(node_);
    }
    return messages;
  }

  std::shared_ptr<rcutils_char_array_t> serialize_string_message(std::string message)
  {
    auto test_message = std::make_shared<std_msgs::msg::String>();
    test_message->data = message;

    auto rcutils_allocator = rcutils_get_default_allocator();
    auto initial_capacity = 8u + static_cast<size_t>(test_message->data.size());
    auto msg = new rcutils_char_array_t;
    *msg = rcutils_get_zero_initialized_char_array();
    auto ret = rcutils_char_array_init(msg, initial_capacity, &rcutils_allocator);
    if (ret != RCUTILS_RET_OK) {
      throw std::runtime_error("Error allocating resources for serialized message" +
              std::to_string(ret));
    }

    auto serialized_message = std::shared_ptr<rcutils_char_array_t>(msg,
        [](rcutils_char_array_t * msg) {
          int error = rcutils_char_array_fini(msg);
          delete msg;
          if (error != RCUTILS_RET_OK) {
            RCUTILS_LOG_ERROR_NAMED(
              "rosbag2", "Leaking memory. Error: %s", rcutils_get_error_string_safe());
          }
        });

    serialized_message->buffer_length = initial_capacity;

    auto string_ts = rosbag2::get_typesupport("std_msgs/String");

    auto error = rmw_serialize(test_message.get(), string_ts, serialized_message.get());
    if (error != RMW_RET_OK) {
      throw std::runtime_error("Something went wrong preparing the serialized message");
    }

    return serialized_message;
  }

  std::string deserialize_string_message(std::shared_ptr<rcutils_char_array_t> serialized_message)
  {
    char * copied = new char[serialized_message->buffer_length];
    auto string_length = serialized_message->buffer_length - 8;
    memcpy(copied, &serialized_message->buffer[8], string_length);
    std::string message_content(copied);
    delete[] copied;
    return message_content;
  }

  std::shared_ptr<rosbag2::Rosbag2Node> node_;
};


TEST_F(RosBag2NodeFixture, publisher_and_subscriber_work)
{
  // We currently publish more messages because they can get lost
  std::vector<std::string> test_messages = {"Hello World", "Hello World", "Hello World"};
  std::string topic_name = "string_topic";
  std::string type = "std_msgs/String";

  auto subscriber_future_ = std::async(std::launch::async, [this, topic_name, type] {
        return subscribe_raw_messages(1, topic_name, type);
      });

  auto publisher = node_->create_raw_publisher(topic_name, type);
  for (const auto & message : test_messages) {
    publisher->publish(serialize_string_message(message));
  }

  auto subscribed_messages = subscriber_future_.get();
  EXPECT_THAT(subscribed_messages, SizeIs(Not(0)));
  EXPECT_THAT(subscribed_messages[0], StrEq("Hello World"));
}
