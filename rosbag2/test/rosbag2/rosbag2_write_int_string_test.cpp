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
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rosbag2/rosbag2.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include "test_memory_management.hpp"
#include "rosbag2_integration_test_fixture.hpp"

using namespace ::testing;  // NOLINT
using namespace rosbag2;  // NOLINT

// TODO(Martin-Idel-SI): merge with other write and read tests once signal handling is sorted out
TEST_F(RosBag2IntegrationTestFixture, published_messages_from_multiple_topics_are_recorded)
{
  std::string int_topic = "/int_topic";
  auto int_message = std::make_shared<std_msgs::msg::UInt8>();
  int_message->data = 10;
  auto serialized_int_bag_message = serialize_message(int_message, int_topic);

  std::string string_topic = "/string_topic";
  auto string_message = std::make_shared<std_msgs::msg::String>();
  string_message->data = "test_message";
  auto serialized_string_bag_message = serialize_message(string_message, string_topic);

  start_publishing(serialized_string_bag_message, string_topic);
  start_publishing(serialized_int_bag_message, int_topic);
  start_recording({string_topic, int_topic});
  stop_recording();

  auto recorded_messages = get_messages(database_name_);

  ASSERT_THAT(recorded_messages, Not(IsEmpty()));
  std::vector<std::shared_ptr<std_msgs::msg::String>> string_messages;
  std::vector<std::shared_ptr<std_msgs::msg::UInt8>> int_messages;
  for (const auto & message : recorded_messages) {
    if (message->topic_name == string_topic) {
      auto deserialized = deserialize_message<std_msgs::msg::String>(message);
      string_messages.push_back(deserialized);
    } else if (message->topic_name == int_topic) {
      auto deserialized = deserialize_message<std_msgs::msg::UInt8>(message);
      int_messages.push_back(deserialized);
    }
  }
  ASSERT_THAT(string_messages, Not(IsEmpty()));
  ASSERT_THAT(int_messages, Not(IsEmpty()));
  EXPECT_THAT(string_messages[0]->data, Eq("test_message"));
  EXPECT_THAT(int_messages[0]->data, Eq(10));
}
