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

#include "rosbag2_record_integration_fixture.hpp"
#include "test_memory_management.hpp"

// TODO(Martin-Idel-SI): merge with other write and read tests once signal handling is sorted out
TEST_F(RosBag2RecordIntegrationTestFixture, published_messages_from_multiple_topics_are_recorded)
{
  std::string int_topic = "/int_topic";
  auto serialized_int_bag_message = serialize_message<std_msgs::msg::UInt8>(int_topic, 10);

  std::string string_topic = "/string_topic";
  auto serialized_string_bag_message = serialize_message<std_msgs::msg::String>(
    string_topic, "test_message");

  start_publishing(serialized_string_bag_message, string_topic, 2);
  start_publishing(serialized_int_bag_message, int_topic, 2);

  start_recording({string_topic, int_topic});
  wait_for_publishers_to_stop();
  stop_recording();

  auto recorded_messages = get_messages(database_name_);

  ASSERT_THAT(recorded_messages, SizeIs(4));
  auto string_messages = filter_messages<std_msgs::msg::String>(recorded_messages, string_topic);
  auto int_messages = filter_messages<std_msgs::msg::UInt8>(recorded_messages, int_topic);
  ASSERT_THAT(string_messages, SizeIs(2));
  ASSERT_THAT(int_messages, SizeIs(2));
  EXPECT_THAT(string_messages[0]->data, Eq("test_message"));
  EXPECT_THAT(int_messages[0]->data, Eq(10));
}
