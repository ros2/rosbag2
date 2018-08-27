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
  auto message = std::make_shared<std_msgs::msg::UInt8>();
  message->data = 10;
  auto serialized_bag_message = serialize_message(message, "/string_topic");

  start_publishing(serialized_bag_message, "string_topic");
  start_recording({"/string_topic"});
  stop_recording();

  auto recorded_messages = get_messages(database_name_);

  ASSERT_THAT(recorded_messages, Not(IsEmpty()));
  auto deserialized = deserialize_message<std_msgs::msg::UInt8>(recorded_messages[0]);
  EXPECT_THAT(deserialized->data, Eq(10));
}
