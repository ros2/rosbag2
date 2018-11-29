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

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rosbag2_transport/rosbag2_transport.hpp"
#include "rosbag2_transport_test_fixture.hpp"
#include "test_msgs/msg/primitives.hpp"
#include "test_msgs/message_fixtures.hpp"

using namespace ::testing;  // NOLINT
using namespace rosbag2_transport;  // NOLINT

TEST_F(Rosbag2TransportTestFixture, playing_respects_relative_timing_of_stored_messages)
{
  rclcpp::init(0, nullptr);
  auto primitive_message1 = get_messages_primitives()[0];
  primitive_message1->string_value = "Hello World 1";

  auto primitive_message2 = get_messages_primitives()[0];
  primitive_message2->string_value = "Hello World 2";

  auto message_time_difference = std::chrono::seconds(1);
  auto topics_and_types =
    std::vector<rosbag2::TopicMetadata>{{"topic1", "test_msgs/Primitives", ""}};
  std::vector<std::shared_ptr<rosbag2::SerializedBagMessage>> messages =
  {serialize_test_message("topic1", 0, primitive_message1),
    serialize_test_message("topic1", 0, primitive_message2)};

  messages[0]->time_stamp = 100;
  messages[1]->time_stamp =
    messages[0]->time_stamp + std::chrono::nanoseconds(message_time_difference).count();

  reader_->prepare(messages, topics_and_types);

  // We can only assert indirectly that the relative times are respected when playing a bag. So
  // we check that time elapsed during playing is at least the time difference between the two
  // messages
  auto start = std::chrono::steady_clock::now();
  Rosbag2Transport rosbag2_transport(reader_, writer_, info_);
  rosbag2_transport.play(storage_options_, play_options_);
  auto replay_time = std::chrono::steady_clock::now() - start;

  ASSERT_THAT(replay_time, Gt(message_time_difference));
  rclcpp::shutdown();
}
