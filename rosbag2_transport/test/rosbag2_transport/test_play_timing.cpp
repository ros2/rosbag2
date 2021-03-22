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
#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include "rosbag2_transport/rosbag2_transport.hpp"

#include "test_msgs/msg/basic_types.hpp"
#include "test_msgs/message_fixtures.hpp"

#include "rosbag2_transport_test_fixture.hpp"

using namespace ::testing;  // NOLINT
using namespace rosbag2_transport;  // NOLINT

TEST_F(Rosbag2TransportTestFixture, playing_respects_relative_timing_of_stored_messages)
{
  rclcpp::init(0, nullptr);
  auto primitive_message1 = get_messages_strings()[0];
  primitive_message1->string_value = "Hello World 1";

  auto primitive_message2 = get_messages_strings()[0];
  primitive_message2->string_value = "Hello World 2";

  auto message_time_difference = std::chrono::seconds(1);
  auto topics_and_types =
    std::vector<rosbag2_storage::TopicMetadata>{{"topic1", "test_msgs/Strings", "", ""}};
  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages =
  {serialize_test_message("topic1", 0, primitive_message1),
    serialize_test_message("topic1", 0, primitive_message2)};

  messages[0]->time_stamp = 100;
  messages[1]->time_stamp =
    messages[0]->time_stamp + std::chrono::nanoseconds(message_time_difference).count();

  auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
  prepared_mock_reader->prepare(messages, topics_and_types);
  reader_ = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));

  // We can only assert indirectly that the relative times are respected when playing a bag. So
  // we check that time elapsed during playing is at least the time difference between the two
  // messages
  auto start = std::chrono::steady_clock::now();
  Rosbag2Transport rosbag2_transport(reader_, writer_);
  rosbag2_transport.play(storage_options_, play_options_);
  auto replay_time = std::chrono::steady_clock::now() - start;

  ASSERT_THAT(replay_time, Gt(message_time_difference));
  rclcpp::shutdown();
}

TEST_F(Rosbag2TransportTestFixture, playing_respects_rate)
{
  rclcpp::init(0, nullptr);
  auto primitive_message1 = get_messages_strings()[0];
  primitive_message1->string_value = "Hello World 1";

  auto primitive_message2 = get_messages_strings()[0];
  primitive_message2->string_value = "Hello World 2";

  auto message_time_difference = std::chrono::seconds(1);
  auto topics_and_types =
    std::vector<rosbag2_storage::TopicMetadata>{{"topic1", "test_msgs/Strings", "", ""}};
  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages =
  {serialize_test_message("topic1", 0, primitive_message1),
    serialize_test_message("topic1", 0, primitive_message2)};

  messages[0]->time_stamp = 100;
  messages[1]->time_stamp =
    messages[0]->time_stamp + std::chrono::nanoseconds(message_time_difference).count();

  // Play at 2x speed
  auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
  prepared_mock_reader->prepare(messages, topics_and_types);
  reader_ = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));

  play_options_.rate = 2.0;
  auto start = std::chrono::steady_clock::now();
  Rosbag2Transport rosbag2_transport(reader_, writer_);
  rosbag2_transport.play(storage_options_, play_options_);
  auto replay_time = std::chrono::steady_clock::now() - start;

  ASSERT_THAT(replay_time, Gt(0.5 * message_time_difference));
  ASSERT_THAT(replay_time, Lt(message_time_difference));

  // Play at 1x speed
  prepared_mock_reader = std::make_unique<MockSequentialReader>();
  prepared_mock_reader->prepare(messages, topics_and_types);
  reader_ = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));

  play_options_.rate = 1.0;
  start = std::chrono::steady_clock::now();
  rosbag2_transport = Rosbag2Transport(reader_, writer_);
  rosbag2_transport.play(storage_options_, play_options_);
  replay_time = std::chrono::steady_clock::now() - start;

  ASSERT_THAT(replay_time, Gt(message_time_difference));

  // Play at half speed
  prepared_mock_reader = std::make_unique<MockSequentialReader>();
  prepared_mock_reader->prepare(messages, topics_and_types);
  reader_ = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));

  play_options_.rate = 0.5;
  start = std::chrono::steady_clock::now();
  rosbag2_transport = Rosbag2Transport(reader_, writer_);
  rosbag2_transport.play(storage_options_, play_options_);
  replay_time = std::chrono::steady_clock::now() - start;

  ASSERT_THAT(replay_time, Gt(2 * message_time_difference));

  // Invalid value should result in playing at default rate 1.0
  prepared_mock_reader = std::make_unique<MockSequentialReader>();
  prepared_mock_reader->prepare(messages, topics_and_types);
  reader_ = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));

  play_options_.rate = 0.0;
  start = std::chrono::steady_clock::now();
  rosbag2_transport = Rosbag2Transport(reader_, writer_);
  rosbag2_transport.play(storage_options_, play_options_);
  replay_time = std::chrono::steady_clock::now() - start;

  ASSERT_THAT(replay_time, Gt(message_time_difference));

  // Invalid value should result in playing at default rate 1.0
  prepared_mock_reader = std::make_unique<MockSequentialReader>();
  prepared_mock_reader->prepare(messages, topics_and_types);
  reader_ = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));

  play_options_.rate = -1.23f;
  start = std::chrono::steady_clock::now();
  rosbag2_transport = Rosbag2Transport(reader_, writer_);
  rosbag2_transport.play(storage_options_, play_options_);
  replay_time = std::chrono::steady_clock::now() - start;

  ASSERT_THAT(replay_time, Gt(message_time_difference));

  rclcpp::shutdown();
}

TEST_F(Rosbag2TransportTestFixture, playing_starts_paused)
{
  rclcpp::init(0, nullptr);
  auto primitive_message1 = get_messages_strings()[0];
  primitive_message1->string_value = "Hello World 1";

  auto primitive_message2 = get_messages_strings()[0];
  primitive_message2->string_value = "Hello World 2";

  auto message_time_difference = std::chrono::seconds(1);
  auto topics_and_types =
    std::vector<rosbag2_storage::TopicMetadata>{{"topic1", "test_msgs/Strings", "", ""}};
  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages =
  {serialize_test_message("topic1", 0, primitive_message1),
    serialize_test_message("topic1", 0, primitive_message2)};

  messages[0]->time_stamp = 100;
  messages[1]->time_stamp =
    messages[0]->time_stamp + std::chrono::nanoseconds(message_time_difference).count();

  // Start unpaused normally
  auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
  prepared_mock_reader->prepare(messages, topics_and_types);
  reader_ = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));

  play_options_.paused = false;

  auto start = std::chrono::steady_clock::now();
  Rosbag2Transport rosbag2_transport(reader_, writer_);
  rosbag2_transport.play(storage_options_, play_options_);
  auto replay_time = std::chrono::steady_clock::now() - start;

  ASSERT_THAT(replay_time, Gt(message_time_difference));

  // Start paused, resume after some time
  prepared_mock_reader = std::make_unique<MockSequentialReader>();
  prepared_mock_reader->prepare(messages, topics_and_types);
  reader_ = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));

  play_options_.paused = true;

  start = std::chrono::steady_clock::now();
  rosbag2_transport = Rosbag2Transport(reader_, writer_);

  auto pause_time = std::chrono::seconds(1);

  // Activate lifecycle node after pausing for some time
  // Pass node by reference so that it is initialized by play() in the test
  // before activate() call in the thread
  std::future<void> future_handle = std::async(
    std::launch::async, [&rosbag2_transport, pause_time]() mutable {
      std::this_thread::sleep_for(pause_time);
      rosbag2_transport.resume();
    });

  // Call play() to initialize the node and wait for activation
  rosbag2_transport.play(storage_options_, play_options_);
  // Time the play after full execution of play()
  replay_time = std::chrono::steady_clock::now() - start;

  ASSERT_THAT(replay_time, Gt(message_time_difference + pause_time));

  // Start unpaused, pause, unpause again
  auto primitive_message3 = get_messages_strings()[0];
  primitive_message3->string_value = "Hello World 3";
  messages.push_back(serialize_test_message("topic1", 0, primitive_message3));
  messages[2]->time_stamp =
    messages[1]->time_stamp + std::chrono::nanoseconds(message_time_difference).count();

  prepared_mock_reader = std::make_unique<MockSequentialReader>();
  prepared_mock_reader->prepare(messages, topics_and_types);
  reader_ = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));

  play_options_.paused = false;

  start = std::chrono::steady_clock::now();
  rosbag2_transport = Rosbag2Transport(reader_, writer_);

  // Pass node by reference so that it is initialized by play().
  future_handle = std::async(
    std::launch::async, [&rosbag2_transport, message_time_difference]() mutable {
      // Wait for node to initialize, pause
      std::this_thread::sleep_for(message_time_difference);
      rosbag2_transport.pause();

      // Wait, resume
      std::this_thread::sleep_for(message_time_difference);
      rosbag2_transport.resume();
    });

  rosbag2_transport.play(storage_options_, play_options_);
  replay_time = std::chrono::steady_clock::now() - start;

  std::cerr << replay_time.count() << std::endl;
  std::cerr << (3 * message_time_difference).count() << std::endl;
  // 1 second for message 0 and 1 in parallel to initialization wait,
  // 1 second for pause, and 1 second for message 2.
  ASSERT_THAT(replay_time, Gt(3 * message_time_difference));

  rclcpp::shutdown();
}
