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

#include "rosbag2_transport/player.hpp"

#include "test_msgs/msg/basic_types.hpp"
#include "test_msgs/message_fixtures.hpp"

#include "rosbag2_transport_test_fixture.hpp"

using namespace ::testing;  // NOLINT
using namespace rosbag2_transport;  // NOLINT

class PlayerTestFixture : public Rosbag2TransportTestFixture
{
public:
  rosbag2_storage::StorageOptions storage_options_;
  rosbag2_transport::PlayOptions play_options_;

protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }
};

TEST_F(PlayerTestFixture, playing_respects_relative_timing_of_stored_messages)
{
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
  auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));

  // We can only assert indirectly that the relative times are respected when playing a bag. So
  // we check that time elapsed during playing is at least the time difference between the two
  // messages
  auto start = std::chrono::steady_clock::now();
  auto player = std::make_shared<rosbag2_transport::Player>(
    std::move(reader), storage_options_, play_options_);
  player->play();
  auto replay_time = std::chrono::steady_clock::now() - start;

  ASSERT_THAT(replay_time, Gt(message_time_difference));
}

TEST_F(PlayerTestFixture, playing_respects_rate)
{
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
  {
    play_options_.rate = 2.0;

    auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
    prepared_mock_reader->prepare(messages, topics_and_types);
    auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));
    auto player = std::make_shared<rosbag2_transport::Player>(
      std::move(reader), storage_options_, play_options_);
    auto start = std::chrono::steady_clock::now();
    player->play();
    auto replay_time = std::chrono::steady_clock::now() - start;

    ASSERT_THAT(replay_time, Gt(0.5 * message_time_difference));
    ASSERT_THAT(replay_time, Lt(message_time_difference));
  }

  // Play at 1x speed
  {
    play_options_.rate = 1.0;

    auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
    prepared_mock_reader->prepare(messages, topics_and_types);
    auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));
    auto player = std::make_shared<rosbag2_transport::Player>(
      std::move(reader), storage_options_, play_options_);
    auto start = std::chrono::steady_clock::now();
    player->play();
    auto replay_time = std::chrono::steady_clock::now() - start;

    ASSERT_THAT(replay_time, Gt(message_time_difference));
  }

  // Play at half speed
  {
    play_options_.rate = 0.5;

    auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
    prepared_mock_reader->prepare(messages, topics_and_types);
    auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));
    auto player = std::make_shared<rosbag2_transport::Player>(
      std::move(reader), storage_options_, play_options_);
    auto start = std::chrono::steady_clock::now();
    player->play();
    auto replay_time = std::chrono::steady_clock::now() - start;

    ASSERT_THAT(replay_time, Gt(2 * message_time_difference));
  }

  // Invalid value should result in playing at default rate 1.0
  {
    play_options_.rate = 0.0;

    auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
    prepared_mock_reader->prepare(messages, topics_and_types);
    auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));
    auto player = std::make_shared<rosbag2_transport::Player>(
      std::move(reader), storage_options_, play_options_);
    auto start = std::chrono::steady_clock::now();
    player->play();
    auto replay_time = std::chrono::steady_clock::now() - start;

    ASSERT_THAT(replay_time, Gt(message_time_difference));
  }

  // Invalid value should result in playing at default rate 1.0
  {
    play_options_.rate = -1.23f;

    auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
    prepared_mock_reader->prepare(messages, topics_and_types);
    auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));
    auto player = std::make_shared<rosbag2_transport::Player>(
      std::move(reader), storage_options_, play_options_);
    auto start = std::chrono::steady_clock::now();
    player->play();
    auto replay_time = std::chrono::steady_clock::now() - start;

    ASSERT_THAT(replay_time, Gt(message_time_difference));
  }
}

TEST_F(PlayerTestFixture, playing_respects_delay)
{
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

  float delay_margin = 1.0;

  // Sleep 5.0 seconds before play
  {
    play_options_.delay = 5.0;
    std::chrono::duration<float> delay(play_options_.delay);
    std::chrono::duration<float> delay_uppper(play_options_.delay + delay_margin);

    auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
    prepared_mock_reader->prepare(messages, topics_and_types);
    auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));
    auto player = std::make_shared<rosbag2_transport::Player>(
      std::move(reader), storage_options_, play_options_);
    auto start = std::chrono::steady_clock::now();
    player->play();
    auto replay_time = std::chrono::steady_clock::now() - start;

    ASSERT_THAT(replay_time, Gt(message_time_difference + delay));
    ASSERT_THAT(replay_time, Lt(message_time_difference + delay_uppper));
  }

  // Invalid value should result in playing at default delay 0.0
  {
    play_options_.delay = -5.0;
    std::chrono::duration<float> delay_uppper(delay_margin);

    auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
    prepared_mock_reader->prepare(messages, topics_and_types);
    auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));
    auto player = std::make_shared<rosbag2_transport::Player>(
      std::move(reader), storage_options_, play_options_);
    auto start = std::chrono::steady_clock::now();
    player->play();
    auto replay_time = std::chrono::steady_clock::now() - start;

    ASSERT_THAT(replay_time, Gt(message_time_difference));
    ASSERT_THAT(replay_time, Lt(message_time_difference + delay_uppper));
  }
}
