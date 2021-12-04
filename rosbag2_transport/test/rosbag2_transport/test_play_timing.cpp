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
#include <cstdint>
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

    auto primitive_message1 = get_messages_strings()[0];
    primitive_message1->string_value = "Hello World 1";

    auto primitive_message2 = get_messages_strings()[0];
    primitive_message2->string_value = "Hello World 2";

    topics_and_types = {{"topic1", "test_msgs/Strings", "", ""}};
    messages = {
      serialize_test_message("topic1", 0, primitive_message1),
      serialize_test_message("topic1", 0, primitive_message2)
    };

    messages[0]->time_stamp = 100;
    messages[1]->time_stamp = messages[0]->time_stamp + message_time_difference.nanoseconds();

    prepared_mock_reader = std::make_unique<MockSequentialReader>();
    prepared_mock_reader->prepare(messages, topics_and_types);
    reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }

  rclcpp::Clock clock{RCL_STEADY_TIME};
  std::vector<rosbag2_storage::TopicMetadata> topics_and_types;
  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages;
  rclcpp::Duration message_time_difference = {1, 0};
  std::unique_ptr<MockSequentialReader> prepared_mock_reader;
  std::unique_ptr<rosbag2_cpp::Reader> reader;
};

TEST_F(PlayerTestFixture, playing_respects_relative_timing_of_stored_messages)
{
  auto player = std::make_shared<rosbag2_transport::Player>(
    std::move(reader), storage_options_, play_options_);

  // We can only assert indirectly that the relative times are respected when playing a bag. So
  // we check that time elapsed during playing is at least the time difference between the two
  // messages
  auto start = clock.now();
  player->play();
  auto replay_time = clock.now() - start;
  ASSERT_THAT(replay_time, Gt(message_time_difference));
}

TEST_F(PlayerTestFixture, playing_rate_2x)
{
  play_options_.rate = 2.0;
  auto player = std::make_shared<rosbag2_transport::Player>(
    std::move(reader), storage_options_, play_options_);

  auto start = clock.now();
  player->play();
  auto replay_time = clock.now() - start;

  ASSERT_THAT(replay_time, Gt(message_time_difference * 0.5));
  ASSERT_THAT(replay_time, Lt(message_time_difference));
}

TEST_F(PlayerTestFixture, playing_rate_1x)
{
  play_options_.rate = 1.0;
  auto player = std::make_shared<rosbag2_transport::Player>(
    std::move(reader), storage_options_, play_options_);

  auto start = clock.now();
  player->play();
  auto replay_time = clock.now() - start;
  ASSERT_THAT(replay_time, Gt(message_time_difference));
}

TEST_F(PlayerTestFixture, playing_rate_halfx)
{
  play_options_.rate = 0.5;
  auto player = std::make_shared<rosbag2_transport::Player>(
    std::move(reader), storage_options_, play_options_);

  auto start = clock.now();
  player->play();
  auto replay_time = clock.now() - start;
  ASSERT_THAT(replay_time, Gt(message_time_difference * 2.0));
}

TEST_F(PlayerTestFixture, playing_rate_zero)
{
  // Invalid value should result in playing at default rate 1.0
  play_options_.rate = 0.0;
  auto player = std::make_shared<rosbag2_transport::Player>(
    std::move(reader), storage_options_, play_options_);

  auto start = clock.now();
  player->play();
  auto replay_time = clock.now() - start;
  ASSERT_THAT(replay_time, Gt(message_time_difference));
}

TEST_F(PlayerTestFixture, playing_rate_negative)
{
  // Invalid value should result in playing at default rate 1.0
  play_options_.rate = -1.23f;
  auto player = std::make_shared<rosbag2_transport::Player>(
    std::move(reader), storage_options_, play_options_);

  auto start = clock.now();
  player->play();
  auto replay_time = clock.now() - start;
  ASSERT_THAT(replay_time, Gt(message_time_difference));
}

TEST_F(PlayerTestFixture, playing_respects_delay)
{
  rclcpp::Duration delay_margin(1, 0);

  // Sleep 5.0 seconds before play
  play_options_.delay = rclcpp::Duration(5, 0);
  auto lower_expected_duration = message_time_difference + play_options_.delay;
  auto upper_expected_duration = message_time_difference + play_options_.delay + delay_margin;
  auto player = std::make_shared<rosbag2_transport::Player>(
    std::move(reader), storage_options_, play_options_);

  auto start = clock.now();
  player->play();
  auto replay_time = clock.now() - start;

  EXPECT_THAT(replay_time, Gt(lower_expected_duration));
  EXPECT_THAT(replay_time, Lt(upper_expected_duration));
}

TEST_F(PlayerTestFixture, play_ignores_invalid_delay)
{
  rclcpp::Duration delay_margin(1, 0);
  // Invalid value should result in playing at default delay 0.0
  play_options_.delay = rclcpp::Duration(-5, 0);
  auto lower_expected_duration = message_time_difference;
  auto upper_expected_duration = message_time_difference + delay_margin;
  auto player = std::make_shared<rosbag2_transport::Player>(
    std::move(reader), storage_options_, play_options_);

  auto start = clock.now();
  player->play();
  auto replay_time = clock.now() - start;

  EXPECT_THAT(replay_time, Gt(lower_expected_duration));
  EXPECT_THAT(replay_time, Lt(upper_expected_duration));
}

TEST_F(PlayerTestFixture, play_respects_starting_time)
{
  rclcpp::Duration start_delay(5, 0);
  rclcpp::Duration delay_margin(1, 0);

  // Start 5 seconds into the bag
  play_options_.starting_time = INT64_C(5);
  auto lower_expected_duration = message_time_difference + start_delay;
  auto upper_expected_duration = message_time_difference + start_delay + delay_margin;
  auto player = std::make_shared<rosbag2_transport::Player>(
    std::move(reader), storage_options_, play_options_);

  auto start = clock.now();
  player->play();
  auto replay_time = clock.now() - start;

  EXPECT_THAT(replay_time, Gt(lower_expected_duration));
  EXPECT_THAT(replay_time, Lt(upper_expected_duration));
}

TEST_F(PlayerTestFixture, play_ignores_invalid_starting_time)
{
  rclcpp::Duration delay_margin(1, 0);

  // Invalid value should result in playing from
  // metadata starting time - which is 0 seconds here
  play_options_.starting_time = INT64_C(-5);
  auto lower_expected_duration = message_time_difference;
  auto upper_expected_duration = message_time_difference + delay_margin;
  auto player = std::make_shared<rosbag2_transport::Player>(
    std::move(reader), storage_options_, play_options_);

  auto start = clock.now();
  player->play();
  auto replay_time = clock.now() - start;

  EXPECT_THAT(replay_time, Gt(lower_expected_duration));
  EXPECT_THAT(replay_time, Lt(upper_expected_duration));
}
