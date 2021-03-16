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
#include "test_msgs/message_fixtures.hpp"
#include "rosbag2_transport_test_fixture.hpp"

using namespace ::testing;  // NOLINT
using namespace rosbag2_transport;  // NOLINT
using namespace std::chrono_literals;  // NOLINT

class Rosbag2PlayTestTimingFixture : public Rosbag2TransportTestFixture
{
public:
  Rosbag2PlayTestTimingFixture()
  : Rosbag2TransportTestFixture()
  {
    rclcpp::init(0, nullptr);

    auto primitive_message1 = get_messages_strings()[0];
    primitive_message1->string_value = "Hello World 1";
    auto primitive_message2 = get_messages_strings()[0];
    primitive_message2->string_value = "Hello World 2";

    auto topics_and_types =
      std::vector<rosbag2_storage::TopicMetadata>{{"topic1", "test_msgs/Strings", "", ""}};
    std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages =
    {serialize_test_message("topic1", 0, primitive_message1),
      serialize_test_message("topic1", 0, primitive_message2),
      serialize_test_message("topic1", 0, primitive_message2)};

    messages[0]->time_stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    messages[1]->time_stamp = messages[0]->time_stamp + message_time_difference_.count();
    messages[2]->time_stamp = messages[1]->time_stamp + message_time_difference_.count();

    auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
    prepared_mock_reader->prepare(messages, topics_and_types);
    reader_ = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));
  }

  ~Rosbag2PlayTestTimingFixture() override
  {
    rclcpp::shutdown();
  }

  const std::chrono::nanoseconds message_time_difference_ = 250ms;
  const std::chrono::nanoseconds original_playback_time_ = 2 * message_time_difference_;
  //  2 corresponds to the number of messages - 1

  // There are additional delay after playback in Player::play(..) method which we need to take
  // into account.
  const std::chrono::nanoseconds delay_after_playback_ = 50ms;
};

TEST_F(Rosbag2PlayTestTimingFixture, playing_respects_relative_timing_of_stored_messages)
{
  // We can only assert indirectly that the relative times are respected when playing a bag. So
  // we check that time elapsed during playing is at least the time difference between the two
  // messages
  Rosbag2Transport rosbag2_transport(reader_, writer_);
  auto start = std::chrono::steady_clock::now();
  rosbag2_transport.play(storage_options_, play_options_);
  auto replay_time = std::chrono::steady_clock::now() - start;

  ASSERT_THAT(replay_time, Gt(original_playback_time_ + delay_after_playback_));
}

TEST_F(Rosbag2PlayTestTimingFixture, playing_respects_rate)
{
  // Play at 2x speed
  play_options_.rate = 2.0;
  Rosbag2Transport rosbag2_transport(reader_, writer_);
  auto start = std::chrono::steady_clock::now();
  rosbag2_transport.play(storage_options_, play_options_);
  auto replay_time = std::chrono::steady_clock::now() - start;

  EXPECT_THAT(replay_time, Gt(0.5 * original_playback_time_ + delay_after_playback_));
  EXPECT_THAT(replay_time, Lt(original_playback_time_ + delay_after_playback_));

  // Play at 1x speed
  play_options_.rate = 1.0;
  rosbag2_transport = Rosbag2Transport(reader_, writer_);
  start = std::chrono::steady_clock::now();
  rosbag2_transport.play(storage_options_, play_options_);
  replay_time = std::chrono::steady_clock::now() - start;

  EXPECT_THAT(replay_time, Gt(original_playback_time_ + delay_after_playback_));

  // Play at half speed
  play_options_.rate = 0.5;
  rosbag2_transport = Rosbag2Transport(reader_, writer_);
  start = std::chrono::steady_clock::now();
  rosbag2_transport.play(storage_options_, play_options_);
  replay_time = std::chrono::steady_clock::now() - start;

  EXPECT_THAT(replay_time, Gt(2 * original_playback_time_ + delay_after_playback_));

  // Invalid value should result in playing at default rate 1.0
  play_options_.rate = 0.0;
  rosbag2_transport = Rosbag2Transport(reader_, writer_);
  start = std::chrono::steady_clock::now();
  rosbag2_transport.play(storage_options_, play_options_);
  replay_time = std::chrono::steady_clock::now() - start;

  EXPECT_THAT(replay_time, Gt(original_playback_time_ + delay_after_playback_));

  // Invalid value should result in playing at default rate 1.0
  play_options_.rate = -1.23f;
  rosbag2_transport = Rosbag2Transport(reader_, writer_);
  start = std::chrono::steady_clock::now();
  rosbag2_transport.play(storage_options_, play_options_);
  replay_time = std::chrono::steady_clock::now() - start;

  EXPECT_THAT(replay_time, Gt(original_playback_time_ + delay_after_playback_));
}
