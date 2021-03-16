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
#include "rosbag2_transport/player.hpp"

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
    auto primitive_message3 = get_messages_strings()[0];
    primitive_message2->string_value = "Hello World 3";

    auto topics_and_types =
      std::vector<rosbag2_storage::TopicMetadata>{{"topic1", "test_msgs/Strings", "", ""}};
    std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages =
    {serialize_test_message("topic1", 0, primitive_message1),
      serialize_test_message("topic1", 0, primitive_message2),
      serialize_test_message("topic1", 0, primitive_message3)};

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
  const std::chrono::nanoseconds original_playback_duartion_ = 2 * message_time_difference_;
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
  auto replay_duration = std::chrono::steady_clock::now() - start;

  ASSERT_THAT(replay_duration, Gt(original_playback_duartion_ + delay_after_playback_));
}

TEST_F(Rosbag2PlayTestTimingFixture, playing_respects_rate)
{
  // Play at 2x speed
  play_options_.rate = 2.0;
  Rosbag2Transport rosbag2_transport(reader_, writer_);
  auto start = std::chrono::steady_clock::now();
  rosbag2_transport.play(storage_options_, play_options_);
  auto replay_duration = std::chrono::steady_clock::now() - start;

  auto expected_lower_bound_time = std::chrono::duration_cast<std::chrono::nanoseconds>(
    (1 / play_options_.rate) * original_playback_duartion_) + delay_after_playback_;

  EXPECT_THAT(replay_duration, Gt(expected_lower_bound_time));
  EXPECT_THAT(replay_duration, Lt(original_playback_duartion_ + delay_after_playback_));

  // Play at 1x speed
  play_options_.rate = 1.0;
  rosbag2_transport = Rosbag2Transport(reader_, writer_);
  start = std::chrono::steady_clock::now();
  rosbag2_transport.play(storage_options_, play_options_);
  replay_duration = std::chrono::steady_clock::now() - start;

  EXPECT_THAT(replay_duration, Gt(original_playback_duartion_ + delay_after_playback_));

  // Play at half speed
  play_options_.rate = 0.5;
  rosbag2_transport = Rosbag2Transport(reader_, writer_);
  start = std::chrono::steady_clock::now();
  rosbag2_transport.play(storage_options_, play_options_);
  replay_duration = std::chrono::steady_clock::now() - start;

  EXPECT_THAT(replay_duration, Gt(2 * original_playback_duartion_ + delay_after_playback_));

  // Invalid value should result in playing at default rate 1.0
  play_options_.rate = 0.0;
  rosbag2_transport = Rosbag2Transport(reader_, writer_);
  start = std::chrono::steady_clock::now();
  rosbag2_transport.play(storage_options_, play_options_);
  replay_duration = std::chrono::steady_clock::now() - start;

  EXPECT_THAT(replay_duration, Gt(original_playback_duartion_ + delay_after_playback_));

  // Invalid value should result in playing at default rate 1.0
  play_options_.rate = -1.23f;
  rosbag2_transport = Rosbag2Transport(reader_, writer_);
  start = std::chrono::steady_clock::now();
  rosbag2_transport.play(storage_options_, play_options_);
  replay_duration = std::chrono::steady_clock::now() - start;

  EXPECT_THAT(replay_duration, Gt(original_playback_duartion_ + delay_after_playback_));
}

TEST_F(Rosbag2PlayTestTimingFixture, play_pause_resume_respect_play_time) {
  reader_->open(storage_options_, {"", rmw_get_serialization_format()});
  auto transport_node = std::make_shared<Rosbag2Node>(play_options_.node_prefix + "_rosbag2");

  Player player(reader_, transport_node);
  EXPECT_FALSE(player.play_next());

  EXPECT_TRUE(player.pause_resume());  // Put player in pause mode before starting

  EXPECT_FALSE(player.play_next());

  // Run play asynchronously in separate thread
  std::future<std::chrono::nanoseconds> play_future_result =
    std::async(
    std::launch::async, [&]() {
      auto start = std::chrono::steady_clock::now();
      player.play(play_options_);
      return std::chrono::steady_clock::now() - start;
    });

  std::this_thread::sleep_for(35ms);  // Tolerance for lower bound
  auto start_in_pause = std::chrono::steady_clock::now();

  // Play first two messages in pause mode.
  EXPECT_TRUE(player.play_next());
  EXPECT_TRUE(player.play_next());
  // After that, expected playback time without pause is going to be
  // "original_playback_duartion_ / 2". Player shall take in to account messages played in pause
  // mode and adjust playback timeouts for following messages.

  std::this_thread::sleep_for(35ms);

  player.set_playback_rate(2.0f);
  EXPECT_FLOAT_EQ(2.0f, player.get_playback_rate());
  EXPECT_FALSE(player.pause_resume());  // Resume playing with 2x speed

  auto time_elapsed_in_pause = std::chrono::steady_clock::now() - start_in_pause;

  play_future_result.wait();
  auto replay_duration = play_future_result.get();

  EXPECT_FALSE(player.play_next());
  EXPECT_FLOAT_EQ(2.0f, player.get_playback_rate());

  auto expected_lower_bound_time = std::chrono::duration_cast<std::chrono::nanoseconds>(
    (1 / player.get_playback_rate()) * original_playback_duartion_ / 2) +
    time_elapsed_in_pause + delay_after_playback_;

  EXPECT_THAT(replay_duration, Gt(expected_lower_bound_time));
  EXPECT_THAT(replay_duration, Lt(original_playback_duartion_ + delay_after_playback_));

  // Make sure that we can replay one more time with the same player object after resetting reader.
  reader_->open(storage_options_, {"", rmw_get_serialization_format()});
  // Replaying the same messages with 1x speed without pause
  play_options_.rate = 1.0f;
  auto start = std::chrono::steady_clock::now();
  player.play(play_options_);
  replay_duration = std::chrono::steady_clock::now() - start;
  EXPECT_THAT(replay_duration, Gt(original_playback_duartion_ + delay_after_playback_));
}
