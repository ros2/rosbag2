// Copyright 2021, Apex.AI
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
#include <future>
#include <memory>
#include <utility>
#include <vector>

#include "mock_player.hpp"
#include "rosbag2_play_test_fixture.hpp"
#include "test_msgs/message_fixtures.hpp"
#include "test_msgs/msg/basic_types.hpp"

using namespace ::testing;  // NOLINT
using namespace rosbag2_transport;  // NOLINT
using namespace rosbag2_test_common;  // NOLINT

class RosBag2PlaySeekTestFixture : public RosBag2PlayTestFixture
{
public:
  RosBag2PlaySeekTestFixture()
  : RosBag2PlayTestFixture()
  {
    topic_types_ = std::vector<rosbag2_storage::TopicMetadata>{
      {"topic1", "test_msgs/BasicTypes", "", ""}};

    messages_.reserve(num_msgs_to_publish_);

    auto primitive_message = get_messages_basic_types()[0];
    for (size_t i = 0; i < num_msgs_to_publish_; i++) {
      primitive_message->int32_value = static_cast<int32_t>(i + 1);
      const int64_t timestamp = start_time_ms_ + message_spacing_ms_ * static_cast<int64_t>(i);
      messages_.push_back(serialize_test_message("topic1", timestamp, primitive_message));
    }

    auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
    prepared_mock_reader->prepare(messages_, topic_types_);
    reader_ = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));
    play_options_.read_ahead_queue_size = 2;
  }

  const size_t num_msgs_to_publish_ = 7;
  const int64_t start_time_ms_ = 1000;
  const int64_t message_spacing_ms_ = 100;
  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages_;
  std::vector<rosbag2_storage::TopicMetadata> topic_types_;
  std::unique_ptr<rosbag2_cpp::Reader> reader_;
};

TEST_F(RosBag2PlaySeekTestFixture, seek_back_in_time) {
  auto player = std::make_shared<MockPlayer>(std::move(reader_), storage_options_, play_options_);

  sub_ = std::make_shared<SubscriptionManager>();
  sub_->add_subscription<test_msgs::msg::BasicTypes>("/topic1", messages_.size() + 2);

  // Wait for discovery to match publishers with subscribers
  ASSERT_TRUE(
    sub_->spin_and_wait_for_matched(player->get_list_of_publishers(), std::chrono::seconds(30)));

  auto await_received_messages = sub_->spin_subscriptions();

  player->pause();
  ASSERT_TRUE(player->is_paused());

  auto player_future = std::async(std::launch::async, [&player]() -> void {player->play();});

  EXPECT_TRUE(player->is_paused());
  EXPECT_TRUE(player->play_next());
  EXPECT_TRUE(player->play_next());

  EXPECT_TRUE(player->seek(start_time_ms_ * 1000000 - 1));
  EXPECT_TRUE(player->play_next());
  EXPECT_TRUE(player->play_next());
  player->resume();
  player_future.get();
  await_received_messages.get();

  auto replayed_topic1 = sub_->get_received_messages<test_msgs::msg::BasicTypes>("/topic1");
  ASSERT_THAT(replayed_topic1, SizeIs(num_msgs_to_publish_ + 2));

  EXPECT_EQ(replayed_topic1[0]->int32_value, 1);
  EXPECT_EQ(replayed_topic1[1]->int32_value, 2);

  for (size_t i = 1; i <= num_msgs_to_publish_; i++) {
    EXPECT_EQ(replayed_topic1[i + 1]->int32_value, static_cast<int32_t>(i)) << "i=" << i;
  }
}

TEST_F(RosBag2PlaySeekTestFixture, seek_with_timestamp_later_than_in_last_message) {
  const size_t expected_number_of_messages = num_msgs_to_publish_;
  auto player = std::make_shared<MockPlayer>(std::move(reader_), storage_options_, play_options_);

  sub_ = std::make_shared<SubscriptionManager>();
  sub_->add_subscription<test_msgs::msg::BasicTypes>("/topic1", expected_number_of_messages);

  // Wait for discovery to match publishers with subscribers
  ASSERT_TRUE(
    sub_->spin_and_wait_for_matched(player->get_list_of_publishers(), std::chrono::seconds(30)));

  auto await_received_messages = sub_->spin_subscriptions();

  player->pause();
  ASSERT_TRUE(player->is_paused());

  auto player_future = std::async(std::launch::async, [&player]() -> void {player->play();});

  EXPECT_TRUE(player->is_paused());
  EXPECT_TRUE(player->play_next());

  EXPECT_FALSE(
    player->seek(
      (start_time_ms_ + message_spacing_ms_ * (num_msgs_to_publish_ - 1)) * 1000000 + 1)
  );

  EXPECT_TRUE(player->play_next());
  EXPECT_TRUE(player->play_next());
  player->resume();
  player_future.get();
  await_received_messages.get();

  auto replayed_topic1 = sub_->get_received_messages<test_msgs::msg::BasicTypes>("/topic1");
  ASSERT_THAT(replayed_topic1, SizeIs(expected_number_of_messages));

  for (size_t i = 0; i < replayed_topic1.size(); i++) {
    EXPECT_EQ(replayed_topic1[i]->int32_value, static_cast<int32_t>(i + 1)) << "i=" << i;
  }
}

TEST_F(RosBag2PlaySeekTestFixture, seek_forward) {
  const size_t expected_number_of_messages = num_msgs_to_publish_ - 1;
  auto player = std::make_shared<MockPlayer>(std::move(reader_), storage_options_, play_options_);

  sub_ = std::make_shared<SubscriptionManager>();
  sub_->add_subscription<test_msgs::msg::BasicTypes>("/topic1", expected_number_of_messages);

  // Wait for discovery to match publishers with subscribers
  ASSERT_TRUE(
    sub_->spin_and_wait_for_matched(player->get_list_of_publishers(), std::chrono::seconds(30)));

  auto await_received_messages = sub_->spin_subscriptions();

  player->pause();
  ASSERT_TRUE(player->is_paused());

  auto player_future = std::async(std::launch::async, [&player]() -> void {player->play();});

  EXPECT_TRUE(player->is_paused());
  EXPECT_TRUE(player->play_next());

  EXPECT_TRUE(player->seek((start_time_ms_ + message_spacing_ms_) * 1000000 + 1));
  EXPECT_TRUE(player->play_next());
  player->resume();
  player_future.get();
  await_received_messages.get();

  auto replayed_topic1 = sub_->get_received_messages<test_msgs::msg::BasicTypes>("/topic1");
  EXPECT_THAT(replayed_topic1, SizeIs(expected_number_of_messages));
  EXPECT_EQ(replayed_topic1[0]->int32_value, 1);
  EXPECT_EQ(replayed_topic1[1]->int32_value, 3);

  for (size_t i = 2; i < replayed_topic1.size(); i++) {
    EXPECT_EQ(replayed_topic1[i]->int32_value, static_cast<int32_t>(i + 2)) << "i=" << i;
  }
}
