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

#include "rosbag2_play_test_fixture.hpp"
#include "rosbag2_transport/player.hpp"
#include "test_msgs/message_fixtures.hpp"
#include "test_msgs/msg/basic_types.hpp"

using namespace ::testing;  // NOLINT
using namespace rosbag2_transport;  // NOLINT
using namespace rosbag2_test_common;  // NOLINT

class MockPlayer : public rosbag2_transport::Player
{
public:
  MockPlayer(
    std::unique_ptr<rosbag2_cpp::Reader> reader,
    const rosbag2_storage::StorageOptions & storage_options,
    const rosbag2_transport::PlayOptions & play_options)
  : Player(std::move(reader), storage_options, play_options)
  {}

  std::vector<rclcpp::PublisherBase *> get_list_of_publishers()
  {
    std::vector<rclcpp::PublisherBase *> pub_list;
    for (const auto & publisher : publishers_) {
      pub_list.push_back(static_cast<rclcpp::PublisherBase *>(publisher.second.get()));
    }
    return pub_list;
  }
};

TEST_F(RosBag2PlayTestFixture, seek_back_in_time) {
  auto topic_types = std::vector<rosbag2_storage::TopicMetadata>{
    {"topic1", "test_msgs/BasicTypes", "", ""}};

  const size_t num_msgs_to_publish = 7;
  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages;
  messages.reserve(num_msgs_to_publish);
  int64_t start_time_ms = 1000;
  int64_t message_spacing_ms = 100;
  auto primitive_message = get_messages_basic_types()[0];
  for (size_t i = 0; i < num_msgs_to_publish; i++) {
    primitive_message->int32_value = static_cast<int32_t>(i);
    const int64_t timestamp = start_time_ms + message_spacing_ms * static_cast<int64_t>(i);
    messages.push_back(serialize_test_message("topic1", timestamp, primitive_message));
  }

  auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
  prepared_mock_reader->prepare(messages, topic_types);
  auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));
  play_options_.read_ahead_queue_size = 3;
  auto player = std::make_shared<MockPlayer>(std::move(reader), storage_options_, play_options_);

  sub_ = std::make_shared<SubscriptionManager>();
  sub_->add_subscription<test_msgs::msg::BasicTypes>("/topic1", messages.size() + 2);

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

  EXPECT_TRUE(player->seek(start_time_ms * 1000000 - 1));
  EXPECT_TRUE(player->play_next());
  EXPECT_TRUE(player->play_next());
  player->resume();
  player_future.get();
  await_received_messages.get();

  auto replayed_topic1 = sub_->get_received_messages<test_msgs::msg::BasicTypes>("/topic1");
  EXPECT_THAT(replayed_topic1, SizeIs(num_msgs_to_publish + 2));
}

TEST_F(RosBag2PlayTestFixture, seek_with_timestamp_later_than_in_last_message) {
  auto topic_types = std::vector<rosbag2_storage::TopicMetadata>{
    {"topic1", "test_msgs/BasicTypes", "", ""}};

  const size_t num_msgs_to_publish = 7;
  const size_t expected_number_of_messages = num_msgs_to_publish;
  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages;
  messages.reserve(num_msgs_to_publish);
  int64_t start_time_ms = 1000;
  int64_t message_spacing_ms = 100;
  auto primitive_message = get_messages_basic_types()[0];
  for (size_t i = 0; i < num_msgs_to_publish; i++) {
    primitive_message->int32_value = static_cast<int32_t>(i + 1);
    const int64_t timestamp = start_time_ms + message_spacing_ms * static_cast<int64_t>(i);
    messages.push_back(serialize_test_message("topic1", timestamp, primitive_message));
  }

  auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
  prepared_mock_reader->prepare(messages, topic_types);
  auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));
  play_options_.read_ahead_queue_size = 3;
  auto player = std::make_shared<MockPlayer>(std::move(reader), storage_options_, play_options_);

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
    player->seek((start_time_ms + message_spacing_ms * (num_msgs_to_publish - 1)) * 1000000 + 1));

  EXPECT_TRUE(player->play_next());
  EXPECT_TRUE(player->play_next());
  player->resume();
  player_future.get();
  await_received_messages.get();

  auto replayed_topic1 = sub_->get_received_messages<test_msgs::msg::BasicTypes>("/topic1");
  ASSERT_THAT(replayed_topic1, SizeIs(expected_number_of_messages));
}

TEST_F(RosBag2PlayTestFixture, seek_forward) {
  auto topic_types = std::vector<rosbag2_storage::TopicMetadata>{
    {"topic1", "test_msgs/BasicTypes", "", ""}};

  const size_t num_msgs_to_publish = 7;
  const size_t expected_number_of_messages = num_msgs_to_publish - 1;
  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages;
  messages.reserve(num_msgs_to_publish);
  int64_t start_time_ms = 1000;
  int64_t message_spacing_ms = 100;
  auto primitive_message = get_messages_basic_types()[0];
  for (size_t i = 0; i < num_msgs_to_publish; i++) {
    primitive_message->int32_value = static_cast<int32_t>(i + 1);
    const int64_t timestamp = start_time_ms + message_spacing_ms * static_cast<int64_t>(i);
    messages.push_back(serialize_test_message("topic1", timestamp, primitive_message));
  }

  auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
  prepared_mock_reader->prepare(messages, topic_types);
  auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));
  play_options_.read_ahead_queue_size = 3;
  auto player = std::make_shared<MockPlayer>(std::move(reader), storage_options_, play_options_);

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

  EXPECT_TRUE(player->seek((start_time_ms + message_spacing_ms) * 1000000 + 1));
  EXPECT_TRUE(player->play_next());
  player->resume();
  player_future.get();
  await_received_messages.get();

  auto replayed_topic1 = sub_->get_received_messages<test_msgs::msg::BasicTypes>("/topic1");
  ASSERT_THAT(replayed_topic1, SizeIs(expected_number_of_messages));
  EXPECT_EQ(replayed_topic1[0]->int32_value, 1);
  EXPECT_EQ(replayed_topic1[1]->int32_value, 3);
}
