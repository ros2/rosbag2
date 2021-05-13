// Copyright 2021, Apex.AI Software Innovations GmbH.
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
#include "test_msgs/msg/arrays.hpp"
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

  void wait_for_playback_to_start()
  {
    while (!playing_messages_from_queue_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
};

TEST_F(RosBag2PlayTestFixture, play_next_with_false_preconditions) {
  auto primitive_message = get_messages_basic_types()[0];
  primitive_message->int32_value = 42;

  auto topic_types = std::vector<rosbag2_storage::TopicMetadata>{
    {"topic1", "test_msgs/BasicTypes", "", ""}};

  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages =
  {serialize_test_message("topic1", 2100, primitive_message)};

  auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
  prepared_mock_reader->prepare(messages, topic_types);
  auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));
  auto player = std::make_shared<MockPlayer>(std::move(reader), storage_options_, play_options_);

  ASSERT_FALSE(player->is_paused());
  ASSERT_FALSE(player->play_next());

  player->pause();
  ASSERT_TRUE(player->is_paused());
  ASSERT_FALSE(player->play_next());
}

TEST_F(RosBag2PlayTestFixture, play_next_playing_all_messages_without_delays) {
  auto primitive_message = get_messages_basic_types()[0];
  primitive_message->int32_value = 42;

  auto topic_types = std::vector<rosbag2_storage::TopicMetadata>{
    {"topic1", "test_msgs/BasicTypes", "", ""}};

  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages =
  {
    serialize_test_message("topic1", 2100, primitive_message),
    serialize_test_message("topic1", 3300, primitive_message),
    serialize_test_message("topic1", 4600, primitive_message),
    serialize_test_message("topic1", 5900, primitive_message)
  };

  auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
  prepared_mock_reader->prepare(messages, topic_types);
  auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));
  auto player = std::make_shared<MockPlayer>(std::move(reader), storage_options_, play_options_);

  sub_ = std::make_shared<SubscriptionManager>();
  sub_->add_subscription<test_msgs::msg::BasicTypes>("/topic1", messages.size());

  // Wait for discovery to match publishers with subscribers
  ASSERT_TRUE(
    sub_->spin_and_wait_for_matched(player->get_list_of_publishers(), std::chrono::seconds(30)));

  auto await_received_messages = sub_->spin_subscriptions();

  player->pause();
  ASSERT_TRUE(player->is_paused());

  auto player_future = std::async(std::launch::async, [&player]() -> void {player->play();});
  player->wait_for_playback_to_start();

  ASSERT_TRUE(player->is_paused());
  auto start = std::chrono::steady_clock::now();
  ASSERT_TRUE(player->play_next());
  size_t played_messages = 1;
  while (player->play_next()) {
    played_messages++;
  }
  auto replay_time = std::chrono::steady_clock::now() - start;
  ASSERT_EQ(played_messages, messages.size());
  ASSERT_THAT(replay_time, Lt(std::chrono::seconds(2)));
  ASSERT_TRUE(player->is_paused());
  player->resume();
  player_future.get();
  await_received_messages.get();

  auto replayed_topic1 = sub_->get_received_messages<test_msgs::msg::BasicTypes>("/topic1");
  EXPECT_THAT(replayed_topic1, SizeIs(messages.size()));
}

TEST_F(RosBag2PlayTestFixture, play_next_playing_one_by_one_messages_with_the_same_timestamp) {
  auto primitive_message = get_messages_basic_types()[0];
  primitive_message->int32_value = 42;

  auto topic_types = std::vector<rosbag2_storage::TopicMetadata>{
    {"topic1", "test_msgs/BasicTypes", "", ""}};

  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages =
  {
    serialize_test_message("topic1", 1000, primitive_message),
    serialize_test_message("topic1", 1000, primitive_message),
    serialize_test_message("topic1", 1000, primitive_message),
    serialize_test_message("topic1", 1000, primitive_message)
  };

  auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
  prepared_mock_reader->prepare(messages, topic_types);
  auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));
  auto player = std::make_shared<MockPlayer>(std::move(reader), storage_options_, play_options_);

  sub_ = std::make_shared<SubscriptionManager>();
  sub_->add_subscription<test_msgs::msg::BasicTypes>("/topic1", messages.size());

  // Wait for discovery to match publishers with subscribers
  ASSERT_TRUE(
    sub_->spin_and_wait_for_matched(player->get_list_of_publishers(), std::chrono::seconds(30)));

  auto await_received_messages = sub_->spin_subscriptions();

  player->pause();
  ASSERT_TRUE(player->is_paused());

  auto player_future = std::async(std::launch::async, [&player]() -> void {player->play();});
  player->wait_for_playback_to_start();

  ASSERT_TRUE(player->is_paused());
  ASSERT_TRUE(player->play_next());
  size_t played_messages = 1;
  while (player->play_next()) {
    // Yield CPU resources for player-play() running in separate thread to make sure that it
    // will not play extra messages.
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    played_messages++;
  }
  ASSERT_EQ(played_messages, messages.size());
  ASSERT_TRUE(player->is_paused());
  player->resume();
  player_future.get();
  await_received_messages.get();

  auto replayed_topic1 = sub_->get_received_messages<test_msgs::msg::BasicTypes>("/topic1");
  EXPECT_THAT(replayed_topic1, SizeIs(messages.size()));
}

TEST_F(RosBag2PlayTestFixture, play_respect_messages_timing_after_play_next) {
  auto primitive_message = get_messages_basic_types()[0];
  primitive_message->int32_value = 42;

  auto topic_types = std::vector<rosbag2_storage::TopicMetadata>{
    {"topic1", "test_msgs/BasicTypes", "", ""}};

  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages =
  {
    serialize_test_message("topic1", 1500, primitive_message),
    serialize_test_message("topic1", 2500, primitive_message),
    serialize_test_message("topic1", 2700, primitive_message),
    serialize_test_message("topic1", 2800, primitive_message)
  };

  auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
  prepared_mock_reader->prepare(messages, topic_types);
  auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));
  auto player = std::make_shared<MockPlayer>(std::move(reader), storage_options_, play_options_);

  sub_ = std::make_shared<SubscriptionManager>();
  sub_->add_subscription<test_msgs::msg::BasicTypes>("/topic1", messages.size());

  // Wait for discovery to match publishers with subscribers
  ASSERT_TRUE(
    sub_->spin_and_wait_for_matched(player->get_list_of_publishers(), std::chrono::seconds(30)));

  auto await_received_messages = sub_->spin_subscriptions();

  player->pause();
  ASSERT_TRUE(player->is_paused());

  auto player_future = std::async(std::launch::async, [&player]() -> void {player->play();});
  player->wait_for_playback_to_start();

  ASSERT_TRUE(player->is_paused());
  ASSERT_TRUE(player->play_next());
  ASSERT_TRUE(player->play_next());
  ASSERT_TRUE(player->is_paused());
  player->resume();
  auto start = std::chrono::steady_clock::now();
  player_future.get();
  auto replay_time = std::chrono::steady_clock::now() - start;

  auto expected_replay_time =
    std::chrono::nanoseconds(messages.back()->time_stamp - messages[1]->time_stamp);
  // Check for lower bound with some tolerance
  ASSERT_THAT(replay_time, Gt(expected_replay_time - std::chrono::milliseconds(50)));
  // Check for upper bound with some tolerance
  ASSERT_THAT(replay_time, Lt(expected_replay_time + std::chrono::milliseconds(100)));

  await_received_messages.get();
  auto replayed_topic1 = sub_->get_received_messages<test_msgs::msg::BasicTypes>("/topic1");
  EXPECT_THAT(replayed_topic1, SizeIs(messages.size()));
}

TEST_F(RosBag2PlayTestFixture, player_can_resume_after_play_next) {
  auto primitive_message = get_messages_basic_types()[0];
  primitive_message->int32_value = 42;

  auto topic_types = std::vector<rosbag2_storage::TopicMetadata>{
    {"topic1", "test_msgs/BasicTypes", "", ""}};

  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages =
  {
    serialize_test_message("topic1", 300, primitive_message),
    serialize_test_message("topic1", 500, primitive_message),
    serialize_test_message("topic1", 700, primitive_message)
  };

  auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
  prepared_mock_reader->prepare(messages, topic_types);
  auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));
  auto player = std::make_shared<MockPlayer>(std::move(reader), storage_options_, play_options_);

  sub_ = std::make_shared<SubscriptionManager>();
  sub_->add_subscription<test_msgs::msg::BasicTypes>("/topic1", messages.size());

  // Wait for discovery to match publishers with subscribers
  ASSERT_TRUE(
    sub_->spin_and_wait_for_matched(player->get_list_of_publishers(), std::chrono::seconds(30)));

  auto await_received_messages = sub_->spin_subscriptions();

  player->pause();
  ASSERT_TRUE(player->is_paused());

  auto player_future = std::async(std::launch::async, [&player]() -> void {player->play();});
  player->wait_for_playback_to_start();

  ASSERT_TRUE(player->is_paused());
  ASSERT_TRUE(player->play_next());
  ASSERT_TRUE(player->is_paused());
  player->resume();
  player_future.get();
  await_received_messages.get();

  auto replayed_topic1 = sub_->get_received_messages<test_msgs::msg::BasicTypes>("/topic1");
  EXPECT_THAT(replayed_topic1, SizeIs(messages.size()));
}

TEST_F(RosBag2PlayTestFixture, play_next_playing_only_filtered_topics) {
  auto primitive_message1 = get_messages_basic_types()[0];
  primitive_message1->int32_value = 42;

  auto complex_message1 = get_messages_arrays()[0];
  complex_message1->float32_values = {{40.0f, 2.0f, 0.0f}};
  complex_message1->bool_values = {{true, false, true}};

  auto topic_types = std::vector<rosbag2_storage::TopicMetadata>{
    {"topic1", "test_msgs/BasicTypes", "", ""},
    {"topic2", "test_msgs/Arrays", "", ""},
  };

  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages =
  {
    serialize_test_message("topic1", 500, primitive_message1),
    serialize_test_message("topic1", 700, primitive_message1),
    serialize_test_message("topic1", 900, primitive_message1),
    serialize_test_message("topic2", 550, complex_message1),
    serialize_test_message("topic2", 750, complex_message1),
    serialize_test_message("topic2", 950, complex_message1)
  };

  // Filter allows /topic2, blocks /topic1
  play_options_.topics_to_filter = {"topic2"};
  auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
  prepared_mock_reader->prepare(messages, topic_types);
  auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));
  auto player = std::make_shared<MockPlayer>(std::move(reader), storage_options_, play_options_);

  sub_ = std::make_shared<SubscriptionManager>();
  sub_->add_subscription<test_msgs::msg::BasicTypes>("/topic1", 0);
  sub_->add_subscription<test_msgs::msg::Arrays>("/topic2", 3);

  // Wait for discovery to match publishers with subscribers
  ASSERT_TRUE(
    sub_->spin_and_wait_for_matched(player->get_list_of_publishers(), std::chrono::seconds(30)));

  auto await_received_messages = sub_->spin_subscriptions();

  player->pause();
  ASSERT_TRUE(player->is_paused());

  auto player_future = std::async(std::launch::async, [&player]() -> void {player->play();});
  player->wait_for_playback_to_start();

  ASSERT_TRUE(player->is_paused());
  ASSERT_TRUE(player->play_next());

  size_t played_messages = 1;
  while (player->play_next()) {
    played_messages++;
  }
  ASSERT_EQ(played_messages, 3U);

  ASSERT_TRUE(player->is_paused());
  player->resume();
  player_future.get();
  await_received_messages.get();

  auto replayed_topic1 = sub_->get_received_messages<test_msgs::msg::BasicTypes>("/topic1");
  // No messages are allowed to have arrived
  EXPECT_THAT(replayed_topic1, SizeIs(0u));

  auto replayed_topic2 = sub_->get_received_messages<test_msgs::msg::Arrays>("/topic2");
  // All we care is that any messages arrived
  EXPECT_THAT(replayed_topic2, SizeIs(Eq(3u)));
}
