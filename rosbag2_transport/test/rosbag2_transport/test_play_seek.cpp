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
#include <string>
#include <utility>
#include <vector>

#include "mock_player.hpp"
#include "rosbag2_cpp/writers/sequential_writer.hpp"
#include "rosbag2_play_test_fixture.hpp"
#include "rosbag2_storage/storage_options.hpp"
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
      {"topic1", "test_msgs/BasicTypes", rmw_get_serialization_format(), ""}};

    const rcpputils::fs::path base{_SRC_RESOURCES_DIR_PATH};
    const rcpputils::fs::path bag_path = base / "test_bag_for_seek";

    storage_options_ = rosbag2_storage::StorageOptions({bag_path.string(), "sqlite3", 0, 0, 0});
    play_options_.read_ahead_queue_size = 2;
    reader_ = std::make_unique<rosbag2_cpp::Reader>();

    check_metadata();
  }


  void check_metadata()
  {
    std::unique_ptr<rosbag2_storage::MetadataIo> metadata_io =
      std::make_unique<rosbag2_storage::MetadataIo>();
    ASSERT_TRUE(metadata_io->metadata_file_exists(storage_options_.uri));
    rosbag2_storage::BagMetadata metadata = metadata_io->read_metadata(storage_options_.uri);
    ASSERT_EQ(metadata.message_count, num_msgs_in_bag_);
    ASSERT_EQ(start_time_ms_, metadata.starting_time.time_since_epoch().count() / 1000000);
  }

  void create_bag_file_for_tests()
  {
    std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages;
    messages.reserve(num_msgs_in_bag_);

    auto primitive_message = get_messages_basic_types()[0];
    for (size_t i = 0; i < num_msgs_in_bag_; i++) {
      primitive_message->int32_value = static_cast<int32_t>(i + 1);
      const int64_t timestamp = start_time_ms_ + message_spacing_ms_ * static_cast<int64_t>(i);
      messages.push_back(serialize_test_message("topic1", timestamp, primitive_message));
    }

    const rosbag2_cpp::ConverterOptions converter_options(
      {rmw_get_serialization_format(), rmw_get_serialization_format()});

    std::unique_ptr<rosbag2_cpp::writers::SequentialWriter> writer =
      std::make_unique<rosbag2_cpp::writers::SequentialWriter>();

    writer->open(storage_options_, converter_options);
    writer->create_topic(topic_types_[0]);

    for (auto const & message : messages) {
      writer->write(message);
    }
    writer->close();
  }

  const size_t num_msgs_in_bag_ = 5;
  const int64_t start_time_ms_ = 1000;
  const int64_t message_spacing_ms_ = 100;
  std::vector<rosbag2_storage::TopicMetadata> topic_types_;
  std::unique_ptr<rosbag2_cpp::Reader> reader_;
};

TEST_F(RosBag2PlaySeekTestFixture, seek_back_in_time) {
  const size_t expected_number_of_messages = num_msgs_in_bag_ + 2;
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
  EXPECT_TRUE(player->play_next());

  // Jump in timestamp equal to the timestamp in first message - 1 nanosecond
  player->seek(start_time_ms_ * 1000000 - 1);
  EXPECT_TRUE(player->play_next());
  EXPECT_TRUE(player->play_next());
  // Jump in timestamp equal to the timestamp in first message
  player->seek(start_time_ms_ * 1000000);
  EXPECT_TRUE(player->play_next());
  EXPECT_TRUE(player->play_next());
  player->resume();
  player_future.get();
  await_received_messages.get();

  auto replayed_topic1 = sub_->get_received_messages<test_msgs::msg::BasicTypes>("/topic1");
  ASSERT_THAT(replayed_topic1, SizeIs(expected_number_of_messages));

  EXPECT_EQ(replayed_topic1[0]->int32_value, 1);
  EXPECT_EQ(replayed_topic1[1]->int32_value, 2);
  EXPECT_EQ(replayed_topic1[2]->int32_value, 1);
  EXPECT_EQ(replayed_topic1[3]->int32_value, 2);

  for (size_t i = 4; i < replayed_topic1.size(); i++) {
    EXPECT_EQ(replayed_topic1[i]->int32_value, static_cast<int32_t>(i - 3)) << "i=" << i;
  }
}

TEST_F(RosBag2PlaySeekTestFixture, seek_with_timestamp_later_than_in_last_message) {
  const size_t expected_number_of_messages = 0;
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

  // Jump in timestamp equal to the timestamp in last message + 1 nanosecond
  player->seek((start_time_ms_ + message_spacing_ms_ * (num_msgs_in_bag_ - 1)) * 1000000 + 1);

  // shouldn't be able to keep playing since we're at end of bag
  EXPECT_FALSE(player->play_next());
  player->resume();
  player_future.get();
  await_received_messages.get();

  auto replayed_topic1 = sub_->get_received_messages<test_msgs::msg::BasicTypes>("/topic1");
  ASSERT_THAT(replayed_topic1, SizeIs(expected_number_of_messages));
}

TEST_F(RosBag2PlaySeekTestFixture, seek_forward) {
  const size_t expected_number_of_messages = num_msgs_in_bag_ - 1;
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

  // Jump on third message (1200 ms)
  player->seek((start_time_ms_ + message_spacing_ms_ * 2) * 1000000);
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

TEST_F(RosBag2PlaySeekTestFixture, seek_back_in_time_from_the_end_of_the_bag) {
  const size_t expected_number_of_messages = num_msgs_in_bag_ + num_msgs_in_bag_ - 2;
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
  // Play all messages  from bag
  for (size_t i = 0; i < num_msgs_in_bag_; i++) {
    EXPECT_TRUE(player->play_next());
  }
  EXPECT_FALSE(player->play_next());  // Make sure there are no messages to play

  // Jump on third message (1200 ms)
  player->seek((start_time_ms_ + message_spacing_ms_ * 2) * 1000000);
  EXPECT_TRUE(player->play_next());
  player->resume();
  player_future.get();
  await_received_messages.get();

  auto replayed_topic1 = sub_->get_received_messages<test_msgs::msg::BasicTypes>("/topic1");
  EXPECT_THAT(replayed_topic1, SizeIs(expected_number_of_messages));

  for (size_t i = 0; i < num_msgs_in_bag_; i++) {
    EXPECT_EQ(replayed_topic1[i]->int32_value, static_cast<int32_t>(i + 1)) << "i=" << i;
  }

  for (size_t i = 0; i < (replayed_topic1.size() - num_msgs_in_bag_); i++) {
    EXPECT_EQ(
      replayed_topic1[i + num_msgs_in_bag_]->int32_value,
      static_cast<int32_t>(i + 3)) << "i=" << i;
  }
}

TEST_F(RosBag2PlaySeekTestFixture, seek_forward_from_the_end_of_the_bag) {
  const size_t expected_number_of_messages = num_msgs_in_bag_;
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
  // Play all messages  from bag
  for (size_t i = 0; i < num_msgs_in_bag_; i++) {
    EXPECT_TRUE(player->play_next());
  }
  EXPECT_FALSE(player->play_next());  // Make sure there are no messages to play

  // Jump in timestamp equal to the timestamp in last message + 1 nanosecond
  player->seek((start_time_ms_ + message_spacing_ms_ * (num_msgs_in_bag_ - 1)) * 1000000 + 1);
  EXPECT_FALSE(player->play_next());
  player->resume();
  player_future.get();
  await_received_messages.get();

  auto replayed_topic1 = sub_->get_received_messages<test_msgs::msg::BasicTypes>("/topic1");
  EXPECT_THAT(replayed_topic1, SizeIs(expected_number_of_messages));

  for (size_t i = 0; i < replayed_topic1.size(); i++) {
    EXPECT_EQ(replayed_topic1[i]->int32_value, static_cast<int32_t>(i + 1)) << "i=" << i;
  }
}
