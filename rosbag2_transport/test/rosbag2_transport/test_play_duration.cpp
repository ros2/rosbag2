// Copyright 2022, Open Source Robotics Corporation.
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
// limitations under the License.Bosch Software Innovations GmbH.

#include <gmock/gmock.h>

#include <chrono>
#include <future>
#include <memory>
#include <vector>
#include <utility>

#include "rclcpp/duration.hpp"

#include "test_msgs/msg/arrays.hpp"
#include "test_msgs/msg/basic_types.hpp"
#include "test_msgs/message_fixtures.hpp"

#include "mock_player.hpp"
#include "rosbag2_play_duration_until_fixture.hpp"
#include "rosbag2_play_test_fixture.hpp"
#include "rosbag2_transport_test_fixture.hpp"

using namespace ::testing;  // NOLINT
using namespace rosbag2_transport;  // NOLINT
using namespace std::chrono_literals;  // NOLINT
using namespace rosbag2_test_common;  // NOLINT

class RosBag2PlayDurationTestFixture : public RosBag2PlayDurationAndUntilTestFixture {};

TEST_F(RosBag2PlayDurationTestFixture, play_for_all_are_played_due_to_duration)
{
  InitializePlayerWith().PlaybackDurationInMillis(350).AndPlay();

  auto replayed_test_primitives = sub_->get_received_messages<test_msgs::msg::BasicTypes>(
    kTopic1_);
  EXPECT_THAT(replayed_test_primitives, SizeIs(Eq(3u)));
  EvalReplayedPrimitives(replayed_test_primitives);

  auto replayed_test_arrays = sub_->get_received_messages<test_msgs::msg::Arrays>(
    kTopic2_);
  EXPECT_THAT(replayed_test_arrays, SizeIs(Eq(3u)));
  EvalReplayedBoolArrayPrimitives(replayed_test_arrays);
  EvalReplayedFloatArrayPrimitives(replayed_test_arrays);
}

TEST_F(RosBag2PlayDurationTestFixture, play_for_none_are_played_due_to_duration)
{
  auto primitive_message1 = get_messages_basic_types()[0];
  auto primitive_message2 = get_messages_basic_types()[0];
  primitive_message1->int32_value = 1;
  primitive_message2->int32_value = 2;

  auto topic_types = std::vector<rosbag2_storage::TopicMetadata>{
    {1, kTopic1Name_, "test_msgs/BasicTypes", "", {}, ""}};

  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages =
  {
    serialize_test_message(kTopic1Name_, 50, primitive_message1),
    serialize_test_message(kTopic1Name_, 100, primitive_message2),
  };

  auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
  prepared_mock_reader->prepare(messages, topic_types);
  auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));

  play_options_.playback_duration = rclcpp::Duration(std::chrono::milliseconds(0));

  std::shared_ptr<MockPlayer> player = std::make_shared<MockPlayer>(
    std::move(reader), storage_options_, play_options_);

  using SerializedBagMessage = rosbag2_storage::SerializedBagMessage;
  testing::MockFunction<void(std::shared_ptr<SerializedBagMessage>)> mock_pre_callback;
  EXPECT_CALL(mock_pre_callback, Call(_)).Times(Exactly(0));

  testing::MockFunction<void(std::shared_ptr<SerializedBagMessage>)> mock_post_callback;
  EXPECT_CALL(mock_post_callback, Call(_)).Times(Exactly(0));

  auto pre_callback_handle =
    player->add_on_play_message_pre_callback(mock_pre_callback.AsStdFunction());
  ASSERT_NE(pre_callback_handle, Player::invalid_callback_handle);

  auto post_callback_handle =
    player->add_on_play_message_post_callback(mock_post_callback.AsStdFunction());
  ASSERT_NE(post_callback_handle, Player::invalid_callback_handle);

  ASSERT_TRUE(player->play());
  player->wait_for_playback_to_finish();
}

TEST_F(RosBag2PlayDurationTestFixture, play_for_less_than_the_total_duration)
{
  auto primitive_message1 = get_messages_basic_types()[0];
  auto primitive_message2 = get_messages_basic_types()[0];
  primitive_message1->int32_value = 1;
  primitive_message2->int32_value = 2;

  auto topic_types = std::vector<rosbag2_storage::TopicMetadata>{
    {1, kTopic1Name_, "test_msgs/BasicTypes", "", {}, ""}};

  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages =
  {
    serialize_test_message(kTopic1Name_, 10, primitive_message1),
    serialize_test_message(kTopic1Name_, 50, primitive_message2),
  };

  auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
  prepared_mock_reader->prepare(messages, topic_types);
  auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));

  // Expect to receive 1 message from play() and 1 messages from play_next in second round
  sub_->add_subscription<test_msgs::msg::BasicTypes>(kTopic1_, 2);
  play_options_.playback_duration = rclcpp::Duration(std::chrono::milliseconds(39));

  std::shared_ptr<MockPlayer> player_ = std::make_shared<MockPlayer>(
    std::move(reader), storage_options_, play_options_);

  // Wait for discovery to match publishers with subscribers
  ASSERT_TRUE(sub_->spin_and_wait_for_matched(player_->get_list_of_publishers(), 5s));

  auto await_received_messages = sub_->spin_subscriptions();
  player_->play();
  player_->wait_for_playback_to_finish();


  // Playing one more time with play_next to save time and count messages
  player_->pause();
  player_->play();

  ASSERT_TRUE(player_->play_next());
  ASSERT_FALSE(player_->play_next());
  player_->resume();
  player_->wait_for_playback_to_finish();
  await_received_messages.get();
  auto replayed_topic1 = sub_->get_received_messages<test_msgs::msg::BasicTypes>(kTopic1_);
  EXPECT_THAT(replayed_topic1, SizeIs(2));
  EXPECT_EQ(replayed_topic1[0]->int32_value, 1);
  EXPECT_EQ(replayed_topic1[1]->int32_value, 1);
}

TEST_F(
  RosBag2PlayDurationTestFixture,
  play_for_intermediate_duration_recorded_messages_with_filtered_topics)
{
  // Filter allows /topic2, blocks /topic1
  play_options_.topics_to_filter = {"topic2"};
  InitializePlayerWith()
  .PlaybackDurationInMillis(270)
  .ExpectedNumberOfMessagesOnTopic1(0)
  .ExpectedNumberOfMessagesOnTopic2(2)
  .AndPlay();

  auto replayed_test_primitives =
    sub_->get_received_messages<test_msgs::msg::BasicTypes>("/topic1");
  // No messages are allowed to have arrived
  EXPECT_THAT(replayed_test_primitives, SizeIs(Eq(0u)));

  auto replayed_test_arrays = sub_->get_received_messages<test_msgs::msg::Arrays>("/topic2");
  // Some messages should have arrived.
  EXPECT_THAT(replayed_test_arrays, SizeIs(Eq(2u)));
  EvalReplayedBoolArrayPrimitives(replayed_test_arrays);
  EvalReplayedFloatArrayPrimitives(replayed_test_arrays);
}

TEST_F(RosBag2PlayDurationTestFixture, play_should_return_false_when_interrupted)
{
  auto topic_types = std::vector<rosbag2_storage::TopicMetadata>{
    {1, kTopic1Name_, "test_msgs/BasicTypes", "", {}, ""}};

  auto primitive_message = get_messages_basic_types()[0];
  primitive_message->int32_value = kIntValue;
  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages =
  {
    serialize_test_message(kTopic1Name_, 50, primitive_message),
    serialize_test_message(kTopic1Name_, 100, primitive_message),
  };

  auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
  prepared_mock_reader->prepare(messages, topic_types);
  auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));

  // Let the player only reproduce one message.
  sub_->add_subscription<test_msgs::msg::BasicTypes>(kTopic1_, 1);
  play_options_.playback_duration = rclcpp::Duration(std::chrono::milliseconds(75));

  std::shared_ptr<MockPlayer> player_ = std::make_shared<MockPlayer>(
    std::move(reader), storage_options_, play_options_);

  // Wait for discovery to match publishers with subscribers
  ASSERT_TRUE(sub_->spin_and_wait_for_matched(player_->get_list_of_publishers(), 5s));

  auto await_received_messages = sub_->spin_subscriptions();
  player_->pause();
  player_->play();
  player_->wait_for_playback_to_start();
  ASSERT_TRUE(player_->is_paused());
  ASSERT_FALSE(player_->play());

  player_->resume();
  player_->wait_for_playback_to_finish();
  await_received_messages.get();
  auto replayed_topic1 = sub_->get_received_messages<test_msgs::msg::BasicTypes>(kTopic1_);
  EXPECT_THAT(replayed_topic1, SizeIs(1));
}
