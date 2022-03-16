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
#include <string>
#include <vector>
#include <unordered_map>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include "rosbag2_test_common/subscription_manager.hpp"

#include "rosbag2_transport/player.hpp"
#include "rosbag2_transport/qos.hpp"

#include "test_msgs/msg/arrays.hpp"
#include "test_msgs/msg/basic_types.hpp"
#include "test_msgs/message_fixtures.hpp"

#include "rosbag2_play_test_fixture.hpp"
#include "rosbag2_transport_test_fixture.hpp"

#include "mock_player.hpp"

using namespace ::testing;  // NOLINT
using namespace rosbag2_transport;  // NOLINT
using namespace std::chrono_literals;  // NOLINT
using namespace rosbag2_test_common;  // NOLINT

constexpr int kIntValue{32};

constexpr float kFloat1Value{40.};
constexpr float kFloat2Value{2.};
constexpr float kFloat3Value{0.};

constexpr bool kBool1Value{false};
constexpr bool kBool2Value{true};
constexpr bool kBool3Value{false};

#define EVAL_REPLAYED_PRIMITIVES(replayed_primitives) \
  EXPECT_THAT( \
    replayed_primitives, \
    Each(Pointee(Field(&test_msgs::msg::BasicTypes::int32_value, kIntValue))))

#define EVAL_REPLAYED_FLOAT_ARRAY_PRIMITIVES(replayed_float_array_primitive) \
  EXPECT_THAT( \
    replayed_float_array_primitive, \
    Each( \
      Pointee( \
        Field( \
          &test_msgs::msg::Arrays::float32_values, \
          ElementsAre(kFloat1Value, kFloat2Value, kFloat3Value)))))

#define EVAL_REPLAYED_BOOL_ARRAY_PRIMITIVES(replayed_bool_array_primitive) \
  EXPECT_THAT( \
    replayed_bool_array_primitive, \
    Each( \
      Pointee( \
        Field( \
          &test_msgs::msg::Arrays::bool_values, \
          ElementsAre(kBool1Value, kBool2Value, kBool3Value)))))

class RosBag2PlayForTestFixture : public RosBag2PlayTestFixture
{
public:
  static constexpr const char * kTopic1Name{"topic1"};
  static constexpr const char * kTopic2Name{"topic2"};
  static constexpr const char * kTopic1{"/topic1"};
  static constexpr const char * kTopic2{"/topic2"};

  std::vector<rosbag2_storage::TopicMetadata> get_topic_types()
  {
    return {{kTopic1Name, "test_msgs/BasicTypes", "", ""},
      {kTopic2Name, "test_msgs/Arrays", "", ""}};
  }

  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>>
  get_serialized_messages()
  {
    auto primitive_message1 = get_messages_basic_types()[0];
    primitive_message1->int32_value = kIntValue;

    auto complex_message1 = get_messages_arrays()[0];
    complex_message1->float32_values = {{kFloat1Value, kFloat2Value, kFloat3Value}};
    complex_message1->bool_values = {{kBool1Value, kBool2Value, kBool3Value}};

    // @{ Ordering matters. The mock reader implementation moves messages
    //    around without any knowledge about message chronology. It just picks
    //    the next one Make sure to keep the list in order or sort it!
    std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages =
    {serialize_test_message(kTopic1Name, 500, primitive_message1),
      serialize_test_message(kTopic2Name, 550, complex_message1),
      serialize_test_message(kTopic1Name, 700, primitive_message1),
      serialize_test_message(kTopic2Name, 750, complex_message1),
      serialize_test_message(kTopic1Name, 900, primitive_message1),
      serialize_test_message(kTopic2Name, 950, complex_message1)};
    // @}
    return messages;
  }

  void InitPlayerWithPlaybackDurationAndPlay(int64_t milliseconds)
  {
    auto topic_types = get_topic_types();
    auto messages = get_serialized_messages();

    auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
    prepared_mock_reader->prepare(messages, topic_types);
    auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));

    sub_->add_subscription<test_msgs::msg::BasicTypes>(kTopic1, 3);
    sub_->add_subscription<test_msgs::msg::Arrays>(kTopic2, 3);

    play_options_.playback_duration =
      rclcpp::Duration(std::chrono::nanoseconds(std::chrono::milliseconds(milliseconds)));
    player_ = std::make_shared<MockPlayer>(
      std::move(
        reader), storage_options_, play_options_);
    // Wait for discovery to match publishers with subscribers
    ASSERT_TRUE(
      sub_->spin_and_wait_for_matched(player_->get_list_of_publishers(), std::chrono::seconds(5)));

    auto await_received_messages = sub_->spin_subscriptions();

    player_->play();

    await_received_messages.get();
  }

  std::shared_ptr<MockPlayer> player_;
};


TEST_F(RosBag2PlayForTestFixture, play_for_all_are_played_due_to_duration)
{
  InitPlayerWithPlaybackDurationAndPlay(1000);

  auto replayed_test_primitives = sub_->get_received_messages<test_msgs::msg::BasicTypes>(
    kTopic1);
  EXPECT_THAT(replayed_test_primitives, SizeIs(Eq(3u)));
  EVAL_REPLAYED_PRIMITIVES(replayed_test_primitives);

  auto replayed_test_arrays = sub_->get_received_messages<test_msgs::msg::Arrays>(
    kTopic2);
  EXPECT_THAT(replayed_test_arrays, SizeIs(Eq(3u)));
  EVAL_REPLAYED_BOOL_ARRAY_PRIMITIVES(replayed_test_arrays);
  EVAL_REPLAYED_FLOAT_ARRAY_PRIMITIVES(replayed_test_arrays);
}

TEST_F(RosBag2PlayForTestFixture, play_for_none_are_played_due_to_duration)
{
  InitPlayerWithPlaybackDurationAndPlay(300);

  auto replayed_test_primitives = sub_->get_received_messages<test_msgs::msg::BasicTypes>(
    kTopic1);
  EXPECT_THAT(replayed_test_primitives, SizeIs(Eq(0u)));

  auto replayed_test_arrays = sub_->get_received_messages<test_msgs::msg::Arrays>(
    kTopic2);
  EXPECT_THAT(replayed_test_arrays, SizeIs(Eq(0u)));
}

TEST_F(RosBag2PlayForTestFixture, play_for_less_than_the_total_duration)
{
  InitPlayerWithPlaybackDurationAndPlay(800);

  auto replayed_test_primitives = sub_->get_received_messages<test_msgs::msg::BasicTypes>(
    kTopic1);
  EXPECT_THAT(replayed_test_primitives, SizeIs(Eq(2u)));
  EVAL_REPLAYED_PRIMITIVES(replayed_test_primitives);

  auto replayed_test_arrays = sub_->get_received_messages<test_msgs::msg::Arrays>(
    kTopic2);
  EXPECT_THAT(replayed_test_arrays, SizeIs(Eq(2u)));
  EVAL_REPLAYED_BOOL_ARRAY_PRIMITIVES(replayed_test_arrays);
  EVAL_REPLAYED_FLOAT_ARRAY_PRIMITIVES(replayed_test_arrays);
}

class RosBag2PlayForFilteredTopicTestFixture : public RosBag2PlayForTestFixture
{
public:
  void SetUp() override
  {
    // Filter allows /topic2, blocks /topic1
    play_options_.topics_to_filter = {"topic2"};
  }
};

TEST_F(
  RosBag2PlayForFilteredTopicTestFixture,
  play_for_full_duration_recorded_messages_with_filtered_topics)
{
  InitPlayerWithPlaybackDurationAndPlay(1000);

  auto replayed_test_primitives =
    sub_->get_received_messages<test_msgs::msg::BasicTypes>("/topic1");
  // No messages are allowed to have arrived
  EXPECT_THAT(replayed_test_primitives, SizeIs(Eq(0u)));

  auto replayed_test_arrays = sub_->get_received_messages<test_msgs::msg::Arrays>("/topic2");
  // All messages should have arrived.
  EXPECT_THAT(replayed_test_arrays, SizeIs(Eq(3u)));
  EVAL_REPLAYED_BOOL_ARRAY_PRIMITIVES(replayed_test_arrays);
  EVAL_REPLAYED_FLOAT_ARRAY_PRIMITIVES(replayed_test_arrays);
}

TEST_F(
  RosBag2PlayForFilteredTopicTestFixture,
  play_for_short_duration_recorded_messages_with_filtered_topics)
{
  InitPlayerWithPlaybackDurationAndPlay(300);

  auto replayed_test_primitives =
    sub_->get_received_messages<test_msgs::msg::BasicTypes>("/topic1");
  // No messages are allowed to have arrived
  EXPECT_THAT(replayed_test_primitives, SizeIs(Eq(0u)));

  auto replayed_test_arrays = sub_->get_received_messages<test_msgs::msg::Arrays>("/topic2");
  // All messages should have arrived.
  EXPECT_THAT(replayed_test_arrays, SizeIs(Eq(0u)));
}

TEST_F(
  RosBag2PlayForFilteredTopicTestFixture,
  play_for_intermediate_duration_recorded_messages_with_filtered_topics)
{
  InitPlayerWithPlaybackDurationAndPlay(800);

  auto replayed_test_primitives =
    sub_->get_received_messages<test_msgs::msg::BasicTypes>("/topic1");
  // No messages are allowed to have arrived
  EXPECT_THAT(replayed_test_primitives, SizeIs(Eq(0u)));

  auto replayed_test_arrays = sub_->get_received_messages<test_msgs::msg::Arrays>("/topic2");
  // Some messages should have arrived.
  EXPECT_THAT(replayed_test_arrays, SizeIs(Eq(2u)));
  EVAL_REPLAYED_BOOL_ARRAY_PRIMITIVES(replayed_test_arrays);
  EVAL_REPLAYED_FLOAT_ARRAY_PRIMITIVES(replayed_test_arrays);
}
