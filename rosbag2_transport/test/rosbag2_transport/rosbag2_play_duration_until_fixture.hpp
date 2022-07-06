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
#ifndef ROSBAG2_TRANSPORT__ROSBAG2_PLAY_DURATION_UNTIL_FIXTURE_HPP_
#define ROSBAG2_TRANSPORT__ROSBAG2_PLAY_DURATION_UNTIL_FIXTURE_HPP_

#include <gmock/gmock.h>

#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include "rosbag2_test_common/subscription_manager.hpp"

#include "rosbag2_transport/player.hpp"

#include "test_msgs/msg/arrays.hpp"
#include "test_msgs/msg/basic_types.hpp"
#include "test_msgs/message_fixtures.hpp"

#include "mock_player.hpp"
#include "rosbag2_play_test_fixture.hpp"
#include "rosbag2_transport_test_fixture.hpp"

class RosBag2PlayDurationAndUntilTestFixture : public RosBag2PlayTestFixture
{
public:
  static constexpr int kIntValue{32};

  static constexpr float kFloat1Value{40.};
  static constexpr float kFloat2Value{2.};
  static constexpr float kFloat3Value{0.};

  static constexpr bool kBool1Value{false};
  static constexpr bool kBool2Value{true};
  static constexpr bool kBool3Value{false};

  static constexpr const char * kTopic1Name_{"topic1"};
  static constexpr const char * kTopic2Name_{"topic2"};
  static constexpr const char * kTopic1_{"/topic1"};
  static constexpr const char * kTopic2_{"/topic2"};

  inline void EvalReplayedPrimitives(
    const std::vector<test_msgs::msg::BasicTypes::SharedPtr> & replayed_primitives) const
  {
    EXPECT_THAT(
      replayed_primitives,
      testing::Each(
        testing::Pointee(
          testing::Field(&test_msgs::msg::BasicTypes::int32_value, kIntValue))));
  }

  inline void EvalReplayedFloatArrayPrimitives(
    const std::vector<test_msgs::msg::Arrays::SharedPtr> & replayed_float_array_primitive) const
  {
    EXPECT_THAT(
      replayed_float_array_primitive,
      testing::Each(
        testing::Pointee(
          testing::Field(
            &test_msgs::msg::Arrays::float32_values,
            testing::ElementsAre(kFloat1Value, kFloat2Value, kFloat3Value)))));
  }

  inline void EvalReplayedBoolArrayPrimitives(
    const std::vector<test_msgs::msg::Arrays::SharedPtr> & replayed_bool_array_primitive) const
  {
    EXPECT_THAT(
      replayed_bool_array_primitive,
      testing::Each(
        testing::Pointee(
          testing::Field(
            &test_msgs::msg::Arrays::bool_values,
            testing::ElementsAre(kBool1Value, kBool2Value, kBool3Value)))));
  }

  std::vector<rosbag2_storage::TopicMetadata> get_topic_types()
  {
    return {{kTopic1Name_, "test_msgs/BasicTypes", "", ""},
      {kTopic2Name_, "test_msgs/Arrays", "", ""}};
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
    {serialize_test_message(kTopic1Name_, 100, primitive_message1),
      serialize_test_message(kTopic2Name_, 120, complex_message1),
      serialize_test_message(kTopic1Name_, 200, primitive_message1),
      serialize_test_message(kTopic2Name_, 220, complex_message1),
      serialize_test_message(kTopic1Name_, 300, primitive_message1),
      serialize_test_message(kTopic2Name_, 320, complex_message1)};
    // @}
    return messages;
  }

  class PlayerInitializationConfiguration
  {
public:
    PlayerInitializationConfiguration & PlaybackUntilTimestampInMillis(
      const int64_t playback_until_timestamp_ms)
    {
      playback_until_timestamp_ms_ = playback_until_timestamp_ms;
      return *this;
    }

    PlayerInitializationConfiguration & PlaybackDurationInMillis(const int64_t playback_duration_ms)
    {
      playback_duration_ms_ = playback_duration_ms;
      return *this;
    }

    PlayerInitializationConfiguration & ExpectedNumberOfMessagesOnTopic1(const size_t num_messages)
    {
      expected_number_of_messages_on_topic1_ = num_messages;
      return *this;
    }

    PlayerInitializationConfiguration & ExpectedNumberOfMessagesOnTopic2(const size_t num_messages)
    {
      expected_number_of_messages_on_topic2_ = num_messages;
      return *this;
    }

    void AndPlay()
    {
      auto topic_types = test_fixture_->get_topic_types();
      auto messages = test_fixture_->get_serialized_messages();

      auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
      prepared_mock_reader->prepare(messages, topic_types);
      auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));

      test_fixture_->sub_->add_subscription<test_msgs::msg::BasicTypes>(
        kTopic1_, expected_number_of_messages_on_topic1_);
      test_fixture_->sub_->add_subscription<test_msgs::msg::Arrays>(
        kTopic2_, expected_number_of_messages_on_topic2_);

      test_fixture_->play_options_.playback_until_timestamp =
        RCL_MS_TO_NS(playback_until_timestamp_ms_);
      test_fixture_->play_options_.playback_duration =
        rclcpp::Duration(std::chrono::milliseconds(playback_duration_ms_));
      test_fixture_->player_ = std::make_shared<MockPlayer>(
        std::move(
          reader), test_fixture_->storage_options_, test_fixture_->play_options_);

      // Wait for discovery to match publishers with subscribers
      ASSERT_TRUE(
        test_fixture_->sub_->spin_and_wait_for_matched(
          test_fixture_->player_->get_list_of_publishers(),
          std::chrono::seconds(5)));

      auto await_received_messages = test_fixture_->sub_->spin_subscriptions();
      ASSERT_TRUE(test_fixture_->player_->play());
      await_received_messages.get();
    }

protected:
    friend class RosBag2PlayDurationAndUntilTestFixture;

    explicit PlayerInitializationConfiguration(
      RosBag2PlayDurationAndUntilTestFixture * test_fixture)
    : test_fixture_(test_fixture) {}

private:
    RosBag2PlayDurationAndUntilTestFixture * test_fixture_{};
    int64_t playback_until_timestamp_ms_{-1};
    int64_t playback_duration_ms_{-1};
    size_t expected_number_of_messages_on_topic1_{3};
    size_t expected_number_of_messages_on_topic2_{3};
  };

  PlayerInitializationConfiguration InitializePlayerWith()
  {
    return PlayerInitializationConfiguration(this);
  }

  std::shared_ptr<MockPlayer> player_;
};

#endif  // ROSBAG2_TRANSPORT__ROSBAG2_PLAY_DURATION_UNTIL_FIXTURE_HPP_
