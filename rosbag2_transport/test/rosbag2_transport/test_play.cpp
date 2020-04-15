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
#include <future>
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include "rosbag2_test_common/subscription_manager.hpp"

#include "rosbag2_transport/rosbag2_transport.hpp"
#include "rosbag2_transport/logging.hpp"

#include "test_msgs/msg/arrays.hpp"
#include "test_msgs/msg/basic_types.hpp"
#include "test_msgs/message_fixtures.hpp"

#include "rosbag2_transport_test_fixture.hpp"

using namespace ::testing;  // NOLINT
using namespace rosbag2_transport;  // NOLINT
using namespace std::chrono_literals;  // NOLINT
using namespace rosbag2_test_common;  // NOLINT

class RosBag2PlayTestFixture : public Rosbag2TransportTestFixture
{
public:
  RosBag2PlayTestFixture()
  : Rosbag2TransportTestFixture()
  {
    rclcpp::init(0, nullptr);
    sub_ = std::make_shared<SubscriptionManager>();
  }

  ~RosBag2PlayTestFixture() override
  {
    rclcpp::shutdown();
  }

  std::shared_ptr<SubscriptionManager> sub_;
};

TEST_F(RosBag2PlayTestFixture, recorded_messages_are_played_for_all_topics)
{
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
  {serialize_test_message("topic1", 500, primitive_message1),
    serialize_test_message("topic1", 700, primitive_message1),
    serialize_test_message("topic1", 900, primitive_message1),
    serialize_test_message("topic2", 550, complex_message1),
    serialize_test_message("topic2", 750, complex_message1),
    serialize_test_message("topic2", 950, complex_message1)};

  auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
  prepared_mock_reader->prepare(messages, topic_types);
  reader_ = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));

  // Due to a problem related to the subscriber, we play many (3) messages but make the subscriber
  // node spin only until 2 have arrived. Hence the 2 as `launch_subscriber()` argument.
  sub_->add_subscription<test_msgs::msg::BasicTypes>("/topic1", 2);
  sub_->add_subscription<test_msgs::msg::Arrays>("/topic2", 2);

  auto await_received_messages = sub_->spin_subscriptions();

  Rosbag2Transport rosbag2_transport(reader_, writer_, info_);
  rosbag2_transport.play(storage_options_, play_options_);

  await_received_messages.get();

  auto replayed_test_primitives = sub_->get_received_messages<test_msgs::msg::BasicTypes>(
    "/topic1");
  EXPECT_THAT(replayed_test_primitives, SizeIs(Ge(2u)));
  EXPECT_THAT(
    replayed_test_primitives,
    Each(Pointee(Field(&test_msgs::msg::BasicTypes::int32_value, 42))));

  auto replayed_test_arrays = sub_->get_received_messages<test_msgs::msg::Arrays>(
    "/topic2");
  EXPECT_THAT(replayed_test_arrays, SizeIs(Ge(2u)));
  EXPECT_THAT(
    replayed_test_arrays,
    Each(
      Pointee(
        Field(
          &test_msgs::msg::Arrays::bool_values,
          ElementsAre(true, false, true)))));
  EXPECT_THAT(
    replayed_test_arrays,
    Each(
      Pointee(
        Field(
          &test_msgs::msg::Arrays::float32_values,
          ElementsAre(40.0f, 2.0f, 0.0f)))));
}

class RosBag2PlayQosOverrideTestFixture : public RosBag2PlayTestFixture
{
public:
  RosBag2PlayQosOverrideTestFixture()
  : RosBag2PlayTestFixture()
  {
    // Because this test only cares about compatibility (receiving any messages at all)
    // We publish one more message than we expect to receive, to avoid caring about
    // shutdown edge behaviors that are not explicitly being tested here.
    const size_t num_msgs_to_publish = num_msgs_to_wait_for_ + 1;
    topic_timestamps_ms_.reserve(num_msgs_to_publish);
    for (size_t i = 0; i < num_msgs_to_publish; i++) {
      topic_timestamps_ms_.push_back(start_time_ms_ + message_spacing_ms_ * i);
    }

    messages_.reserve(topic_timestamps_ms_.size());
    for (const auto topic_timestamp : topic_timestamps_ms_) {
      messages_.push_back(serialize_test_message(topic_name_, topic_timestamp, basic_msg_));
    }

    auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
    prepared_mock_reader->prepare(messages_, topic_types_);
    reader_ = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));
  }

  const std::string topic_name_{"/test_topic"};
  const std::string msg_type_{"test_msgs/BasicTypes"};
  const size_t num_msgs_to_wait_for_{3};
  test_msgs::msg::BasicTypes::SharedPtr basic_msg_{get_messages_basic_types()[0]};
  const std::vector<rosbag2_storage::TopicMetadata> topic_types_{
    {topic_name_, msg_type_, "" /*serialization_format*/, "" /*offered_qos_profiles*/}
  };
  const int64_t start_time_ms_{500};
  const int64_t message_spacing_ms_{200};
  std::vector<int64_t> topic_timestamps_ms_{};
  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages_;
};

TEST_F(RosBag2PlayQosOverrideTestFixture, topic_qos_profiles_overridden)
{
  // By default playback uses DURABILITY_VOLATILE.
  // When we request DURABILITY_TRANSIENT_LOCAL, we should expect no connection because that
  // request is incompatible.
  // However, if we override playback to offer DURABILITY_TRANSIENT_LOCAL, now we should expect
  // to receive messages.
  const auto qos_request = rclcpp::QoS{rclcpp::KeepAll()}.reliable().transient_local();
  const auto qos_playback_override = rclcpp::QoS{rclcpp::KeepAll()}.reliable().transient_local();

  const auto topic_qos_profile_overrides = std::unordered_map<std::string, rclcpp::QoS>{
    std::pair<std::string, rclcpp::QoS>{topic_name_, qos_playback_override},
  };

  sub_->add_subscription<test_msgs::msg::BasicTypes>(
    topic_name_, num_msgs_to_wait_for_, qos_request);
  auto await_received_messages = sub_->spin_subscriptions();
  play_options_.topic_qos_profile_overrides = topic_qos_profile_overrides;

  Rosbag2Transport rosbag2_transport{reader_, writer_, info_};
  rosbag2_transport.play(storage_options_, play_options_);

  // This should normally take less than 1s - just making it shorter than 60s default
  const auto future_result = await_received_messages.wait_for(5s);
  EXPECT_NE(future_result, std::future_status::timeout);
  rosbag2_transport.shutdown();

  const auto received_messages =
    sub_->get_received_messages<test_msgs::msg::BasicTypes>(topic_name_);
  EXPECT_FALSE(received_messages.empty());
}

TEST_F(RosBag2PlayQosOverrideTestFixture, topic_qos_profiles_overridden_incompatible)
{
  // By default playback offers RELIABILITY_RELIABLE
  // We request RELIABILITY_RELIABLE here, which should be compatible.
  // However, we override the playback to offer RELIABILITY_BEST_EFFORT,
  // which should not be a compatible offer and therefore we should receive no messages.
  const auto qos_request = rclcpp::QoS{rclcpp::KeepAll()}.reliable();
  const auto qos_playback_override = rclcpp::QoS{rclcpp::KeepAll()}.best_effort();

  const auto topic_qos_profile_overrides = std::unordered_map<std::string, rclcpp::QoS>{
    std::pair<std::string, rclcpp::QoS>{topic_name_, qos_playback_override},
  };
  play_options_.topic_qos_profile_overrides = topic_qos_profile_overrides;

  sub_->add_subscription<test_msgs::msg::BasicTypes>(
    topic_name_, num_msgs_to_wait_for_, qos_request);
  auto await_received_messages = sub_->spin_subscriptions();

  Rosbag2Transport rosbag2_transport{reader_, writer_, info_};
  rosbag2_transport.play(storage_options_, play_options_);

  using namespace std::chrono_literals;
  const auto future_result = await_received_messages.wait_for(3s);
  EXPECT_EQ(future_result, std::future_status::timeout);

  rosbag2_transport.shutdown();
  const auto received_messages =
    sub_->get_received_messages<test_msgs::msg::BasicTypes>(topic_name_);
  EXPECT_EQ(received_messages.size(), 0u);
}
