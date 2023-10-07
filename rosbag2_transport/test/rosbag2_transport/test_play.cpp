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

#include "rosbag2_transport/player.hpp"

#include "test_msgs/msg/arrays.hpp"
#include "test_msgs/msg/basic_types.hpp"
#include "test_msgs/message_fixtures.hpp"

#include "rosbag2_transport/qos.hpp"

#include "rosbag2_play_test_fixture.hpp"
#include "rosbag2_transport_test_fixture.hpp"
#include "mock_player.hpp"

using namespace ::testing;  // NOLINT
using namespace rosbag2_transport;  // NOLINT
using namespace std::chrono_literals;  // NOLINT
using namespace rosbag2_test_common;  // NOLINT

TEST_F(RosBag2PlayTestFixture, recorded_messages_are_played_for_all_topics)
{
  auto primitive_message1 = get_messages_basic_types()[0];
  primitive_message1->int32_value = 42;

  auto complex_message1 = get_messages_arrays()[0];
  complex_message1->float32_values = {{40.0f, 2.0f, 0.0f}};
  complex_message1->bool_values = {{true, false, true}};

  auto topic_types = std::vector<rosbag2_storage::TopicMetadata>{
    {"topic1", "test_msgs/BasicTypes", "", "", ""},
    {"topic2", "test_msgs/Arrays", "", "", ""},
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
  auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));

  // Due to a problem related to the subscriber, we play many (3) messages but make the subscriber
  // node spin only until 2 have arrived. Hence the 2 as `launch_subscriber()` argument.
  sub_->add_subscription<test_msgs::msg::BasicTypes>("/topic1", 2);
  sub_->add_subscription<test_msgs::msg::Arrays>("/topic2", 2);

  auto await_received_messages = sub_->spin_subscriptions();

  auto player = std::make_shared<rosbag2_transport::Player>(
    std::move(
      reader), storage_options_, play_options_);
  player->play();

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

TEST_F(RosBag2PlayTestFixture, recorded_messages_are_played_for_all_topics_with_unknown_type)
{
  auto primitive_message1 = get_messages_basic_types()[0];
  primitive_message1->int32_value = 42;

  auto complex_message1 = get_messages_arrays()[0];
  complex_message1->float32_values = {{40.0f, 2.0f, 0.0f}};
  complex_message1->bool_values = {{true, false, true}};

  auto unknown_message1 = get_messages_basic_types()[0];
  unknown_message1->int32_value = 42;

  auto topic_types = std::vector<rosbag2_storage::TopicMetadata>{
    {"topic1", "test_msgs/BasicTypes", "", "", ""},
    {"topic2", "test_msgs/Arrays", "", "", ""},
    {"topic3", "unknown_msgs/UnknownType", "", "", ""},
  };

  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages =
  {serialize_test_message("topic1", 500, primitive_message1),
    serialize_test_message("topic1", 700, primitive_message1),
    serialize_test_message("topic1", 900, primitive_message1),
    serialize_test_message("topic2", 550, complex_message1),
    serialize_test_message("topic2", 750, complex_message1),
    serialize_test_message("topic2", 950, complex_message1),
    serialize_test_message("topic3", 900, unknown_message1)};

  auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
  prepared_mock_reader->prepare(messages, topic_types);
  auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));

  // Due to a problem related to the subscriber, we play many (3) messages but make the subscriber
  // node spin only until 2 have arrived. Hence the 2 as `launch_subscriber()` argument.
  sub_->add_subscription<test_msgs::msg::BasicTypes>("/topic1", 2);
  sub_->add_subscription<test_msgs::msg::Arrays>("/topic2", 2);

  auto await_received_messages = sub_->spin_subscriptions();

  auto player = std::make_shared<rosbag2_transport::Player>(
    std::move(
      reader), storage_options_, play_options_);
  player->play();

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

TEST_F(RosBag2PlayTestFixture, recorded_messages_are_played_for_filtered_topics)
{
  auto primitive_message1 = get_messages_basic_types()[0];
  primitive_message1->int32_value = 42;

  auto complex_message1 = get_messages_arrays()[0];
  complex_message1->float32_values = {{40.0f, 2.0f, 0.0f}};
  complex_message1->bool_values = {{true, false, true}};

  auto topic_types = std::vector<rosbag2_storage::TopicMetadata>{
    {"topic1", "test_msgs/BasicTypes", "", "", ""},
    {"topic2", "test_msgs/Arrays", "", "", ""},
  };

  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages =
  {serialize_test_message("topic1", 500, primitive_message1),
    serialize_test_message("topic1", 700, primitive_message1),
    serialize_test_message("topic1", 900, primitive_message1),
    serialize_test_message("topic2", 550, complex_message1),
    serialize_test_message("topic2", 750, complex_message1),
    serialize_test_message("topic2", 950, complex_message1)};

  // Filter allows /topic2, blocks /topic1
  {
    play_options_.topics_to_filter = {"topic2"};

    // SubscriptionManager has to be recreated for every unique test
    // If it isn't, message counts accumulate
    sub_.reset();
    sub_ = std::make_shared<SubscriptionManager>();
    sub_->add_subscription<test_msgs::msg::BasicTypes>("/topic1", 0);
    sub_->add_subscription<test_msgs::msg::Arrays>("/topic2", 2);

    auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
    prepared_mock_reader->prepare(messages, topic_types);
    auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));

    auto await_received_messages = sub_->spin_subscriptions();

    auto player = std::make_shared<rosbag2_transport::Player>(
      std::move(
        reader), storage_options_, play_options_);
    player->play();

    await_received_messages.get();

    auto replayed_topic1 = sub_->get_received_messages<test_msgs::msg::BasicTypes>("/topic1");
    // No messages are allowed to have arrived
    EXPECT_THAT(replayed_topic1, SizeIs(0u));

    auto replayed_topic2 = sub_->get_received_messages<test_msgs::msg::Arrays>("/topic2");
    // All we care is that any messages arrived
    EXPECT_THAT(replayed_topic2, SizeIs(Ge(1u)));
  }

  // Filter allows /topic1, blocks /topic2
  {
    play_options_.topics_to_filter = {"topic1"};

    // SubscriptionManager has to be recreated for every unique test
    // otherwise counts accumulate, returning the spin immediately
    sub_.reset();
    sub_ = std::make_shared<SubscriptionManager>();
    sub_->add_subscription<test_msgs::msg::BasicTypes>("/topic1", 2);
    sub_->add_subscription<test_msgs::msg::Arrays>("/topic2", 0);

    auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
    prepared_mock_reader->prepare(messages, topic_types);
    auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));

    auto await_received_messages = sub_->spin_subscriptions();

    auto player = std::make_shared<rosbag2_transport::Player>(
      std::move(
        reader), storage_options_, play_options_);
    player->play();

    await_received_messages.get();

    auto replayed_topic1 = sub_->get_received_messages<test_msgs::msg::BasicTypes>("/topic1");
    // All we care is that any messages arrived
    EXPECT_THAT(replayed_topic1, SizeIs(Ge(1u)));

    auto replayed_topic2 = sub_->get_received_messages<test_msgs::msg::Arrays>("/topic2");
    // No messages are allowed to have arrived
    EXPECT_THAT(replayed_topic2, SizeIs(0u));
  }

  // No filter, receive both topics
  {
    play_options_.topics_to_filter.clear();

    // SubscriptionManager has to be recreated for every unique test
    // otherwise counts accumulate, returning the spin immediately
    sub_.reset();
    sub_ = std::make_shared<SubscriptionManager>();
    sub_->add_subscription<test_msgs::msg::BasicTypes>("/topic1", 2);
    sub_->add_subscription<test_msgs::msg::Arrays>("/topic2", 2);

    auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
    prepared_mock_reader->prepare(messages, topic_types);
    auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));

    auto await_received_messages = sub_->spin_subscriptions();

    auto player = std::make_shared<rosbag2_transport::Player>(
      std::move(
        reader), storage_options_, play_options_);
    player->play();

    await_received_messages.get();

    auto replayed_topic1 = sub_->get_received_messages<test_msgs::msg::BasicTypes>("/topic1");
    // All we care is that any messages arrived
    EXPECT_THAT(replayed_topic1, SizeIs(Ge(1u)));

    auto replayed_topic2 = sub_->get_received_messages<test_msgs::msg::Arrays>("/topic2");
    // All we care is that any messages arrived
    EXPECT_THAT(replayed_topic2, SizeIs(Ge(1u)));
  }
}

TEST_F(RosBag2PlayTestFixture, recorded_messages_are_played_for_filtered_topics_with_unknown_type)
{
  auto primitive_message1 = get_messages_basic_types()[0];
  primitive_message1->int32_value = 42;

  auto complex_message1 = get_messages_arrays()[0];
  complex_message1->float32_values = {{40.0f, 2.0f, 0.0f}};
  complex_message1->bool_values = {{true, false, true}};

  auto unknown_message1 = get_messages_basic_types()[0];
  unknown_message1->int32_value = 42;

  auto topic_types = std::vector<rosbag2_storage::TopicMetadata>{
    {"topic1", "test_msgs/BasicTypes", "", "", ""},
    {"topic2", "test_msgs/Arrays", "", "", ""},
    {"topic3", "unknown_msgs/UnknownType", "", "", ""},
  };

  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages =
  {serialize_test_message("topic1", 500, primitive_message1),
    serialize_test_message("topic1", 700, primitive_message1),
    serialize_test_message("topic1", 900, primitive_message1),
    serialize_test_message("topic2", 550, complex_message1),
    serialize_test_message("topic2", 750, complex_message1),
    serialize_test_message("topic2", 950, complex_message1),
    serialize_test_message("topic3", 900, unknown_message1)};

  {
    play_options_.topics_to_filter = {"topic2"};

    auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
    prepared_mock_reader->prepare(messages, topic_types);
    auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));

    // Due to a problem related to the subscriber, we play many (3) messages but make the subscriber
    // node spin only until 2 have arrived. Hence the 2 as `launch_subscriber()` argument.

    sub_->add_subscription<test_msgs::msg::Arrays>("/topic2", 2);
    auto await_received_messages = sub_->spin_subscriptions();

    auto player = std::make_shared<rosbag2_transport::Player>(
      std::move(
        reader), storage_options_, play_options_);
    player->play();

    await_received_messages.get();

    auto replayed_test_primitives = sub_->get_received_messages<test_msgs::msg::BasicTypes>(
      "/topic1");
    EXPECT_THAT(replayed_test_primitives, SizeIs(Ge(0u)));

    auto replayed_test_arrays = sub_->get_received_messages<test_msgs::msg::Arrays>(
      "/topic2");
    EXPECT_THAT(replayed_test_arrays, SizeIs(Ge(2u)));
  }

  // Set new filter
  {
    play_options_.topics_to_filter = {"topic1"};

    auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
    prepared_mock_reader->prepare(messages, topic_types);
    auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));

    sub_.reset();
    sub_ = std::make_shared<SubscriptionManager>();
    sub_->add_subscription<test_msgs::msg::BasicTypes>("/topic1", 2);

    auto await_received_messages = sub_->spin_subscriptions();

    auto player = std::make_shared<rosbag2_transport::Player>(
      std::move(
        reader), storage_options_, play_options_);
    player->play();

    await_received_messages.get();

    auto replayed_test_primitives = sub_->get_received_messages<test_msgs::msg::BasicTypes>(
      "/topic1");
    EXPECT_THAT(replayed_test_primitives, SizeIs(Ge(2u)));

    auto replayed_test_arrays = sub_->get_received_messages<test_msgs::msg::Arrays>(
      "/topic2");
    EXPECT_THAT(replayed_test_arrays, SizeIs(Ge(0u)));
  }

  // Reset filter
  {
    play_options_.topics_to_filter = {"topic1", "topic2"};

    auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
    prepared_mock_reader->prepare(messages, topic_types);
    auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));

    sub_.reset();
    sub_ = std::make_shared<SubscriptionManager>();
    sub_->add_subscription<test_msgs::msg::BasicTypes>("/topic1", 2);
    sub_->add_subscription<test_msgs::msg::Arrays>("/topic2", 2);

    auto await_received_messages = sub_->spin_subscriptions();

    auto player = std::make_shared<rosbag2_transport::Player>(
      std::move(
        reader), storage_options_, play_options_);
    player->play();

    await_received_messages.get();

    auto replayed_test_primitives = sub_->get_received_messages<test_msgs::msg::BasicTypes>(
      "/topic1");
    EXPECT_THAT(replayed_test_primitives, SizeIs(Ge(2u)));

    auto replayed_test_arrays = sub_->get_received_messages<test_msgs::msg::Arrays>(
      "/topic2");
    EXPECT_THAT(replayed_test_arrays, SizeIs(Ge(2u)));
  }
}

TEST_F(RosBag2PlayTestFixture, player_gracefully_exit_by_rclcpp_shutdown_in_pause) {
  auto primitive_message1 = get_messages_basic_types()[0];
  primitive_message1->int32_value = 42;
  auto topic_types = std::vector<rosbag2_storage::TopicMetadata>{
    {"topic1", "test_msgs/BasicTypes", "", "", ""},
  };

  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages =
  {
    serialize_test_message("topic1", 500, primitive_message1),
    serialize_test_message("topic1", 700, primitive_message1)
  };

  auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
  prepared_mock_reader->prepare(messages, topic_types);
  auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));
  auto player = std::make_shared<MockPlayer>(std::move(reader), storage_options_, play_options_);

  player->pause();
  auto player_future = std::async(std::launch::async, [&player]() -> void {player->play();});
  player->wait_for_playback_to_start();
  ASSERT_TRUE(player->is_paused());

  rclcpp::shutdown();
  player_future.get();
}

class RosBag2PlayQosOverrideTestFixture : public RosBag2PlayTestFixture
{
public:
  RosBag2PlayQosOverrideTestFixture()
  : RosBag2PlayTestFixture()
  {
  }

  void initialize(const std::vector<rosbag2_transport::Rosbag2QoS> & offered_qos)
  {
    // Because these tests only cares about compatibility (receiving any messages at all)
    // We publish one more message than we expect to receive, to avoid caring about
    // shutdown edge behaviors that are not explicitly being tested here.
    const size_t num_msgs_to_publish = num_msgs_to_wait_for_ + 1;
    messages_.reserve(num_msgs_to_publish);
    for (size_t i = 0; i < num_msgs_to_publish; i++) {
      const auto timestamp = start_time_ms_ + message_spacing_ms_ * i;
      messages_.push_back(serialize_test_message(topic_name_, timestamp, basic_msg_));
    }

    std::string serialized_offered_qos = "";
    if (!offered_qos.empty()) {
      YAML::Node offered_qos_yaml;
      for (const auto & profile : offered_qos) {
        offered_qos_yaml.push_back(profile);
      }
      serialized_offered_qos = YAML::Dump(offered_qos_yaml);
    }
    topic_types_.push_back(
      {topic_name_, msg_type_, "" /*serialization_format*/, serialized_offered_qos, ""});
  }

  template<typename Duration>
  void play_and_wait(Duration timeout, bool expect_timeout = false)
  {
    auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
    prepared_mock_reader->prepare(messages_, topic_types_);
    auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));

    auto await_received_messages = sub_->spin_subscriptions();
    auto player = std::make_shared<rosbag2_transport::Player>(
      std::move(
        reader), storage_options_, play_options_);
    player->play();
    const auto result = await_received_messages.wait_for(timeout);
    // Must EXPECT, can't ASSERT because transport needs to be shutdown if timed out
    if (expect_timeout) {
      EXPECT_EQ(result, std::future_status::timeout);
    } else {
      EXPECT_NE(result, std::future_status::timeout);
    }
    // Have to rclcpp::shutdown here to make the spin_subscriptions async thread exit
    rclcpp::shutdown();
  }

  const std::string topic_name_{"/test_topic"};
  const std::string msg_type_{"test_msgs/BasicTypes"};
  // Receiving _any_ messages means we've confirmed compatibility in these tests
  const size_t num_msgs_to_wait_for_{1};
  test_msgs::msg::BasicTypes::SharedPtr basic_msg_{get_messages_basic_types()[0]};
  std::vector<rosbag2_storage::TopicMetadata> topic_types_{};
  const int64_t start_time_ms_{500};
  const int64_t message_spacing_ms_{200};
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
  // This should normally take less than 1s - just making it shorter than 60s default
  const auto timeout = 5s;

  initialize({});

  sub_->add_subscription<test_msgs::msg::BasicTypes>(
    topic_name_, num_msgs_to_wait_for_, qos_request);
  play_options_.topic_qos_profile_overrides = topic_qos_profile_overrides;

  // Fails if times out
  play_and_wait(timeout);
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
  // If any messages were going to come in, it should happen in under 1s even in slow scenarios
  const auto timeout = 3s;

  initialize({});

  sub_->add_subscription<test_msgs::msg::BasicTypes>(
    topic_name_, num_msgs_to_wait_for_, qos_request);
  play_options_.topic_qos_profile_overrides = topic_qos_profile_overrides;

  // Fails if it doesn't time out
  play_and_wait(timeout, true /* expect timeout */);
}

TEST_F(RosBag2PlayQosOverrideTestFixture, playback_uses_recorded_transient_local_profile)
{
  // In this test, we subscribe requesting DURABILITY_TRANSIENT_LOCAL.
  // The bag metadata has this recorded for the original Publisher,
  // so playback's offer should be compatible (whereas the default offer would not be)
  const auto transient_local_profile = Rosbag2QoS{Rosbag2QoS{}.transient_local()};
  // This should normally take less than 1s - just making it shorter than 60s default
  const auto timeout = 5s;

  initialize({transient_local_profile});

  sub_->add_subscription<test_msgs::msg::BasicTypes>(
    topic_name_, num_msgs_to_wait_for_, transient_local_profile);

  // Fails if times out
  play_and_wait(timeout);
}

TEST_F(RosBag2PlayQosOverrideTestFixture, playback_uses_recorded_deadline)
{
  // By default, QoS profiles use "unspecified/infinite" offers for duration-based policies
  // The subscription in this test requests a finite Deadline, and by receiving messages
  // we know that the playback has used the recorded finite Deadline duration.

  // The publisher is offering 2Hz. The subscription requests 1Hz, which is less strict of a
  // requirement, so they are compatible.
  const rclcpp::Duration request_deadline{1s};
  const rclcpp::Duration offer_deadline{500ms};
  const auto request_profile = Rosbag2QoS{}.deadline(request_deadline);
  const auto offer_profile = Rosbag2QoS{Rosbag2QoS{}.deadline(offer_deadline)};
  const auto timeout = 5s;

  initialize({offer_profile});
  sub_->add_subscription<test_msgs::msg::BasicTypes>(
    topic_name_, num_msgs_to_wait_for_, request_profile);
  play_and_wait(timeout);
}

TEST_F(RosBag2PlayQosOverrideTestFixture, override_has_precedence_over_recorded)
{
  // In this test, we show that the playback prefers the user override to the recorded values.
  // The subscription requests a Liveliness lease_duration that is shorter than the original
  // recorded publisher offered, so no messages should be passed.
  // However, the override to the publisher offers a shorter duration than the request, so
  // it should now be compatible.
  const rclcpp::Duration liveliness_request{500ms};
  const rclcpp::Duration recorded_liveliness_offer{1000ms};
  const rclcpp::Duration override_liveliness_offer{250ms};
  ASSERT_LT(liveliness_request, recorded_liveliness_offer);
  ASSERT_LT(override_liveliness_offer, liveliness_request);
  const auto request_profile = Rosbag2QoS{}.liveliness_lease_duration(liveliness_request);
  const auto recorded_offer_profile = Rosbag2QoS{Rosbag2QoS{}.liveliness_lease_duration(
      recorded_liveliness_offer)};
  const auto override_offer_profile = Rosbag2QoS{Rosbag2QoS{}.liveliness_lease_duration(
      override_liveliness_offer)};
  const auto topic_qos_profile_overrides = std::unordered_map<std::string, rclcpp::QoS>{
    std::pair<std::string, rclcpp::QoS>{topic_name_, override_offer_profile},
  };
  // This should normally take less than 1s - just making it shorter than 60s default
  const auto timeout = 5s;

  initialize({recorded_offer_profile});

  sub_->add_subscription<test_msgs::msg::BasicTypes>(
    topic_name_, num_msgs_to_wait_for_, request_profile);
  play_options_.topic_qos_profile_overrides = topic_qos_profile_overrides;

  // Fails if times out
  play_and_wait(timeout);
}

TEST_F(RosBag2PlayTestFixture, read_split_callback_is_called)
{
  auto topic_types = std::vector<rosbag2_storage::TopicMetadata>{
    {"topic1", "test_msgs/BasicTypes", "", "", ""},
  };

  auto prepared_mock_reader = std::make_unique<MockSequentialReader>();

  const size_t num_msgs_in_bag = prepared_mock_reader->max_messages_per_file() + 1;
  const int64_t start_time_ms = 100;
  const int64_t message_spacing_ms = 50;

  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages;
  messages.reserve(num_msgs_in_bag);

  auto primitive_message = get_messages_basic_types()[0];
  for (size_t i = 0; i < num_msgs_in_bag; i++) {
    primitive_message->int32_value = static_cast<int32_t>(i + 1);
    const int64_t timestamp = start_time_ms + message_spacing_ms * static_cast<int64_t>(i);
    messages.push_back(serialize_test_message("topic1", timestamp, primitive_message));
  }

  prepared_mock_reader->prepare(messages, topic_types);
  auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));

  bool callback_called = false;
  std::string closed_file, opened_file;
  rosbag2_cpp::bag_events::ReaderEventCallbacks callbacks;
  callbacks.read_split_callback =
    [&callback_called, &closed_file, &opened_file](rosbag2_cpp::bag_events::BagSplitInfo & info) {
      closed_file = info.closed_file;
      opened_file = info.opened_file;
      callback_called = true;
    };
  // This tests adding to the underlying prepared_mock_reader via the Reader instance
  reader->add_event_callbacks(callbacks);

  auto player = std::make_shared<MockPlayer>(std::move(reader), storage_options_, play_options_);

  sub_ = std::make_shared<SubscriptionManager>();
  sub_->add_subscription<test_msgs::msg::BasicTypes>("/topic1", messages.size());

  // Wait for discovery to match publishers with subscribers
  ASSERT_TRUE(
    sub_->spin_and_wait_for_matched(player->get_list_of_publishers(), std::chrono::seconds(30)));

  auto await_received_messages = sub_->spin_subscriptions();

  player->play();

  await_received_messages.get();

  auto replayed_test_primitives = sub_->get_received_messages<test_msgs::msg::BasicTypes>(
    "/topic1");
  EXPECT_THAT(replayed_test_primitives, SizeIs(messages.size()));

  // Confirm that the callback was called and the file names have been sent with the event
  ASSERT_TRUE(callback_called);
  EXPECT_EQ(closed_file, "BagFile0");
  EXPECT_EQ(opened_file, "BagFile1");
}
