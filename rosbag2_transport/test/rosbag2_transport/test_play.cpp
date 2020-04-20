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

#include "test_msgs/msg/arrays.hpp"
#include "test_msgs/msg/basic_types.hpp"
#include "test_msgs/message_fixtures.hpp"

#include "qos.hpp"

#include "rosbag2_play_test_fixture.hpp"
#include "rosbag2_transport_test_fixture.hpp"

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

TEST_F(RosBag2PlayTestFixture, recorded_messages_are_played_for_filtered_topics)
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

  // Set filter
  rosbag2_storage::StorageFilter storage_filter;
  storage_filter.topics.push_back("topic2");
  reader_->set_filter(storage_filter);

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
  EXPECT_THAT(replayed_test_primitives, SizeIs(Ge(0u)));

  auto replayed_test_arrays = sub_->get_received_messages<test_msgs::msg::Arrays>(
    "/topic2");
  EXPECT_THAT(replayed_test_arrays, SizeIs(Ge(2u)));

  // Set new filter
  auto prepared_mock_reader2 = std::make_unique<MockSequentialReader>();
  reader_.reset();
  reader_ = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader2));
  storage_filter.topics.clear();
  storage_filter.topics.push_back("topic1");
  reader_->set_filter(storage_filter);

  await_received_messages = sub_->spin_subscriptions();

  rosbag2_transport = Rosbag2Transport(reader_, writer_, info_);
  rosbag2_transport.play(storage_options_, play_options_);

  await_received_messages.get();

  replayed_test_primitives = sub_->get_received_messages<test_msgs::msg::BasicTypes>(
    "/topic1");
  EXPECT_THAT(replayed_test_primitives, SizeIs(Ge(2u)));

  replayed_test_arrays = sub_->get_received_messages<test_msgs::msg::Arrays>(
    "/topic2");
  EXPECT_THAT(replayed_test_arrays, SizeIs(Ge(0u)));

  // Reset filter
  auto prepared_mock_reader3 = std::make_unique<MockSequentialReader>();
  reader_.reset();
  reader_ = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader3));
  reader_->reset_filter();

  await_received_messages = sub_->spin_subscriptions();

  rosbag2_transport = Rosbag2Transport(reader_, writer_, info_);
  rosbag2_transport.play(storage_options_, play_options_);

  await_received_messages.get();

  replayed_test_primitives = sub_->get_received_messages<test_msgs::msg::BasicTypes>(
    "/topic1");
  EXPECT_THAT(replayed_test_primitives, SizeIs(Ge(2u)));

  replayed_test_arrays = sub_->get_received_messages<test_msgs::msg::Arrays>(
    "/topic2");
  EXPECT_THAT(replayed_test_arrays, SizeIs(Ge(2u)));
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
      {topic_name_, msg_type_, "" /*serialization_format*/, serialized_offered_qos});

    auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
    prepared_mock_reader->prepare(messages_, topic_types_);
    reader_ = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));
  }

  template<typename Duration>
  void play_and_wait(Duration timeout, bool expect_timeout = false)
  {
    auto await_received_messages = sub_->spin_subscriptions();
    Rosbag2Transport transport(reader_, writer_, info_);
    transport.play(storage_options_, play_options_);
    const auto result = await_received_messages.wait_for(timeout);
    // Must EXPECT, can't ASSERT because we need to shut down the transport
    if (expect_timeout) {
      EXPECT_EQ(result, std::future_status::timeout);
    } else {
      EXPECT_NE(result, std::future_status::timeout);
    }
    transport.shutdown();
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
