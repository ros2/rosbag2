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

TEST_F(RosBag2PlayTestFixture, topic_qos_profiles_overriden)
{
  const auto topic_name = "/test_topic";
  const auto msg_type = "test_msgs/BasicTypes";
  const auto num_msgs = 3;
  auto basic_msg = get_messages_basic_types()[0];
  basic_msg->int32_value = 42;
  const auto topic_types = std::vector<rosbag2_storage::TopicMetadata>{
    {topic_name, msg_type, "" /*serialization_format*/, "" /*offered_qos_profiles*/}
  };

  const std::vector<int64_t> topic_timestamps_ms = {500, 700, 900};
  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages;
  messages.reserve(topic_timestamps_ms.size());
  for (const auto topic_timestamp : topic_timestamps_ms) {
    messages.push_back(serialize_test_message(topic_name, topic_timestamp, basic_msg));
  }

  auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
  prepared_mock_reader->prepare(messages, topic_types);
  reader_ = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));

  const auto qos_request = rclcpp::QoS{rclcpp::KeepAll()}.durability_volatile();
  sub_->add_subscription<test_msgs::msg::BasicTypes>(topic_name, num_msgs, qos_request);
  auto await_received_messages = sub_->spin_subscriptions();

  // The previous subscriber requested durability VOLATILE which is the default in rosbag2.
  // We override the requested durability to TRANSIENT_LOCAL so that we can receive messages.
  // If the previous subscription requested TRANSIENT_LOCAL and we overrode with VOLATILE, then we
  // would not receive any messages.
  const auto qos_override = rclcpp::QoS{rclcpp::KeepAll()}.transient_local();
  const auto topic_qos_profile_overrides = std::unordered_map<std::string, rclcpp::QoS>{
    std::pair<std::string, rclcpp::QoS>{topic_name, qos_override},
  };
  play_options_.topic_qos_profile_overrides = topic_qos_profile_overrides;

  Rosbag2Transport rosbag2_transport(reader_, writer_, info_);
  rosbag2_transport.play(storage_options_, play_options_);

  await_received_messages.get();
  const auto received_messages_1 =
    sub_->get_received_messages<test_msgs::msg::BasicTypes>(topic_name);

  EXPECT_GT(received_messages_1.size(), 0u);
}
