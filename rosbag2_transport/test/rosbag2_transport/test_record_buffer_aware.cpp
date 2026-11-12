// Copyright 2026 Open Source Robotics Foundation, Inc.
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
#include <cstddef>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "mock_sequential_writer.hpp"
#include "rclcpp/generic_subscription.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rcpputils/scope_exit.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"
#include "rosbag2_test_common/publication_manager.hpp"
#include "rosbag2_test_common/wait_for.hpp"

#include "rosbag2_transport/record_options.hpp"
#include "rosbag2_transport/recorder.hpp"

#include "test_msgs/message_fixtures.hpp"
#include "test_msgs/msg/arrays.hpp"
#include "test_msgs/msg/strings.hpp"
#include "test_msgs/msg/unbounded_sequences.hpp"

#include "record_integration_fixture.hpp"

using namespace ::testing;  // NOLINT

namespace
{
template<typename MessageT>
std::vector<uint8_t> expected_bytes(const MessageT & message)
{
  rclcpp::Serialization<MessageT> serialization;
  rclcpp::SerializedMessage serialized;
  serialization.serialize_message(&message, &serialized);
  const auto & rcl_message = serialized.get_rcl_serialized_message();
  return std::vector<uint8_t>(rcl_message.buffer, rcl_message.buffer + rcl_message.buffer_length);
}

std::vector<uint8_t> recorded_bytes(
  const std::shared_ptr<const rosbag2_storage::SerializedBagMessage> & message)
{
  const auto & data = *message->serialized_data;
  return std::vector<uint8_t>(data.buffer, data.buffer + data.buffer_length);
}
}  // namespace

// With acceptable_buffer_backends set to something other than "cpu", the recorder subscribes
// with rosbag2_transport::BufferAwareSubscription instead of a generic subscription. Plain CPU
// publishers must still be recorded, with the same size and content as the normal serialization.
TEST_F(RecordIntegrationTestFixture, buffer_aware_recording_stores_equivalent_messages)
{
  auto array_message = get_messages_arrays()[0];
  const std::string array_topic = "/array_topic";
  auto string_message = get_messages_strings()[1];
  const std::string string_topic = "/string_topic";
  auto sequence_message = get_messages_unbounded_sequences()[0];
  const std::string sequence_topic = "/sequence_topic";

  rosbag2_test_common::PublicationManager pub_manager;
  pub_manager.setup_publisher(array_topic, array_message, 2);
  pub_manager.setup_publisher(string_topic, string_message, 2);
  pub_manager.setup_publisher(sequence_topic, sequence_message, 2);

  rosbag2_transport::RecordOptions record_options;
  record_options.topics = {array_topic, string_topic, sequence_topic};
  record_options.rmw_serialization_format = "rmw_format";
  record_options.topic_polling_interval = std::chrono::milliseconds(50);
  record_options.acceptable_buffer_backends = "any";
  auto recorder = std::make_shared<rosbag2_transport::Recorder>(
    std::move(writer_), storage_options_, record_options);
  recorder->record();

  constexpr size_t expected_messages = 6;
  std::vector<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>> recorded_messages;
  start_async_spin(recorder);
  {
    auto cleanup_process_handle = rcpputils::make_scope_exit([&]() {stop_spinning();});

    ASSERT_TRUE(pub_manager.wait_for_matched(array_topic.c_str()));
    ASSERT_TRUE(pub_manager.wait_for_matched(string_topic.c_str()));
    ASSERT_TRUE(pub_manager.wait_for_matched(sequence_topic.c_str()));

    pub_manager.run_publishers();

    auto & writer = recorder->get_writer_handle();
    MockSequentialWriter & mock_writer =
      static_cast<MockSequentialWriter &>(writer.get_implementation_handle());
    auto ret = rosbag2_test_common::wait_until_condition(
      [ =, &mock_writer]() {
        return mock_writer.get_number_of_recorded_messages() >= expected_messages;
      },
      std::chrono::seconds(5));
    EXPECT_TRUE(ret) << "failed to capture expected messages in time";
    recorded_messages = mock_writer.get_messages();
  }

  ASSERT_THAT(recorded_messages, SizeIs(expected_messages));

  // The recorder must have used the buffer-aware subscription, not a generic one: a generic
  // subscription cannot take messages delivered through a non-CPU buffer backend.
  const auto & subscriptions = recorder->subscriptions();
  ASSERT_THAT(subscriptions, SizeIs(3));
  for (const auto & [topic, subscription] : subscriptions) {
    EXPECT_EQ(nullptr, dynamic_cast<rclcpp::GenericSubscription *>(subscription.get())) << topic;
  }

  // The stored size must equal the normal serialization. Raw bytes are not compared because CDR
  // alignment padding is not zeroed and may differ between serializations of the same message.
  const std::unordered_map<std::string, size_t> expected_sizes = {
    {array_topic, expected_bytes(*array_message).size()},
    {string_topic, expected_bytes(*string_message).size()},
    {sequence_topic, expected_bytes(*sequence_message).size()},
  };
  std::unordered_map<std::string, size_t> count_per_topic;
  for (const auto & message : recorded_messages) {
    ASSERT_TRUE(expected_sizes.count(message->topic_name) == 1) << message->topic_name;
    EXPECT_EQ(expected_sizes.at(message->topic_name), recorded_bytes(message).size()) <<
      message->topic_name;
    count_per_topic[message->topic_name]++;
  }
  EXPECT_EQ(2u, count_per_topic[array_topic]);
  EXPECT_EQ(2u, count_per_topic[string_topic]);
  EXPECT_EQ(2u, count_per_topic[sequence_topic]);

  // The deserialized content must equal what was published, for every message.
  auto array_messages = filter_messages<test_msgs::msg::Arrays>(recorded_messages, array_topic);
  auto string_messages = filter_messages<test_msgs::msg::Strings>(recorded_messages, string_topic);
  auto sequence_messages = filter_messages<test_msgs::msg::UnboundedSequences>(
    recorded_messages, sequence_topic);
  ASSERT_THAT(array_messages, SizeIs(2));
  ASSERT_THAT(string_messages, SizeIs(2));
  ASSERT_THAT(sequence_messages, SizeIs(2));
  for (const auto & message : array_messages) {
    EXPECT_EQ(*array_message, *message);
  }
  for (const auto & message : string_messages) {
    EXPECT_EQ(*string_message, *message);
  }
  for (const auto & message : sequence_messages) {
    EXPECT_EQ(*sequence_message, *message);
  }
}

// A backend name that is not installed must fail when the recorder is constructed, instead of
// producing a recorder that runs but never subscribes to anything.
TEST_F(RecordIntegrationTestFixture, unknown_buffer_backend_name_fails_fast)
{
  rosbag2_transport::RecordOptions record_options;
  record_options.topics = {"/some_topic"};
  record_options.rmw_serialization_format = "rmw_format";
  record_options.acceptable_buffer_backends = "cpu, no_such_backend";

  try {
    auto recorder = std::make_shared<rosbag2_transport::Recorder>(
      std::move(writer_), storage_options_, record_options);
    FAIL() << "Expected std::invalid_argument for an unknown buffer backend";
  } catch (const std::invalid_argument & e) {
    EXPECT_THAT(std::string(e.what()), HasSubstr("no_such_backend"));
    EXPECT_THAT(std::string(e.what()), HasSubstr("Available backends"));
  }
}
