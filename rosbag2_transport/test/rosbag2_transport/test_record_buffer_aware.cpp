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
#include "rclcpp/node.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rclcpp/typesupport_helpers.hpp"
#include "rcpputils/scope_exit.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"
#include "rosbag2_test_common/publication_manager.hpp"
#include "rosbag2_test_common/wait_for.hpp"

#include "rosbag2_transport/record_options.hpp"
#include "rosbag2_transport/recorder.hpp"

#include "test_msgs/message_fixtures.hpp"
#include "test_msgs/msg/arrays.hpp"
#include "test_msgs/msg/strings.hpp"
#include "test_msgs/msg/unbounded_sequences.hpp"

#include "buffer_aware_subscription.hpp"
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

// With acceptable_buffer_backends set to something other than "cpu", topics whose type has a
// rosidl::Buffer field are subscribed with rosbag2_transport::BufferAwareSubscription instead of a
// generic subscription. Plain CPU publishers must still be recorded, with the same size and
// content as the normal serialization, and other topics must keep the generic subscription.
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

  // Only a type with a rosidl::Buffer field needs the buffer-aware subscription (a generic one
  // cannot take messages delivered through a non-CPU backend). Types without one stay generic.
  const auto & subscriptions = recorder->subscriptions();
  ASSERT_THAT(subscriptions, SizeIs(3));
  EXPECT_EQ(nullptr, dynamic_cast<rclcpp::GenericSubscription *>(
      subscriptions.at(sequence_topic).get())) << sequence_topic;
  EXPECT_NE(nullptr, dynamic_cast<rclcpp::GenericSubscription *>(
      subscriptions.at(array_topic).get())) << array_topic;
  EXPECT_NE(nullptr, dynamic_cast<rclcpp::GenericSubscription *>(
      subscriptions.at(string_topic).get())) << string_topic;

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

// Spellings that only name "cpu" must behave exactly like "cpu": no buffer-aware subscription,
// and buffer_aware_topics is rejected.
TEST_F(RecordIntegrationTestFixture, cpu_only_backend_spellings_are_treated_as_cpu)
{
  const std::string topic = "/sequence_topic";
  auto publisher_node = std::make_shared<rclcpp::Node>("cpu_spellings_publisher_node");
  auto publisher =
    publisher_node->create_publisher<test_msgs::msg::UnboundedSequences>(topic, 10);

  for (const std::string backends : {"", " ", "cpu ", " cpu", "cpu,cpu", " , cpu,"}) {
    SCOPED_TRACE("acceptable_buffer_backends: '" + backends + "'");
    rosbag2_transport::RecordOptions record_options;
    record_options.topics = {topic};
    record_options.rmw_serialization_format = "rmw_format";
    record_options.acceptable_buffer_backends = backends;

    auto options_with_topics = record_options;
    options_with_topics.buffer_aware_topics = {topic};
    EXPECT_THROW(
      std::make_shared<rosbag2_transport::Recorder>(
        std::make_shared<rosbag2_cpp::Writer>(std::make_unique<MockSequentialWriter>()),
        storage_options_, options_with_topics),
      std::invalid_argument);

    auto recorder = std::make_shared<rosbag2_transport::Recorder>(
      std::make_shared<rosbag2_cpp::Writer>(std::make_unique<MockSequentialWriter>()),
      storage_options_, record_options);
    ASSERT_TRUE(rosbag2_test_common::wait_until_condition(
        [&]() {return recorder->count_publishers(topic) > 0;}, std::chrono::seconds(5)));
    recorder->record();
    recorder->stop_discovery();
    const auto & subscriptions = recorder->subscriptions();
    ASSERT_EQ(1u, subscriptions.count(topic));
    EXPECT_NE(nullptr, dynamic_cast<rclcpp::GenericSubscription *>(
        subscriptions.at(topic).get()));
    recorder->stop();
  }
}

// "any" wins over every other name, as it does in the RMW, whatever the spelling around it.
TEST_F(RecordIntegrationTestFixture, any_buffer_backend_takes_precedence)
{
  const std::string topic = "/sequence_topic";
  auto publisher_node = std::make_shared<rclcpp::Node>("any_precedence_publisher_node");
  auto publisher =
    publisher_node->create_publisher<test_msgs::msg::UnboundedSequences>(topic, 10);

  for (const std::string backends : {" any ", "cpu, any,cpu", "any,no_such_backend"}) {
    SCOPED_TRACE("acceptable_buffer_backends: '" + backends + "'");
    rosbag2_transport::RecordOptions record_options;
    record_options.topics = {topic};
    record_options.rmw_serialization_format = "rmw_format";
    record_options.acceptable_buffer_backends = backends;
    record_options.buffer_aware_topics = {topic};

    auto recorder = std::make_shared<rosbag2_transport::Recorder>(
      std::make_shared<rosbag2_cpp::Writer>(std::make_unique<MockSequentialWriter>()),
      storage_options_, record_options);
    ASSERT_TRUE(rosbag2_test_common::wait_until_condition(
        [&]() {return recorder->count_publishers(topic) > 0;}, std::chrono::seconds(5)));
    recorder->record();
    recorder->stop_discovery();
    const auto & subscriptions = recorder->subscriptions();
    ASSERT_EQ(1u, subscriptions.count(topic));
    EXPECT_EQ(nullptr, dynamic_cast<rclcpp::GenericSubscription *>(
        subscriptions.at(topic).get()));
    recorder->stop();
  }
}

namespace
{
const rosidl_message_type_support_t * introspection_type_support(const std::string & type)
{
  static std::vector<std::shared_ptr<rcpputils::SharedLibrary>> libraries;
  libraries.push_back(
    rclcpp::get_typesupport_library(type, "rosidl_typesupport_introspection_cpp"));
  return rclcpp::get_message_typesupport_handle(
    type, "rosidl_typesupport_introspection_cpp", *libraries.back());
}
}  // namespace

TEST(buffer_field_detection, only_types_with_a_buffer_field_are_detected)
{
  using rosbag2_transport::message_type_has_buffer_field;
  EXPECT_TRUE(message_type_has_buffer_field(
      introspection_type_support("test_msgs/msg/UnboundedSequences")));
  EXPECT_TRUE(message_type_has_buffer_field(
      introspection_type_support("sensor_msgs/msg/Image")));
  EXPECT_FALSE(message_type_has_buffer_field(
      introspection_type_support("test_msgs/msg/Strings")));
  EXPECT_FALSE(message_type_has_buffer_field(
      introspection_type_support("test_msgs/msg/BasicTypes")));
  // A nested message without buffer fields is not detected either.
  EXPECT_FALSE(message_type_has_buffer_field(
      introspection_type_support("test_msgs/msg/Nested")));
  EXPECT_FALSE(message_type_has_buffer_field(nullptr));
}

// buffer_aware_topics narrows the buffer-aware subscription to the listed topics, even among
// topics whose type has a Buffer field.
TEST_F(RecordIntegrationTestFixture, buffer_aware_topics_selects_topics_explicitly)
{
  auto message = get_messages_unbounded_sequences()[0];
  const std::string listed_topic = "/listed_sequence_topic";
  const std::string unlisted_topic = "/unlisted_sequence_topic";
  auto string_message = get_messages_strings()[1];
  const std::string string_topic = "/listed_string_topic";

  rosbag2_test_common::PublicationManager pub_manager;
  pub_manager.setup_publisher(listed_topic, message, 2);
  pub_manager.setup_publisher(unlisted_topic, message, 2);
  pub_manager.setup_publisher(string_topic, string_message, 2);

  rosbag2_transport::RecordOptions record_options;
  record_options.topics = {listed_topic, unlisted_topic, string_topic};
  record_options.rmw_serialization_format = "rmw_format";
  record_options.topic_polling_interval = std::chrono::milliseconds(50);
  record_options.acceptable_buffer_backends = "any";
  // The string topic is listed but its type has no Buffer field, so it stays generic.
  record_options.buffer_aware_topics = {listed_topic, string_topic};
  auto recorder = std::make_shared<rosbag2_transport::Recorder>(
    std::move(writer_), storage_options_, record_options);
  recorder->record();

  constexpr size_t expected_messages = 6;
  std::vector<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>> recorded_messages;
  start_async_spin(recorder);
  {
    auto cleanup_process_handle = rcpputils::make_scope_exit([&]() {stop_spinning();});
    ASSERT_TRUE(pub_manager.wait_for_matched(listed_topic.c_str()));
    ASSERT_TRUE(pub_manager.wait_for_matched(unlisted_topic.c_str()));
    ASSERT_TRUE(pub_manager.wait_for_matched(string_topic.c_str()));
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

  const auto & subscriptions = recorder->subscriptions();
  ASSERT_THAT(subscriptions, SizeIs(3));
  EXPECT_EQ(nullptr, dynamic_cast<rclcpp::GenericSubscription *>(
      subscriptions.at(listed_topic).get())) << listed_topic;
  EXPECT_NE(nullptr, dynamic_cast<rclcpp::GenericSubscription *>(
      subscriptions.at(unlisted_topic).get())) << unlisted_topic;
  EXPECT_NE(nullptr, dynamic_cast<rclcpp::GenericSubscription *>(
      subscriptions.at(string_topic).get())) << string_topic;

  for (const auto & topic : {listed_topic, unlisted_topic}) {
    auto sequences = filter_messages<test_msgs::msg::UnboundedSequences>(recorded_messages, topic);
    ASSERT_THAT(sequences, SizeIs(2)) << topic;
    for (const auto & recorded : sequences) {
      EXPECT_EQ(*message, *recorded) << topic;
    }
  }
}

// buffer_aware_topics only makes sense together with a non-CPU backend.
TEST_F(RecordIntegrationTestFixture, buffer_aware_topics_without_a_backend_fails_fast)
{
  rosbag2_transport::RecordOptions record_options;
  record_options.topics = {"/some_topic"};
  record_options.rmw_serialization_format = "rmw_format";
  record_options.buffer_aware_topics = {"/some_topic"};

  EXPECT_THROW(
    std::make_shared<rosbag2_transport::Recorder>(
      std::move(writer_), storage_options_, record_options),
    std::invalid_argument);
}

namespace
{
/// A minimal non-CPU buffer implementation, to reproduce what a buffer backend delivers when the
/// payload of a message could not be imported in time: a non-CPU buffer with no elements.
class FakeBackendBufferImpl : public rosidl::BufferImplBase<uint8_t>
{
public:
  explicit FakeBackendBufferImpl(size_t size)
  : size_(size) {}
  std::string get_backend_type() const override {return "fake";}
  size_t size() const override {return size_;}
  std::unique_ptr<rosidl::BufferImplBase<uint8_t>> to_cpu() const override
  {
    return std::make_unique<rosidl::CpuBufferImpl<uint8_t>>();
  }
  std::unique_ptr<rosidl::BufferImplBase<uint8_t>> clone() const override
  {
    return std::make_unique<FakeBackendBufferImpl>(size_);
  }

private:
  size_t size_;
};

rosidl::Buffer<uint8_t> fake_backend_buffer(size_t size)
{
  return rosidl::Buffer<uint8_t>(std::make_unique<FakeBackendBufferImpl>(size));
}
}  // namespace

TEST(stale_buffer_detection, only_an_empty_non_cpu_buffer_is_stale)
{
  using rosbag2_transport::message_has_stale_buffer;
  const auto * members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(
    introspection_type_support("test_msgs/msg/UnboundedSequences")->data);

  test_msgs::msg::UnboundedSequences message;
  // A default message has empty CPU buffers, which are legitimate empty arrays.
  EXPECT_FALSE(message_has_stale_buffer(&message, members));

  message.uint8_values = fake_backend_buffer(0);
  EXPECT_TRUE(message_has_stale_buffer(&message, members));

  message.uint8_values = fake_backend_buffer(5);
  EXPECT_FALSE(message_has_stale_buffer(&message, members));

  // A whole new CPU buffer replaces the backend one. Assigning a vector to a non-CPU buffer
  // throws.
  message.uint8_values = rosidl::Buffer<uint8_t>(std::vector<uint8_t>{1, 2, 3});
  EXPECT_FALSE(message_has_stale_buffer(&message, members));

  EXPECT_FALSE(message_has_stale_buffer(nullptr, members));
  EXPECT_FALSE(message_has_stale_buffer(&message, nullptr));
}

// A message whose backend buffer arrived empty must be dropped and counted, never passed on.
TEST_F(RecordIntegrationTestFixture, stale_backend_message_is_dropped_and_reported)
{
  auto node = std::make_shared<rclcpp::Node>("stale_buffer_test_node");
  size_t delivered = 0;
  uint64_t reported_total = 0;
  size_t reported_calls = 0;
  auto subscription = std::make_shared<rosbag2_transport::BufferAwareSubscription>(
    node->get_node_base_interface().get(), "/stale_topic", "test_msgs/msg/UnboundedSequences",
    rclcpp::QoS(10), rclcpp::SubscriptionOptions(),
    [&](std::shared_ptr<const rclcpp::SerializedMessage>, const rclcpp::MessageInfo &) {
      ++delivered;
    },
    [&](uint64_t total) {
      reported_total = total;
      ++reported_calls;
    });
  rclcpp::MessageInfo message_info;

  auto stale = subscription->create_message();
  static_cast<test_msgs::msg::UnboundedSequences *>(stale.get())->uint8_values =
    fake_backend_buffer(0);
  subscription->handle_message(stale, message_info);
  EXPECT_EQ(0u, delivered);
  EXPECT_EQ(1u, reported_calls);
  EXPECT_EQ(1u, reported_total);
  EXPECT_EQ(1u, subscription->stale_messages_dropped());

  auto intact = subscription->create_message();
  static_cast<test_msgs::msg::UnboundedSequences *>(intact.get())->uint8_values =
    std::vector<uint8_t>{1, 2, 3};
  subscription->handle_message(intact, message_info);
  EXPECT_EQ(1u, delivered);
  EXPECT_EQ(1u, reported_calls);
  EXPECT_EQ(1u, subscription->stale_messages_dropped());
}
