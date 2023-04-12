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

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "rosbag2_test_common/publication_manager.hpp"
#include "rosbag2_test_common/wait_for.hpp"

#include "rosbag2_transport/recorder.hpp"

#include "test_msgs/msg/arrays.hpp"
#include "test_msgs/message_fixtures.hpp"

#include "rosbag2_transport/qos.hpp"
#include "record_integration_fixture.hpp"

TEST_F(RecordIntegrationTestFixture, published_messages_from_multiple_topics_are_recorded)
{
  auto array_message = get_messages_arrays()[0];
  std::string array_topic = "/array_topic";

  auto string_message = get_messages_strings()[1];
  std::string string_topic = "/string_topic";

  rosbag2_test_common::PublicationManager pub_manager;
  pub_manager.setup_publisher(array_topic, array_message, 2);
  pub_manager.setup_publisher(string_topic, string_message, 2);

  rosbag2_transport::RecordOptions record_options =
  {false, false, {string_topic, array_topic}, "rmw_format", 100ms};
  auto recorder = std::make_shared<rosbag2_transport::Recorder>(
    std::move(writer_), storage_options_, record_options);
  recorder->record();

  start_async_spin(recorder);

  ASSERT_TRUE(pub_manager.wait_for_matched(array_topic.c_str()));
  ASSERT_TRUE(pub_manager.wait_for_matched(string_topic.c_str()));

  pub_manager.run_publishers();

  auto & writer = recorder->get_writer_handle();
  MockSequentialWriter & mock_writer =
    static_cast<MockSequentialWriter &>(writer.get_implementation_handle());

  size_t expected_messages = 4;
  auto ret = rosbag2_test_common::wait_until_shutdown(
    std::chrono::seconds(5),
    [&mock_writer, &expected_messages]() {
      return mock_writer.get_messages().size() >= expected_messages;
    });
  auto recorded_messages = mock_writer.get_messages();
  EXPECT_TRUE(ret) << "failed to capture expected messages in time";
  EXPECT_THAT(recorded_messages, SizeIs(expected_messages));

  auto recorded_topics = mock_writer.get_topics();
  ASSERT_THAT(recorded_topics, SizeIs(2));
  EXPECT_THAT(recorded_topics.at(string_topic).serialization_format, Eq("rmw_format"));
  EXPECT_THAT(recorded_topics.at(array_topic).serialization_format, Eq("rmw_format"));
  ASSERT_THAT(recorded_messages, SizeIs(4));
  auto string_messages = filter_messages<test_msgs::msg::Strings>(
    recorded_messages, string_topic);
  auto array_messages = filter_messages<test_msgs::msg::Arrays>(
    recorded_messages, array_topic);
  ASSERT_THAT(string_messages, SizeIs(2));
  ASSERT_THAT(array_messages, SizeIs(2));
  EXPECT_THAT(string_messages[0]->string_value, Eq(string_message->string_value));
  EXPECT_THAT(array_messages[0]->bool_values, Eq(array_message->bool_values));
  EXPECT_THAT(array_messages[0]->float32_values, Eq(array_message->float32_values));
}

TEST_F(RecordIntegrationTestFixture, qos_is_stored_in_metadata)
{
  auto string_message = get_messages_strings()[1];
  std::string topic = "/qos_chatter";

  rosbag2_test_common::PublicationManager pub_manager;
  pub_manager.setup_publisher(topic, string_message, 2);

  rosbag2_transport::RecordOptions record_options =
  {false, false, {topic}, "rmw_format", 100ms};
  auto recorder = std::make_shared<rosbag2_transport::Recorder>(
    std::move(writer_), storage_options_, record_options);
  recorder->record();

  start_async_spin(recorder);

  ASSERT_TRUE(pub_manager.wait_for_matched(topic.c_str()));

  pub_manager.run_publishers();

  auto & writer = recorder->get_writer_handle();
  MockSequentialWriter & mock_writer =
    static_cast<MockSequentialWriter &>(writer.get_implementation_handle());

  size_t expected_messages = 2;
  auto ret = rosbag2_test_common::wait_until_shutdown(
    std::chrono::seconds(5),
    [&mock_writer, &expected_messages]() {
      return mock_writer.get_messages().size() >= expected_messages;
    });
  auto recorded_messages = mock_writer.get_messages();
  EXPECT_TRUE(ret) << "failed to capture expected messages in time";
  EXPECT_THAT(recorded_messages, SizeIs(expected_messages));

  auto recorded_topics = mock_writer.get_topics();
  std::string serialized_profiles = recorded_topics.at(topic).offered_qos_profiles;
  // Basic smoke test that the profile was serialized into the metadata as a string.
  EXPECT_THAT(serialized_profiles, ContainsRegex("reliability: 1\n"));
  EXPECT_THAT(serialized_profiles, ContainsRegex("durability: 2\n"));
  EXPECT_THAT(
    serialized_profiles, ContainsRegex(
      "deadline:\n"
      "    sec: .+\n"
      "    nsec: .+\n"
  ));
  EXPECT_THAT(
    serialized_profiles, ContainsRegex(
      "lifespan:\n"
      "    sec: .+\n"
      "    nsec: .+\n"
  ));
  EXPECT_THAT(serialized_profiles, ContainsRegex("liveliness: 1\n"));
  EXPECT_THAT(
    serialized_profiles, ContainsRegex(
      "liveliness_lease_duration:\n"
      "    sec: .+\n"
      "    nsec: .+\n"
  ));
}

TEST_F(RecordIntegrationTestFixture, records_sensor_data)
{
  auto string_message = get_messages_strings()[1];
  std::string topic = "/sensor_chatter";

  rosbag2_test_common::PublicationManager pub_manager;
  pub_manager.setup_publisher(topic, string_message, 2, rclcpp::SensorDataQoS());

  rosbag2_transport::RecordOptions record_options =
  {false, false, {topic}, "rmw_format", 100ms};
  auto recorder = std::make_shared<rosbag2_transport::Recorder>(
    std::move(writer_), storage_options_, record_options);
  recorder->record();

  start_async_spin(recorder);

  ASSERT_TRUE(pub_manager.wait_for_matched(topic.c_str()));

  pub_manager.run_publishers();

  auto & writer = recorder->get_writer_handle();
  MockSequentialWriter & mock_writer =
    static_cast<MockSequentialWriter &>(writer.get_implementation_handle());

  size_t expected_messages = 2;
  auto ret = rosbag2_test_common::wait_until_shutdown(
    std::chrono::seconds(5),
    [&mock_writer, &expected_messages]() {
      return mock_writer.get_messages().size() >= expected_messages;
    });
  auto recorded_messages = mock_writer.get_messages();
  auto recorded_topics = mock_writer.get_topics();
  EXPECT_TRUE(ret) << "failed to capture expected messages in time";
  EXPECT_THAT(recorded_messages, SizeIs(expected_messages));
  EXPECT_EQ(recorded_topics.size(), 1u);
  EXPECT_FALSE(recorded_messages.empty());
}

TEST_F(RecordIntegrationTestFixture, receives_latched_messages)
{
  auto string_message = get_messages_strings()[1];
  std::string topic = "/latched_chatter";

  size_t num_latched_messages = 3;
  auto profile_transient_local = rclcpp::QoS(num_latched_messages).transient_local();
  rosbag2_test_common::PublicationManager pub_manager;
  pub_manager.setup_publisher(
    topic, string_message, num_latched_messages, profile_transient_local);
  // Publish messages before starting recording
  pub_manager.run_publishers();

  rosbag2_transport::RecordOptions record_options =
  {false, false, {topic}, "rmw_format", 100ms};
  auto recorder = std::make_shared<rosbag2_transport::Recorder>(
    std::move(writer_), storage_options_, record_options);
  recorder->record();

  start_async_spin(recorder);

  ASSERT_TRUE(pub_manager.wait_for_matched(topic.c_str()));

  auto & writer = recorder->get_writer_handle();
  MockSequentialWriter & mock_writer =
    static_cast<MockSequentialWriter &>(writer.get_implementation_handle());

  size_t expected_messages = num_latched_messages;
  auto ret = rosbag2_test_common::wait_until_shutdown(
    std::chrono::seconds(5),
    [&mock_writer, &expected_messages]() {
      return mock_writer.get_messages().size() >= expected_messages;
    });
  auto recorded_messages = mock_writer.get_messages();
  auto recorded_topics = mock_writer.get_topics();
  EXPECT_TRUE(ret) << "failed to capture expected messages in time";
  EXPECT_THAT(recorded_messages, SizeIs(expected_messages));
  EXPECT_EQ(recorded_topics.size(), 1u);
  EXPECT_FALSE(recorded_messages.empty());
}

TEST_F(RecordIntegrationTestFixture, mixed_qos_subscribes) {
  // Ensure that rosbag2 subscribes to publishers that offer different durability policies
  const size_t arbitrary_history = 5;

  std::string topic = "/string_topic";
  test_msgs::msg::Strings msg;
  msg.string_value = "Hello";

  auto profile_volatile = rclcpp::QoS(arbitrary_history).reliable().durability_volatile();
  auto profile_transient_local = rclcpp::QoS(arbitrary_history).reliable().transient_local();

  auto publisher_node = std::make_shared<rclcpp::Node>("rosbag2_test_record_5");
  auto publisher_volatile = publisher_node->create_publisher<test_msgs::msg::Strings>(
    topic, profile_volatile);
  auto publisher_transient_local = publisher_node->create_publisher<test_msgs::msg::Strings>(
    topic, profile_transient_local);

  rosbag2_transport::RecordOptions record_options = {false, false, {topic}, "rmw_format", 100ms};
  auto recorder = std::make_shared<rosbag2_transport::Recorder>(
    std::move(writer_), storage_options_, record_options);
  recorder->record();

  // Takes ~100ms in local testing, 5s chosen as a very long timeout
  bool succeeded = rosbag2_test_common::spin_and_wait_for(
    std::chrono::seconds(5), recorder,
    [publisher_volatile, publisher_transient_local]() {
      // This test is a success if rosbag2 has connected to both publishers
      return
      publisher_volatile->get_subscription_count() &&
      publisher_transient_local->get_subscription_count();
    });
  ASSERT_TRUE(succeeded);
}

TEST_F(RecordIntegrationTestFixture, duration_and_noncompatibility_policies_mixed) {
  // Ensure that the duration-based and non-compatibility QoS policies don't affect subscription
  // These values are arbitrary, the significance is that they are non-default
  const std::string topic = "/mixed_nondelivery_policies";
  const size_t same_history = 5;
  const size_t different_history = 12;
  const rmw_time_t deadline{0, 1000};
  const rmw_time_t lifespan{3, 12};
  const auto liveliness = RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC;
  const rmw_time_t liveliness_lease_duration{0, 5000000};

  auto publisher_node = std::make_shared<rclcpp::Node>("rosbag2_test_record_6");
  auto create_pub = [publisher_node, topic](auto qos) {
      return publisher_node->create_publisher<test_msgs::msg::Strings>(topic, qos);
    };

  auto profile_history = rclcpp::QoS(different_history);
  auto publisher_history = create_pub(profile_history);

  auto profile_lifespan = rclcpp::QoS(same_history).lifespan(lifespan);
  auto publisher_lifespan = create_pub(profile_lifespan);

  auto profile_deadline = rclcpp::QoS(same_history).deadline(deadline);
  auto publisher_deadline = create_pub(profile_deadline);

  auto profile_liveliness = rclcpp::QoS(same_history)
    .liveliness(liveliness).liveliness_lease_duration(liveliness_lease_duration);
  auto publisher_liveliness = create_pub(profile_liveliness);

  rosbag2_transport::RecordOptions record_options = {false, false, {topic}, "rmw_format", 100ms};
  auto recorder = std::make_shared<rosbag2_transport::Recorder>(
    std::move(writer_), storage_options_, record_options);
  recorder->record();

  // Takes ~200ms in local testing, 5s chosen as a very long timeout
  bool succeeded = rosbag2_test_common::spin_and_wait_for(
    std::chrono::seconds(5), recorder,
    [publisher_history, publisher_lifespan, publisher_deadline, publisher_liveliness]() {
      return
      publisher_history->get_subscription_count() &&
      publisher_lifespan->get_subscription_count() &&
      publisher_deadline->get_subscription_count() &&
      publisher_liveliness->get_subscription_count();
    });
  ASSERT_TRUE(succeeded);
}

TEST_F(RecordIntegrationTestFixture, write_split_callback_is_called)
{
  auto string_message = get_messages_strings()[1];
  std::string string_topic = "/string_topic";

  bool callback_called = false;
  std::string closed_file, opened_file;
  rosbag2_cpp::bag_events::WriterEventCallbacks callbacks;
  callbacks.write_split_callback =
    [&callback_called, &closed_file, &opened_file](rosbag2_cpp::bag_events::BagSplitInfo & info) {
      closed_file = info.closed_file;
      opened_file = info.opened_file;
      callback_called = true;
    };
  writer_->add_event_callbacks(callbacks);

  auto & mock_writer = dynamic_cast<MockSequentialWriter &>(writer_->get_implementation_handle());
  mock_writer.set_max_messages_per_file(5);

  rosbag2_transport::RecordOptions record_options =
  {false, false, {string_topic}, "rmw_format", 100ms};
  auto recorder = std::make_shared<rosbag2_transport::Recorder>(
    std::move(writer_), storage_options_, record_options);
  recorder->record();

  start_async_spin(recorder);

  auto & writer = recorder->get_writer_handle();
  mock_writer = dynamic_cast<MockSequentialWriter &>(writer.get_implementation_handle());

  const size_t expected_messages = mock_writer.max_messages_per_file() + 1;

  rosbag2_test_common::PublicationManager pub_manager;
  pub_manager.setup_publisher(string_topic, string_message, expected_messages);
  ASSERT_TRUE(pub_manager.wait_for_matched(string_topic.c_str()));
  pub_manager.run_publishers();

  auto ret = rosbag2_test_common::wait_until_shutdown(
    std::chrono::seconds(5),
    [&mock_writer, &expected_messages]() {
      return mock_writer.get_messages().size() >= expected_messages;
    });
  auto recorded_messages = mock_writer.get_messages();
  EXPECT_TRUE(ret) << "failed to capture expected messages in time";
  EXPECT_THAT(recorded_messages, SizeIs(expected_messages));

  // Confirm that the callback was called and the file names have been sent with the event
  ASSERT_TRUE(callback_called);
  EXPECT_EQ(closed_file, "BagFile0");
  EXPECT_EQ(opened_file, "BagFile1");
}
