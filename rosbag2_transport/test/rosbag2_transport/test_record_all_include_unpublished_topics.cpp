// Copyright 2022 Seegrid Corporation. All Rights Reserved.
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
#include <memory>
#include <string>
#include <utility>

#include "test_msgs/msg/basic_types.hpp"
#include "test_msgs/message_fixtures.hpp"

#include "rosbag2_test_common/publication_manager.hpp"
#include "rosbag2_test_common/wait_for.hpp"

#include "rosbag2_transport/recorder.hpp"

#include "record_integration_fixture.hpp"

using namespace std::chrono_literals;  // NOLINT

TEST_F(RecordIntegrationTestFixture, record_all_include_unpublished_false_ignores_unpublished)
{
  const std::string string_topic = "/string_topic";
  auto node = std::make_shared<rclcpp::Node>("test_string_msg_listener_node");
  auto string_msgs_sub = node->create_subscription<test_msgs::msg::Strings>(
    string_topic, 10, [](test_msgs::msg::Strings::ConstSharedPtr) {});

  rosbag2_transport::RecordOptions record_options = {true, false, {}, "rmw_format", 100ms};
  record_options.include_unpublished_topics = false;
  auto recorder = std::make_shared<rosbag2_transport::Recorder>(
    std::move(writer_), storage_options_, record_options);
  recorder->record();

  start_async_spin(recorder);

  auto & writer = recorder->get_writer_handle();
  MockSequentialWriter & mock_writer =
    static_cast<MockSequentialWriter &>(writer.get_implementation_handle());

  rosbag2_test_common::wait_until_shutdown(
    std::chrono::seconds(2),
    [&mock_writer]() {
      // Return false so we wait the full two seconds
      return false;
    });

  auto recorded_topics = mock_writer.get_topics();
  EXPECT_EQ(0u, recorded_topics.count(string_topic));
}

TEST_F(RecordIntegrationTestFixture, record_all_include_unpublished_true_includes_unpublished)
{
  const std::string string_topic = "/string_topic";
  auto node = std::make_shared<rclcpp::Node>("test_string_msg_listener_node");
  auto string_msgs_sub = node->create_subscription<test_msgs::msg::Strings>(
    string_topic, 10, [](test_msgs::msg::Strings::ConstSharedPtr) {});

  rosbag2_transport::RecordOptions record_options = {true, false, {}, "rmw_format", 100ms};
  record_options.include_unpublished_topics = true;
  auto recorder = std::make_shared<rosbag2_transport::Recorder>(
    std::move(writer_), storage_options_, record_options);
  recorder->record();

  start_async_spin(recorder);

  auto & writer = recorder->get_writer_handle();
  MockSequentialWriter & mock_writer =
    static_cast<MockSequentialWriter &>(writer.get_implementation_handle());

  rosbag2_test_common::wait_until_shutdown(
    std::chrono::seconds(2),
    [&mock_writer]() {
      // Return false so we wait the full two seconds
      return false;
    });

  auto recorded_topics = mock_writer.get_topics();
  EXPECT_EQ(1u, recorded_topics.count(string_topic));
}

TEST_F(
  RecordIntegrationTestFixture,
  record_all_include_unpublished_false_includes_later_published_default_qos)
{
  const std::string string_topic = "/string_topic";
  auto node = std::make_shared<rclcpp::Node>("test_string_msg_listener_node");
  auto string_msgs_sub = node->create_subscription<test_msgs::msg::Strings>(
    string_topic, 10, [](test_msgs::msg::Strings::ConstSharedPtr) {});

  rosbag2_transport::RecordOptions record_options = {true, false, {}, "rmw_format", 100ms};
  record_options.include_unpublished_topics = false;
  auto recorder = std::make_shared<rosbag2_transport::Recorder>(
    std::move(writer_), storage_options_, record_options);
  recorder->record();

  start_async_spin(recorder);

  // Start up a publisher on our topic *after* the recording has started
  auto string_message = get_messages_strings()[0];
  string_message->string_value = "Hello World";
  rosbag2_test_common::PublicationManager pub_manager;

  // Publish 40 messages at a 50ms interval for a steady 2 seconds worth of data
  pub_manager.setup_publisher(
    string_topic, string_message, 40, rclcpp::QoS{rclcpp::KeepAll()}, 50ms);

  ASSERT_TRUE(pub_manager.wait_for_matched(string_topic.c_str()));

  pub_manager.run_publishers();

  auto & writer = recorder->get_writer_handle();
  MockSequentialWriter & mock_writer =
    static_cast<MockSequentialWriter &>(writer.get_implementation_handle());

  rosbag2_test_common::wait_until_shutdown(
    std::chrono::seconds(2),
    [&mock_writer]() {
      // Return false so we wait the full two seconds
      return false;
    });
  auto recorded_messages = mock_writer.get_messages();
  auto recorded_topics = mock_writer.get_topics();
  EXPECT_EQ(1u, recorded_topics.count(string_topic));

  // We expect to drop some messages due to the discovery process, but there
  // should be at least one message received in the duration of this test.
  auto string_messages = filter_messages<test_msgs::msg::Strings>(
    recorded_messages, string_topic);
  ASSERT_THAT(string_messages, SizeIs(Ge(1u)));
  EXPECT_THAT(string_messages[0]->string_value, Eq("Hello World"));
}

TEST_F(
  RecordIntegrationTestFixture,
  record_all_include_unpublished_false_includes_later_published_sensor_data_qos)
{
  const std::string string_topic = "/string_topic";
  auto node = std::make_shared<rclcpp::Node>("test_string_msg_listener_node");
  auto string_msgs_sub = node->create_subscription<test_msgs::msg::Strings>(
    string_topic, 10, [](test_msgs::msg::Strings::ConstSharedPtr) {});

  rosbag2_transport::RecordOptions record_options = {true, false, {}, "rmw_format", 100ms};
  record_options.include_unpublished_topics = false;
  auto recorder = std::make_shared<rosbag2_transport::Recorder>(
    std::move(writer_), storage_options_, record_options);
  recorder->record();

  start_async_spin(recorder);

  // Start up a publisher on our topic *after* the recording has started
  auto string_message = get_messages_strings()[0];
  string_message->string_value = "Hello World";
  rosbag2_test_common::PublicationManager pub_manager;

  // Publish 40 messages at a 50ms interval for a steady 2 seconds worth of data
  pub_manager.setup_publisher(string_topic, string_message, 40, rclcpp::SensorDataQoS(), 50ms);

  ASSERT_TRUE(pub_manager.wait_for_matched(string_topic.c_str()));

  pub_manager.run_publishers();

  auto & writer = recorder->get_writer_handle();
  MockSequentialWriter & mock_writer =
    static_cast<MockSequentialWriter &>(writer.get_implementation_handle());

  rosbag2_test_common::wait_until_shutdown(
    std::chrono::seconds(2),
    [&mock_writer]() {
      // Return false so we wait the full two seconds
      return false;
    });

  auto recorded_topics = mock_writer.get_topics();
  EXPECT_EQ(1u, recorded_topics.count(string_topic));

  // We expect to drop some messages due to the discovery process, but there
  // should be at least one message received in the duration of this test.
  auto recorded_messages = mock_writer.get_messages();
  auto string_messages = filter_messages<test_msgs::msg::Strings>(
    recorded_messages, string_topic);
  ASSERT_THAT(string_messages, SizeIs(Ge(1u)));
  EXPECT_THAT(string_messages[0]->string_value, Eq("Hello World"));
}
