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

#include "mock_recorder.hpp"
#include "test_msgs/msg/basic_types.hpp"
#include "test_msgs/message_fixtures.hpp"
#include "rosbag2_test_common/publication_manager.hpp"
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
  auto recorder = std::make_shared<MockRecorder>(writer_, storage_options_, record_options);
  recorder->record();
  start_async_spin(recorder);

  ASSERT_TRUE(recorder->wait_for_topic_to_be_discovered(string_topic));
  ASSERT_FALSE(recorder->topic_available_for_recording(string_topic));
}

TEST_F(RecordIntegrationTestFixture, record_all_include_unpublished_true_includes_unpublished)
{
  const std::string string_topic = "/string_topic";
  auto node = std::make_shared<rclcpp::Node>("test_string_msg_listener_node");
  auto string_msgs_sub = node->create_subscription<test_msgs::msg::Strings>(
    string_topic, 10, [](test_msgs::msg::Strings::ConstSharedPtr) {});

  rosbag2_transport::RecordOptions record_options = {true, false, {}, "rmw_format", 100ms};
  record_options.include_unpublished_topics = true;
  auto recorder = std::make_shared<MockRecorder>(writer_, storage_options_, record_options);
  recorder->record();
  start_async_spin(recorder);

  ASSERT_TRUE(recorder->wait_for_topic_to_be_discovered(string_topic));
  ASSERT_TRUE(recorder->topic_available_for_recording(string_topic));
}

TEST_F(
  RecordIntegrationTestFixture,
  record_all_include_unpublished_false_includes_later_published)
{
  const std::string string_topic = "/string_topic";
  auto node = std::make_shared<rclcpp::Node>("test_string_msg_listener_node");
  auto string_msgs_sub = node->create_subscription<test_msgs::msg::Strings>(
    string_topic, 10, [](test_msgs::msg::Strings::ConstSharedPtr) {});

  rosbag2_transport::RecordOptions record_options = {true, false, {}, "rmw_format", 100ms};
  record_options.include_unpublished_topics = false;
  auto recorder = std::make_shared<MockRecorder>(writer_, storage_options_, record_options);
  recorder->record();
  start_async_spin(recorder);

  ASSERT_TRUE(recorder->wait_for_topic_to_be_discovered(string_topic));
  ASSERT_FALSE(recorder->topic_available_for_recording(string_topic));

  // Start up a publisher on our topic *after* the recording has started
  auto string_message = get_messages_strings()[0];
  string_message->string_value = "Hello World";
  rosbag2_test_common::PublicationManager pub_manager;

  // Publish 10 messages at a 30ms interval for a steady 300 milliseconds worth of data
  pub_manager.setup_publisher(
    string_topic, string_message, 10, rclcpp::QoS{rclcpp::KeepAll()}, 30ms);

  ASSERT_TRUE(pub_manager.wait_for_matched(string_topic.c_str()));
  pub_manager.run_publishers();

  ASSERT_TRUE(recorder->topic_available_for_recording(string_topic));
}
