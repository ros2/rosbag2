// Copyright 2021, Robotec.ai sp. z o.o.
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
#include <regex>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include "rosbag2_test_common/publication_manager.hpp"
#include "rosbag2_test_common/wait_for.hpp"

#include "rosbag2_transport/recorder.hpp"

#include "test_msgs/msg/arrays.hpp"
#include "test_msgs/msg/basic_types.hpp"
#include "test_msgs/message_fixtures.hpp"

#include "record_integration_fixture.hpp"

using namespace std::chrono_literals;  // NOLINT

TEST_F(RecordIntegrationTestFixture, regex_topics_recording)
{
  auto test_string_messages = get_messages_strings();
  auto test_array_messages = get_messages_arrays();
  std::string regex = "^/aa$";

  // matching topic
  std::string v1 = "/aa";

  // topics that shouldn't match
  std::string b1 = "/aaa";
  std::string b2 = "/baa";
  std::string b3 = "/baaa";
  std::string b4 = "/aa/aa";

  // checking the test data itself
  std::regex re(regex);
  ASSERT_TRUE(std::regex_match(v1, re));
  ASSERT_FALSE(std::regex_match(b1, re));
  ASSERT_FALSE(std::regex_match(b2, re));
  ASSERT_FALSE(std::regex_match(b3, re));
  ASSERT_FALSE(std::regex_match(b4, re));

  rosbag2_transport::RecordOptions record_options = {false, false, {}, "rmw_format", 10ms};
  record_options.regex = regex;

  // TODO(karsten1987) Refactor this into publication manager
  rosbag2_test_common::PublicationManager pub_manager;
  pub_manager.setup_publisher(v1, test_string_messages[0], 3);
  pub_manager.setup_publisher(b1, test_string_messages[0], 3);
  pub_manager.setup_publisher(b2, test_string_messages[1], 3);
  pub_manager.setup_publisher(b3, test_string_messages[0], 3);
  pub_manager.setup_publisher(b4, test_string_messages[1], 3);

  auto recorder = std::make_shared<rosbag2_transport::Recorder>(
    std::move(writer_), storage_options_, record_options);
  recorder->record();

  start_async_spin(recorder);

  ASSERT_TRUE(pub_manager.wait_for_matched(v1.c_str()));

  pub_manager.run_publishers();

  auto & writer = recorder->get_writer_handle();
  MockSequentialWriter & mock_writer =
    static_cast<MockSequentialWriter &>(writer.get_implementation_handle());

  size_t expected_messages = 3;
  auto ret = rosbag2_test_common::wait_until_shutdown(
    std::chrono::seconds(5),
    [&mock_writer, &expected_messages]() {
      return mock_writer.get_messages().size() >= expected_messages;
    });
  auto recorded_messages = mock_writer.get_messages();
  // We may receive additional messages from rosout, it doesn't matter,
  // as long as we have received at least as many total messages as we expect
  EXPECT_TRUE(ret) << "failed to capture expected messages in time";
  EXPECT_THAT(recorded_messages, SizeIs(Ge(expected_messages)));
  auto recorded_topics = mock_writer.get_topics();
  EXPECT_THAT(recorded_topics, SizeIs(1));
  EXPECT_TRUE(recorded_topics.find(v1) != recorded_topics.end());
}

TEST_F(RecordIntegrationTestFixture, regex_and_exclude_recording)
{
  auto test_string_messages = get_messages_strings();
  auto test_array_messages = get_messages_arrays();
  std::string regex = "/[a-z]+_nice(_.*)";
  std::string regex_exclude = "/[a-z]+_nice_[a-z]+/(.*)";

  // matching topics - the only ones that should be recorded
  std::string v1 = "/awesome_nice_topic";
  std::string v2 = "/still_nice_topic";

  // excluded topics
  std::string e1 = "/quite_nice_namespace/but_it_is_excluded";

  // topics that shouldn't match
  std::string b1 = "/numberslike1arenot_nice";
  std::string b2 = "/namespace_before/not_nice";

  // checking the test data itself
  std::regex re(regex);
  std::regex exclude(regex_exclude);
  ASSERT_TRUE(std::regex_match(v1, re));
  ASSERT_TRUE(std::regex_match(v2, re));
  ASSERT_FALSE(std::regex_match(b1, re));
  ASSERT_FALSE(std::regex_match(b2, re));

  // this example matches both regexes - should be excluded
  ASSERT_TRUE(std::regex_match(e1, re));
  ASSERT_TRUE(std::regex_match(e1, exclude));

  rosbag2_transport::RecordOptions record_options = {false, false, {}, "rmw_format", 10ms};
  record_options.regex = regex;
  record_options.exclude = regex_exclude;


  // TODO(karsten1987) Refactor this into publication manager
  rosbag2_test_common::PublicationManager pub_manager;
  pub_manager.setup_publisher(v1, test_string_messages[0], 3);
  pub_manager.setup_publisher(v2, test_string_messages[1], 3);
  pub_manager.setup_publisher(b1, test_string_messages[0], 3);
  pub_manager.setup_publisher(b2, test_string_messages[1], 3);
  pub_manager.setup_publisher(e1, test_string_messages[0], 3);

  auto recorder = std::make_shared<rosbag2_transport::Recorder>(
    std::move(writer_), storage_options_, record_options);
  recorder->record();

  start_async_spin(recorder);

  ASSERT_TRUE(pub_manager.wait_for_matched(v1.c_str()));
  ASSERT_TRUE(pub_manager.wait_for_matched(v2.c_str()));

  pub_manager.run_publishers();

  auto & writer = recorder->get_writer_handle();
  MockSequentialWriter & mock_writer =
    static_cast<MockSequentialWriter &>(writer.get_implementation_handle());

  size_t expected_messages = 3;
  auto ret = rosbag2_test_common::wait_until_shutdown(
    std::chrono::seconds(5),
    [&mock_writer, &expected_messages]() {
      return mock_writer.get_messages().size() >= expected_messages;
    });
  auto recorded_messages = mock_writer.get_messages();
  // We may receive additional messages from rosout, it doesn't matter,
  // as long as we have received at least as many total messages as we expect
  EXPECT_TRUE(ret) << "failed to capture expected messages in time";
  EXPECT_THAT(recorded_messages, SizeIs(Ge(expected_messages)));

  auto recorded_topics = mock_writer.get_topics();
  EXPECT_THAT(recorded_topics, SizeIs(2));
  EXPECT_TRUE(recorded_topics.find(v1) != recorded_topics.end());
  EXPECT_TRUE(recorded_topics.find(v2) != recorded_topics.end());
}
