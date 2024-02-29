// Copyright 2024, Open Source Robotics Corporation.
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
#include "rosbag2_test_common/client_manager.hpp"

#include "rosbag2_transport/recorder.hpp"

#include "test_msgs/msg/arrays.hpp"
#include "test_msgs/msg/basic_types.hpp"
#include "test_msgs/message_fixtures.hpp"
#include "test_msgs/srv/basic_types.hpp"

#include "record_integration_fixture.hpp"

using namespace std::chrono_literals;  // NOLINT

TEST_F(RecordIntegrationTestFixture, regex_topics_recording)
{
  auto array_message = get_messages_arrays()[0];
  array_message->float32_values = {{40.0f, 2.0f, 0.0f}};
  array_message->bool_values = {{true, false, true}};
  std::string array_topic = "/array_topic";

  auto string_message = get_messages_strings()[0];
  string_message->string_value = "Hello World";
  std::string string_topic = "/string_topic";

  rosbag2_test_common::PublicationManager pub_manager;
  pub_manager.setup_publisher(array_topic, array_message, 2);
  pub_manager.setup_publisher(string_topic, string_message, 2);

  rosbag2_transport::RecordOptions record_options =
  {true, false, false, {}, {"test_msgs/msg/Arrays"}, {}, {}, {}, "rmw_format", 100ms};

  auto recorder = std::make_shared<rosbag2_transport::Recorder>(
    std::move(writer_), storage_options_, record_options);
  recorder->record();

  start_async_spin(recorder);

  ASSERT_TRUE(pub_manager.wait_for_matched(array_topic.c_str()));
  ASSERT_FALSE(pub_manager.wait_for_matched(string_topic.c_str()));

  pub_manager.run_publishers();

  auto & writer = recorder->get_writer_handle();
  auto & mock_writer = dynamic_cast<MockSequentialWriter &>(writer.get_implementation_handle());

  size_t expected_messages = 2;
  auto ret = rosbag2_test_common::wait_until_shutdown(
    std::chrono::seconds(5),
    [&mock_writer, &expected_messages]() {
      return mock_writer.get_messages().size() >= expected_messages;
    });
  EXPECT_TRUE(ret) << "failed to capture expected messages in time";
  auto recorded_messages = mock_writer.get_messages();
  EXPECT_EQ(recorded_messages.size(), expected_messages);
  auto recorded_topics = mock_writer.get_topics();
  EXPECT_THAT(recorded_topics, SizeIs(1));
  EXPECT_TRUE(recorded_topics.find(array_topic) != recorded_topics.end());
  EXPECT_FALSE(recorded_topics.find(string_topic) != recorded_topics.end());
}
