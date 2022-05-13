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
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "rosbag2_test_common/publication_manager.hpp"
#include "rosbag2_test_common/wait_for.hpp"

#include "rosbag2_transport/recorder.hpp"

#include "test_msgs/msg/basic_types.hpp"
#include "test_msgs/message_fixtures.hpp"

#include "record_integration_fixture.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "mock_recorder.hpp"

TEST_F(RecordIntegrationTestFixture, record_all_with_sim_time)
{
  std::string string_topic = "/string_topic";
  auto string_message = get_messages_strings()[0];
  string_message->string_value = "Hello World";

  // clock
  std::string clock_topic = "/clock";
  auto clock_message = std::make_shared<rosgraph_msgs::msg::Clock>();
  clock_message->clock.sec = 1234567890;
  clock_message->clock.nanosec = 123456789;

  rosbag2_test_common::PublicationManager pub_manager;
  pub_manager.setup_publisher(clock_topic, clock_message, 5);
  pub_manager.setup_publisher(string_topic, string_message, 5);

  rosbag2_transport::RecordOptions record_options =
  {
    false, false, {string_topic, clock_topic}, "rmw_format", 100ms
  };
  record_options.use_sim_time = true;
  auto recorder = std::make_shared<MockRecorder>(
    std::move(writer_), storage_options_, record_options);
  recorder->record();

  start_async_spin(recorder);

  ASSERT_TRUE(pub_manager.wait_for_matched(clock_topic.c_str()));
  ASSERT_TRUE(pub_manager.wait_for_matched(string_topic.c_str()));

  ASSERT_TRUE(recorder->wait_for_topic_to_be_discovered(string_topic));
  ASSERT_TRUE(recorder->wait_for_topic_to_be_discovered(clock_topic));

  ASSERT_TRUE(recorder->topic_available_for_recording(string_topic));
  ASSERT_TRUE(recorder->topic_available_for_recording(clock_topic));

  pub_manager.run_publishers();

  auto & writer = recorder->get_writer_handle();
  MockSequentialWriter & mock_writer =
    static_cast<MockSequentialWriter &>(writer.get_implementation_handle());

  size_t expected_messages = 10;
  auto ret = rosbag2_test_common::wait_until_shutdown(
    std::chrono::seconds(10),
    [&mock_writer, &expected_messages]() {
      return mock_writer.get_messages().size() >= expected_messages;
    });
  auto recorded_messages = mock_writer.get_messages();
  EXPECT_TRUE(ret) << "failed to capture expected messages in time. " <<
    "recorded messages = " << recorded_messages.size();
  stop_spinning();

  EXPECT_THAT(recorded_messages, SizeIs(Ge(expected_messages)));

  std::vector<rosbag2_storage::SerializedBagMessageSharedPtr> string_messages;
  for (const auto & message : recorded_messages) {
    if (message->topic_name == string_topic) {
      string_messages.push_back(message);
    }
  }
  // check that the timestamp is same as the clock message
  EXPECT_THAT(string_messages[0]->time_stamp, 1234567890123456789);
}
