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

TEST_F(RecordIntegrationTestFixture, record_all_without_discovery_ignores_later_announced_topics)
{
  auto topic = "/string_topic";
  auto string_message = get_messages_strings()[0];
  string_message->string_value = "Hello World";

  rosbag2_transport::RecordOptions record_options = {true, true, {}, "rmw_format", 100ms};
  auto recorder = std::make_shared<rosbag2_transport::Recorder>(
    std::move(writer_), storage_options_, record_options);
  recorder->record();

  start_async_spin(recorder);

  rosbag2_test_common::PublicationManager pub_manager;
  pub_manager.setup_publisher(topic, string_message, 5);
  pub_manager.run_publishers();

  auto & writer = recorder->get_writer_handle();
  MockSequentialWriter & mock_writer =
    static_cast<MockSequentialWriter &>(writer.get_implementation_handle());

  size_t expected_messages = 0;
  rosbag2_test_common::wait_until_shutdown(
    std::chrono::seconds(2),
    [&mock_writer, &expected_messages]() {
      return mock_writer.get_messages().size() > expected_messages;
    });
  (void) expected_messages;  // we can't say anything here, there might be some rosout

  auto recorded_topics = mock_writer.get_topics();
  EXPECT_EQ(0u, recorded_topics.count(topic));
}
