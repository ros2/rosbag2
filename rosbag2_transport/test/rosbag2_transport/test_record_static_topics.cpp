// Copyright 2025, Apex.AI
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

#include <filesystem>
#include <memory>
#include <utility>

#include "record_integration_fixture.hpp"
#include "mock_recorder.hpp"
#include "rosbag2_test_common/publication_manager.hpp"
#include "test_msgs/message_fixtures.hpp"

using namespace ::testing;  // NOLINT
namespace fs = std::filesystem;

TEST_F(RecordIntegrationTestFixture, recorder_can_read_static_topics_list)
{
  rosbag2_transport::RecordOptions record_options = rosbag2_transport::RecordOptions{
    true, true, true, true, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, "rmw_format", 100ms};

  // _SRC_RESOURCES_DIR_PATH defined in CMakeLists.txt
  record_options.static_topics_uri =
    fs::absolute(fs::path(_SRC_RESOURCES_DIR_PATH) / "static_topics_list.yaml").generic_string();
  auto recorder = std::make_shared<MockRecorder>(
    std::move(writer_), storage_options_, record_options);

  auto static_topics = recorder->get_static_topics();

  ASSERT_EQ(static_topics.size(), 3U);

  EXPECT_EQ(static_topics[0].first, "/topic_name1");
  EXPECT_EQ(static_topics[0].second, "test_msgs/msg/Strings");

  EXPECT_EQ(static_topics[2].first, "/topic_name3");
  EXPECT_EQ(static_topics[2].second, "test_msgs/msg/Strings");
}

TEST_F(RecordIntegrationTestFixture, exclude_applies_above_static_topics_list)
{
  auto string_message = get_messages_strings()[0];
  string_message->string_value = "Hello World";
  rosbag2_test_common::PublicationManager pub_manager;
  pub_manager.setup_publisher("/topic_name1", string_message, 5);
  rosbag2_transport::RecordOptions record_options = rosbag2_transport::RecordOptions{
    false, false, false, true, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, "rmw_format", 100ms};
  record_options.include_unpublished_topics = true;
  record_options.exclude_regex = ".*_name2.*";

  // _SRC_RESOURCES_DIR_PATH defined in CMakeLists.txt
  record_options.static_topics_uri =
    fs::absolute(fs::path(_SRC_RESOURCES_DIR_PATH) / "static_topics_list.yaml").generic_string();
  auto recorder = std::make_shared<MockRecorder>(
    std::move(writer_), storage_options_, record_options);

  recorder->record();
  // Wait until the recorder will subscribe to the publisher's topic to make sure that
  // recorder::record(..) in another thread got to the topics filter call
  ASSERT_TRUE(pub_manager.wait_for_matched("/topic_name1"));
  recorder->stop();

  EXPECT_TRUE(recorder->topic_available_for_recording("/topic_name1"));
  EXPECT_FALSE(recorder->topic_available_for_recording("/topic_name2"));
  EXPECT_TRUE(recorder->topic_available_for_recording("/topic_name3"));
}
