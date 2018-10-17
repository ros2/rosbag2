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

#include <string>

// rclcpp.hpp included in record_fixture.hpp must be included before process_execution_helpers.hpp
#include "record_fixture.hpp"
#include "process_execution_helpers.hpp"
#include "rosbag2_storage/metadata_io.hpp"

TEST_F(RecordFixture, record_end_to_end_test) {
  auto message = get_messages_primitives()[0];
  message->string_value = "test";
  size_t expected_test_messages = 3;
  pub_man_.add_publisher("/test_topic", message, expected_test_messages);

  auto wrong_message = get_messages_primitives()[0];
  wrong_message->string_value = "wrong_content";
  pub_man_.add_publisher("/wrong_topic", wrong_message);

  auto process_handle = start_execution("ros2 bag record --output " + bag_path_ + " /test_topic");
  wait_for_db();

  rosbag2_storage_plugins::SqliteWrapper
    db(database_path_, rosbag2_storage::storage_interfaces::IOFlag::READ_ONLY);
  pub_man_.run_publishers([this, &db](const std::string & topic_name) {
      return count_stored_messages(db, topic_name);
    });

  stop_execution(process_handle);

  // TODO(Martin-Idel-SI): Find out how to correctly send a Ctrl-C signal on Windows
  // This is necessary as the process is killed hard on Windows and doesn't write a metadata file
#ifdef _WIN32
  rosbag2_storage::BagMetadata metadata{};
  metadata.version = 1;
  metadata.storage_identifier = "sqlite3";
  metadata.storage_format = "cdr";
  metadata.relative_file_paths.emplace_back("bag.db3");
  metadata.duration = std::chrono::nanoseconds(0);
  metadata.starting_time =
    std::chrono::time_point<std::chrono::high_resolution_clock>(std::chrono::nanoseconds(0));
  metadata.message_count = 0;
  rosbag2_storage::MetadataIo metadata_io;
  metadata_io.write_metadata(bag_path_, metadata);
#endif

  auto test_topic_messages = get_messages_for_topic<test_msgs::msg::Primitives>("/test_topic");
  EXPECT_THAT(test_topic_messages, SizeIs(Ge(expected_test_messages)));
  EXPECT_THAT(test_topic_messages,
    Each(Pointee(Field(&test_msgs::msg::Primitives::string_value, "test"))));

  auto wrong_topic_messages = get_messages_for_topic<test_msgs::msg::Primitives>("/wrong_topic");
  EXPECT_THAT(wrong_topic_messages, IsEmpty());
}

TEST_F(RecordFixture, record_fails_gracefully_if_bag_already_exists) {
  auto database_path = _SRC_RESOURCES_DIR_PATH;  // variable defined in CMakeLists.txt

  internal::CaptureStderr();
  auto exit_code =
    execute_and_wait_until_completion("ros2 bag record --output test -a", database_path);
  auto error_output = internal::GetCapturedStderr();

  EXPECT_THAT(exit_code, Eq(EXIT_FAILURE));
  EXPECT_THAT(error_output, HasSubstr("Output folder 'test' already exists"));
}

TEST_F(RecordFixture, record_fails_if_both_all_and_topic_list_is_specified) {
  internal::CaptureStderr();
  auto exit_code =
    execute_and_wait_until_completion("ros2 bag record -a /some_topic", temporary_dir_path_);
  auto error_output = internal::GetCapturedStderr();

  EXPECT_THAT(exit_code, Eq(EXIT_FAILURE));
  EXPECT_THAT(error_output, HasSubstr("Can not specify topics and -a at the same time."));
}
