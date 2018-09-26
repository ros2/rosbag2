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

// The record_fixture.hpp must be included before process_execution_helpers.hpp
#include "record_fixture.hpp"
#include "process_execution_helpers.hpp"

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

  auto test_topic_messages = get_messages_for_topic<test_msgs::msg::Primitives>("/test_topic");
  EXPECT_THAT(test_topic_messages, SizeIs(Ge(expected_test_messages)));
  EXPECT_THAT(test_topic_messages,
    Each(Pointee(Field(&test_msgs::msg::Primitives::string_value, "test"))));

  auto wrong_topic_messages = get_messages_for_topic<test_msgs::msg::Primitives>("/wrong_topic");
  EXPECT_THAT(wrong_topic_messages, IsEmpty());
}
