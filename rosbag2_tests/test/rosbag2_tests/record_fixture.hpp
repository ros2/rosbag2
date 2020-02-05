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

#ifndef ROSBAG2_TESTS__RECORD_FIXTURE_HPP_
#define ROSBAG2_TESTS__RECORD_FIXTURE_HPP_

#include <gmock/gmock.h>

#include <future>
#include <memory>
#include <string>
#include <vector>

#include "rcpputils/filesystem_helper.hpp"

#include "rclcpp/rclcpp.hpp"

#include "rosbag2_storage_default_plugins/sqlite/sqlite_storage.hpp"

#include "rosbag2_test_common/temporary_directory_fixture.hpp"
#include "rosbag2_test_common/publisher_manager.hpp"
#include "rosbag2_test_common/memory_management.hpp"

#include "test_msgs/msg/arrays.hpp"
#include "test_msgs/msg/basic_types.hpp"
#include "test_msgs/message_fixtures.hpp"

using namespace ::testing;  // NOLINT
using namespace std::chrono_literals;  // NOLINT
using namespace rosbag2_test_common;  // NOLINT

class RecordFixture : public TemporaryDirectoryFixture
{
public:
  RecordFixture() = default;

  void SetUp() override
  {
    root_bag_path_ = rcpputils::fs::path(temporary_dir_path_) / get_test_name();

    // Clean up potentially leftover bag files.
    // There may be leftovers if the system reallocates a temp directory
    // used by a previous test execution and the test did not have a clean exit.
    if (root_bag_path_.exists()) {
      remove_directory_recursively(root_bag_path_.string());
    }
  }

  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  std::string get_test_name() const
  {
    const auto * test_info = UnitTest::GetInstance()->current_test_info();

    return test_info->name();
  }

  std::string get_bag_file_name(int split_index) const
  {
    std::stringstream bag_file_name;
    bag_file_name << get_test_name() << "_" << split_index;

    return bag_file_name.str();
  }

  rcpputils::fs::path get_bag_file_path(int split_index)
  {
    return root_bag_path_ / (get_bag_file_name(split_index) + ".db3");
  }

  void wait_for_metadata(int timeout_in_sec = 5)
  {
    const auto metadata_path = rcpputils::fs::path{root_bag_path_} / "metadata.yaml";
    const auto start_time = std::chrono::steady_clock::now();

    while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(timeout_in_sec) &&
      rclcpp::ok())
    {
      if (metadata_path.exists()) {
        return;
      }
      std::this_thread::sleep_for(50ms);
    }
    // Final check for metadata if we timeout. Fail otherwise
    ASSERT_EQ(metadata_path.exists(), true) << "Could not find metadata.yaml";
  }

  void wait_for_db()
  {
    const auto database_path = get_bag_file_path(0);

    while (true) {
      try {
        std::this_thread::sleep_for(50ms);  // wait a bit to not query constantly
        if (database_path.exists()) {
          rosbag2_storage_plugins::SqliteWrapper db{
            database_path.string(), rosbag2_storage::storage_interfaces::IOFlag::READ_ONLY};
          return;
        }
      } catch (const rosbag2_storage_plugins::SqliteException & ex) {
        (void) ex;  // still waiting
      }
    }
  }

  size_t count_stored_messages(
    rosbag2_storage_plugins::SqliteWrapper & db, const std::string & topic_name)
  {
    // protect against concurrent writes (from recording) that may make the count query throw.
    while (true) {
      try {
        return count_stored_messages_in_db(db, topic_name);
      } catch (const rosbag2_storage_plugins::SqliteException & e) {
        (void) e;
      }
      std::this_thread::sleep_for(50ms);
    }
  }

  size_t count_stored_messages_in_db(
    rosbag2_storage_plugins::SqliteWrapper & db, const std::string & topic_name)
  {
    auto row = db.prepare_statement(
      "SELECT COUNT(*) FROM sqlite_master WHERE type='table' AND name = 'topics';")
      ->execute_query<int>().get_single_line();
    if (std::get<0>(row) == 0) {
      return 0;
    }
    auto message_count = db.prepare_statement(
      "SELECT COUNT(*) "
      "FROM messages LEFT JOIN topics ON messages.topic_id = topics.id "
      "WHERE topics.name = ?;")->bind(topic_name)->execute_query<int>().get_single_line();
    return static_cast<size_t>(std::get<0>(message_count));
  }

  template<typename MessageT>
  std::vector<std::shared_ptr<MessageT>> get_messages_for_topic(const std::string & topic)
  {
    auto all_messages = get_messages();
    auto topic_messages = std::vector<std::shared_ptr<MessageT>>();
    for (const auto & msg : all_messages) {
      if (msg->topic_name == topic) {
        topic_messages.push_back(
          memory_management_.deserialize_message<MessageT>(msg->serialized_data));
      }
    }
    return topic_messages;
  }

  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> get_messages()
  {
    std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> table_msgs;
    auto storage = std::make_shared<rosbag2_storage_plugins::SqliteStorage>();
    const auto database_path = get_bag_file_path(0).string();
    storage->open(database_path, rosbag2_storage::storage_interfaces::IOFlag::READ_ONLY);

    while (storage->has_next()) {
      table_msgs.push_back(storage->read_next());
    }

    return table_msgs;
  }

  std::string get_rwm_format_for_topic(
    const std::string & topic_name, rosbag2_storage_plugins::SqliteWrapper & db)
  {
    auto topic_format = db.prepare_statement(
      "SELECT serialization_format "
      "FROM topics "
      "WHERE name = ?;")->bind(topic_name)->execute_query<std::string>().get_single_line();
    return std::get<0>(topic_format);
  }

  // relative path to the root of the bag file.
  rcpputils::fs::path root_bag_path_;

  PublisherManager pub_man_;
  MemoryManagement memory_management_;
};

#endif  // ROSBAG2_TESTS__RECORD_FIXTURE_HPP_
