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

#include <memory>
#include <future>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "test_msgs/msg/arrays.hpp"
#include "test_msgs/msg/basic_types.hpp"
#include "test_msgs/message_fixtures.hpp"
#include "rosbag2_storage/filesystem_helper.hpp"
#include "rosbag2_storage_default_plugins/sqlite/sqlite_storage.hpp"
#include "rosbag2_test_common/temporary_directory_fixture.hpp"
#include "rosbag2_test_common/publisher_manager.hpp"
#include "rosbag2_test_common/memory_management.hpp"

using namespace ::testing;  // NOLINT
using namespace std::chrono_literals;  // NOLINT
using namespace rosbag2_test_common;  // NOLINT

class RecordFixture : public TemporaryDirectoryFixture
{
public:
  RecordFixture()
  {
    bag_path_ = rosbag2_storage::FilesystemHelper::concat({temporary_dir_path_, "bag"});
    storage_path_ = rosbag2_storage::FilesystemHelper::concat({bag_path_, "bag_0"});
    database_path_ = storage_path_ + ".db3";
    std::cout << "Database " << database_path_ << " in " << temporary_dir_path_ << std::endl;
  }

  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  void wait_for_db()
  {
    while (true) {
      try {
        std::this_thread::sleep_for(50ms);  // wait a bit to not query constantly
        rosbag2_storage_plugins::SqliteWrapper
          db(database_path_, rosbag2_storage::storage_interfaces::IOFlag::READ_ONLY);
        return;
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
    storage->open(storage_path_, rosbag2_storage::storage_interfaces::IOFlag::READ_ONLY);

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

  std::string bag_path_;
  std::string database_path_;
  std::string storage_path_;
  PublisherManager pub_man_;
  MemoryManagement memory_management_;
};

#endif  // ROSBAG2_TESTS__RECORD_FIXTURE_HPP_
