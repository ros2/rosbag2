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

#ifndef ROSBAG2_STORAGE_DEFAULT_PLUGINS__SQLITE__STORAGE_TEST_FIXTURE_HPP_
#define ROSBAG2_STORAGE_DEFAULT_PLUGINS__SQLITE__STORAGE_TEST_FIXTURE_HPP_

#include <gtest/gtest.h>

#include <cstdio>
#include <cstring>
#include <iostream>
#include <fstream>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>
#include <unordered_map>

#include "rcpputils/filesystem_helper.hpp"

#include "rcutils/logging_macros.h"
#include "rcutils/snprintf.h"

#include "rosbag2_storage/metadata_io.hpp"

#include "rosbag2_storage_default_plugins/sqlite/sqlite_storage.hpp"
#include "rosbag2_storage_default_plugins/sqlite/sqlite_wrapper.hpp"

#include "rosbag2_test_common/temporary_directory_fixture.hpp"

using namespace ::testing;  // NOLINT
using namespace rosbag2_test_common; // NOLINT

class StorageTestFixture : public TemporaryDirectoryFixture
{
public:
  StorageTestFixture()
  {
    allocator_ = rcutils_get_default_allocator();
  }

  std::shared_ptr<rcutils_uint8_array_t> make_serialized_message(const std::string & message)
  {
    size_t message_size = get_buffer_capacity(message);

    auto msg = new rcutils_uint8_array_t;
    *msg = rcutils_get_zero_initialized_uint8_array();
    auto ret = rcutils_uint8_array_init(msg, message_size, &allocator_);
    if (ret != RCUTILS_RET_OK) {
      delete msg;
      throw std::runtime_error("Error allocating resources " + std::to_string(ret));
    }

    auto serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
      msg,
      [](rcutils_uint8_array_t * msg) {
        int error = rcutils_uint8_array_fini(msg);
        delete msg;
        if (error != RCUTILS_RET_OK) {
          RCUTILS_LOG_ERROR_NAMED(
            "rosbag2_storage_default_plugins", "Leaking memory %i", error);
        }
      });

    serialized_data->buffer_length = message_size;
    write_data_to_serialized_string_message(
      serialized_data->buffer, serialized_data->buffer_capacity, message);

    return serialized_data;
  }

  std::string deserialize_message(std::shared_ptr<rcutils_uint8_array_t> serialized_message)
  {
    size_t preamble_len = this->get_preamble().size();
    assert(serialized_message->buffer_length > preamble_len);
    size_t amount_to_read = serialized_message->buffer_length - preamble_len;
    return std::string(
      reinterpret_cast<char *>(&serialized_message->buffer[preamble_len]),
      amount_to_read);
  }

  std::shared_ptr<rosbag2_storage_plugins::SqliteStorage>
  write_messages_to_sqlite(
    std::vector<std::tuple<std::string, int64_t, std::string, std::string, std::string>> messages,
    std::shared_ptr<rosbag2_storage_plugins::SqliteStorage> writable_storage = nullptr)
  {
    if (nullptr == writable_storage) {
      writable_storage = std::make_shared<rosbag2_storage_plugins::SqliteStorage>();

      auto db_file = (rcpputils::fs::path(temporary_dir_path_) / "rosbag").string();

      writable_storage->open({db_file, plugin_id_});
    }

    rosbag2_storage::storage_interfaces::ReadWriteInterface & rw_storage = *writable_storage;

    for (auto msg : messages) {
      std::string topic_name = std::get<2>(msg);
      std::string type_name = std::get<3>(msg);
      std::string rmw_format = std::get<4>(msg);
      rw_storage.create_topic({topic_name, type_name, rmw_format, ""});
      auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
      bag_message->serialized_data = make_serialized_message(std::get<0>(msg));
      bag_message->time_stamp = std::get<1>(msg);
      bag_message->topic_name = topic_name;
      rw_storage.write(bag_message);
    }

    metadata_io_.write_metadata(temporary_dir_path_, rw_storage.get_metadata());

    return writable_storage;
  }

  void write_messages_to_sqlite_in_pre_foxy_format(
    const std::vector<
      std::tuple<std::string, int64_t, std::string, std::string, std::string>
    > & messages)
  {
    auto db_file = (rcpputils::fs::path(temporary_dir_path_) / "rosbag").string();
    std::string relative_path = db_file + ".db3";

    // READ_WRITE requires the DB to not exist.
    if (rcpputils::fs::path(relative_path).exists()) {
      throw std::runtime_error(
              "Failed to create bag: File '" + relative_path + "' already exists!");
    }
    using rosbag2_storage_plugins::SqliteWrapper;
    using rosbag2_storage_plugins::SqliteException;
    std::unordered_map<std::string, std::string> pragmas;

    std::shared_ptr<SqliteWrapper> database;
    try {
      database = std::make_unique<SqliteWrapper>(
        relative_path,
        rosbag2_storage::storage_interfaces::IOFlag::READ_WRITE,
        std::move(pragmas));
    } catch (const SqliteException & e) {
      throw std::runtime_error("Failed to setup storage. Error: " + std::string(e.what()));
    }

    // Init database
    std::string create_stmt = "CREATE TABLE topics(" \
      "id INTEGER PRIMARY KEY," \
      "name TEXT NOT NULL," \
      "type TEXT NOT NULL," \
      "serialization_format TEXT NOT NULL);";
    database->prepare_statement(create_stmt)->execute_and_reset();
    create_stmt = "CREATE TABLE messages(" \
      "id INTEGER PRIMARY KEY," \
      "topic_id INTEGER NOT NULL," \
      "timestamp INTEGER NOT NULL, " \
      "data BLOB NOT NULL);";
    database->prepare_statement(create_stmt)->execute_and_reset();
    create_stmt = "CREATE INDEX timestamp_idx ON messages (timestamp ASC);";
    database->prepare_statement(create_stmt)->execute_and_reset();

    std::unordered_map<std::string, int> topics;

    using SqliteStatement = std::shared_ptr<rosbag2_storage_plugins::SqliteStatementWrapper>;
    SqliteStatement write_statement = database->prepare_statement(
      "INSERT INTO messages (timestamp, topic_id, data) VALUES (?, ?, ?);");

    for (const auto & msg : messages) {
      std::string topic_name = std::get<2>(msg);
      std::string type_name = std::get<3>(msg);
      std::string rmw_format = std::get<4>(msg);

      // Create topic in DB if message with new topic name
      if (topics.find(topic_name) == std::end(topics)) {
        auto insert_topic = database->prepare_statement(
          "INSERT INTO topics (name, type, serialization_format) VALUES (?, ?, ?)");
        insert_topic->bind(topic_name, type_name, rmw_format);
        insert_topic->execute_and_reset();
        topics.emplace(topic_name, static_cast<int>(database->get_last_insert_id()));
      }

      // Prepare rosbag2 serialized message to write
      auto message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
      message->serialized_data = make_serialized_message(std::get<0>(msg));
      message->time_stamp = std::get<1>(msg);
      message->topic_name = topic_name;
      // Write message to DB
      auto topic_entry = topics.find(topic_name);
      if (topic_entry == end(topics)) {
        throw SqliteException("Topic '" + topic_name + "' has not been created yet!");
      }
      write_statement->bind(message->time_stamp, topic_entry->second, message->serialized_data);
      write_statement->execute_and_reset();
    }
  }

  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>>
  read_all_messages_from_sqlite()
  {
    std::unique_ptr<rosbag2_storage::storage_interfaces::ReadOnlyInterface> readable_storage =
      std::make_unique<rosbag2_storage_plugins::SqliteStorage>();

    auto db_file = (rcpputils::fs::path(temporary_dir_path_) / "rosbag.db3").string();

    readable_storage->open(
      {db_file, plugin_id_},
      rosbag2_storage::storage_interfaces::IOFlag::READ_ONLY);
    std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> read_messages;

    while (readable_storage->has_next()) {
      read_messages.push_back(readable_storage->read_next());
    }

    return read_messages;
  }

  rosbag2_storage::StorageOptions make_storage_options_with_config(
    const std::string & config_yaml,
    const std::string & plugin_id)
  {
    auto temp_dir = rcpputils::fs::path(temporary_dir_path_);
    const auto storage_uri = (temp_dir / "rosbag").string();
    const auto yaml_config = (temp_dir / "sqlite_config.yaml").string();

    { // populate temporary config file
      std::ofstream out(yaml_config);
      out << config_yaml;
    }

    rosbag2_storage::StorageOptions storage_options{storage_uri, plugin_id, 0, 0, 0,
      "", yaml_config};
    return storage_options;
  }

protected:
  std::string get_preamble() const
  {
    return {0x00, 0x01, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00};
  }

  size_t get_buffer_capacity(const std::string & message)
  {
    return get_preamble().size() + message.size();
  }

  size_t write_data_to_serialized_string_message(
    uint8_t * buffer, size_t buffer_capacity, const std::string & message)
  {
    size_t remaining_buffer = buffer_capacity;
    size_t amount_written = 0;
    if (remaining_buffer == 0) {
      return amount_written;
    }

    std::string preamble = get_preamble();
    assert(buffer_capacity >= preamble.size() + message.size());

    size_t amount_to_write = remaining_buffer;
    if (preamble.size() < amount_to_write) {
      amount_to_write = preamble.size();
    }
    std::memcpy(buffer, preamble.data(), amount_to_write);
    amount_written += amount_to_write;
    remaining_buffer -= amount_written;
    if (remaining_buffer == 0) {
      return amount_written;
    }

    amount_to_write = remaining_buffer;
    if (message.size() < amount_to_write) {
      amount_to_write = message.size();
    }
    std::memcpy(buffer + amount_written, message.data(), amount_to_write);
    amount_written += amount_to_write;

    return amount_written;
  }

  rcutils_allocator_t allocator_;
  rosbag2_storage::MetadataIo metadata_io_;

  const std::string plugin_id_ = "sqlite3";
};

#endif  // ROSBAG2_STORAGE_DEFAULT_PLUGINS__SQLITE__STORAGE_TEST_FIXTURE_HPP_
