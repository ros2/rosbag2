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
#include <iostream>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "rcutils/logging_macros.h"
#include "rcutils/snprintf.h"
#include "rosbag2_storage_default_plugins/sqlite/sqlite_storage.hpp"
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

  std::shared_ptr<rcutils_char_array_t> make_serialized_message(std::string message)
  {
    int message_size = get_buffer_capacity(message);
    message_size++;  // need to account for terminating null character
    assert(message_size > 0);

    auto msg = new rcutils_char_array_t;
    *msg = rcutils_get_zero_initialized_char_array();
    auto ret = rcutils_char_array_init(msg, message_size, &allocator_);
    if (ret != RCUTILS_RET_OK) {
      throw std::runtime_error("Error allocating resources " + std::to_string(ret));
    }

    auto serialized_data = std::shared_ptr<rcutils_char_array_t>(msg,
        [](rcutils_char_array_t * msg) {
          int error = rcutils_char_array_fini(msg);
          delete msg;
          if (error != RCUTILS_RET_OK) {
            RCUTILS_LOG_ERROR_NAMED(
              "rosbag2_storage_default_plugins", "Leaking memory %i", error);
          }
        });

    serialized_data->buffer_length = message_size;
    int written_size = write_data_to_serialized_string_message(
      serialized_data->buffer, serialized_data->buffer_capacity, message);

    assert(written_size == message_size - 1);  // terminated null character not counted
    return serialized_data;
  }

  std::string deserialize_message(std::shared_ptr<rcutils_char_array_t> serialized_message)
  {
    char * copied = new char[serialized_message->buffer_length];
    auto string_length = serialized_message->buffer_length - 8;
    memcpy(copied, &serialized_message->buffer[8], string_length);
    std::string message_content(copied);
    // cppcheck-suppress mismatchAllocDealloc ; complains about "copied" but used new[] and delete[]
    delete[] copied;
    return message_content;
  }

  void write_messages_to_sqlite(
    std::vector<std::tuple<std::string, int64_t, std::string, std::string>> messages)
  {
    std::unique_ptr<rosbag2_storage::storage_interfaces::ReadWriteInterface> writable_storage =
      std::make_unique<rosbag2_storage_plugins::SqliteStorage>();

    writable_storage->open(temporary_dir_path_);

    for (auto msg : messages) {
      std::string topic_name = std::get<2>(msg);
      std::string type_name = std::get<3>(msg);
      writable_storage->create_topic({topic_name, type_name});
      auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
      bag_message->serialized_data = make_serialized_message(std::get<0>(msg));
      bag_message->time_stamp = std::get<1>(msg);
      bag_message->topic_name = topic_name;
      writable_storage->write(bag_message);
    }
  }

  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>>
  read_all_messages_from_sqlite()
  {
    std::unique_ptr<rosbag2_storage::storage_interfaces::ReadOnlyInterface> readable_storage =
      std::make_unique<rosbag2_storage_plugins::SqliteStorage>();
    readable_storage->open(
      temporary_dir_path_, rosbag2_storage::storage_interfaces::IOFlag::READ_ONLY);
    std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> read_messages;

    while (readable_storage->has_next()) {
      read_messages.push_back(readable_storage->read_next());
    }

    return read_messages;
  }

protected:
  int get_buffer_capacity(const std::string & message)
  {
    return write_data_to_serialized_string_message(nullptr, 0, message);
  }

  int write_data_to_serialized_string_message(
    char * buffer, size_t buffer_capacity, const std::string & message)
  {
    // This function also writes the final null charachter, which is absent in the CDR format.
    // Here this behaviour is ok, because we only test test writing and reading from/to sqlite.
    return rcutils_snprintf(buffer,
             buffer_capacity,
             "%c%c%c%c%c%c%c%c%s",
             0x00, 0x01, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00,
             message.c_str());
  }

  rcutils_allocator_t allocator_;
};

#endif  // ROSBAG2_STORAGE_DEFAULT_PLUGINS__SQLITE__STORAGE_TEST_FIXTURE_HPP_
