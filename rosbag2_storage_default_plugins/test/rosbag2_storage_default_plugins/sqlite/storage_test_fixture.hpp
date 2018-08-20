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
#include <vector>

#ifdef _WIN32
# include <direct.h>
# include <Windows.h>
#endif

#include "../../../src/rosbag2_storage_default_plugins/sqlite/sqlite_storage.hpp"

using namespace ::testing;  // NOLINT

class StorageTestFixture : public Test
{
public:
  StorageTestFixture()
  : database_name_(UnitTest::GetInstance()->current_test_info()->name())
  {
    std::string system_separator = "/";
#ifdef _WIN32
    system_separator = "\\";
#endif
    database_name_ = temporary_dir_path_ + system_separator + database_name_;
    std::cout << "Database name: " << database_name_ << std::endl;
  }

  ~StorageTestFixture() override
  {
#ifdef _WIN32
    DeleteFileA(database_name_.c_str());
#else
// TODO(botteroa-si): once filesystem::remove_all() can be used, this line can be removed and
// the ful directory can be deleted in remove_temporary_dir()
    remove(database_name_.c_str());
#endif
  }

  static void SetUpTestCase()
  {
    char template_char[] = "tmp_test_dir.XXXXXX";
#ifdef _WIN32
    char temp_path[255];
    GetTempPathA(255, temp_path);
    _mktemp_s(template_char, strnlen(template_char, 20) + 1);
    temporary_dir_path_ = std::string(temp_path) + std::string(template_char);
    _mkdir(temporary_dir_path_.c_str());
#else
    char * dir_name = mkdtemp(template_char);
    temporary_dir_path_ = dir_name;
#endif
  }

  static void TearDownTestCase()
  {
    remove_temporary_dir();
  }

  static void remove_temporary_dir()
  {
#ifdef _WIN32
    // TODO(botteroa-si): find a way to delete a not empty directory in Windows, so that we don't
    // need the Windows line in the fixture destructor anymore.
    RemoveDirectoryA(temporary_dir_path_.c_str());
#else
    remove(temporary_dir_path_.c_str());
#endif
  }

  void write_messages_to_sqlite(std::vector<std::string> messages)
  {
    std::unique_ptr<rosbag2_storage::storage_interfaces::ReadWriteInterface> writable_storage =
      std::make_unique<rosbag2_storage_plugins::SqliteStorage>();
    writable_storage->open(database_name_);
    writable_storage->create_topic();

    for (auto message : messages) {
      auto msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
      auto payload = new rcutils_char_array_t;
      *payload = rcutils_get_zero_initialized_char_array();
      payload->allocator = rcutils_get_default_allocator();
      auto ret = rcutils_char_array_resize(payload, strlen(message.c_str()));
      if (ret != RCUTILS_RET_OK) {
        FAIL() << " Failed to resize serialized bag message";
      }
      strcpy(payload->buffer, "Hello World");  // NOLINT cpplint doesn't like strcpy here

      msg->serialized_data = std::shared_ptr<rcutils_char_array_t>(payload,
          [](rcutils_char_array_t * msg) {
            auto error = rcutils_char_array_fini(msg);
            delete msg;
            if (error != RCUTILS_RET_OK) {
              FAIL() << " Failed to destroy serialized bag message";
            }
          });
      writable_storage->write(msg);
    }
  }

  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>>
  read_all_messages_from_sqlite()
  {
    std::unique_ptr<rosbag2_storage::storage_interfaces::ReadWriteInterface> readable_storage =
      std::make_unique<rosbag2_storage_plugins::SqliteStorage>();
    readable_storage->open(database_name_);
    std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> read_messages;

    std::string message;
    while (readable_storage->has_next()) {
      read_messages.emplace_back(readable_storage->read_next());
    }

    return read_messages;
  }

  std::string database_name_;
  static std::string temporary_dir_path_;
};

std::string StorageTestFixture::temporary_dir_path_ = "";  // NOLINT

#endif  // ROSBAG2_STORAGE_DEFAULT_PLUGINS__SQLITE__STORAGE_TEST_FIXTURE_HPP_
