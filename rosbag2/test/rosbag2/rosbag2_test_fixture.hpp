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

#ifndef ROSBAG2__ROSBAG2_TEST_FIXTURE_HPP_
#define ROSBAG2__ROSBAG2_TEST_FIXTURE_HPP_

#include <gtest/gtest.h>

#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "std_msgs/msg/string.hpp"

#ifdef _WIN32
# include <direct.h>
# include <Windows.h>
#endif

#include "rosbag2_storage/storage_interfaces/read_only_interface.hpp"
#include "rosbag2_storage/storage_interfaces/read_write_interface.hpp"
#include "rosbag2_storage/storage_factory.hpp"

using namespace ::testing;  // NOLINT

class Rosbag2TestFixture : public Test
{
public:
  Rosbag2TestFixture()
  : database_name_(UnitTest::GetInstance()->current_test_info()->name())
  {
    std::string system_separator = "/";
#ifdef _WIN32
    system_separator = "\\";
#endif
    database_name_ = temporary_dir_path_ + system_separator + database_name_;
    std::cout << "Database name: " << database_name_ << std::endl;
  }

  ~Rosbag2TestFixture() override
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

  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>>
  get_messages(const std::string & db_name)
  {
    std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> table_msgs;
    rosbag2_storage::StorageFactory factory;
    auto storage =
      factory.open_read_only(db_name, "sqlite3");
    if (storage == nullptr) {
      throw std::runtime_error("failed to open sqlite3 storage");
    }

    while (storage->has_next()) {
      table_msgs.push_back(storage->read_next());
    }

    return table_msgs;
  }

  void write_messages(
    const std::string & db_name,
    std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages)
  {
    rosbag2_storage::StorageFactory factory;
    auto storage =
      factory.open_read_write(db_name, "sqlite3");
    if (storage == nullptr) {
      throw std::runtime_error("failed to open sqlite3 storage");
    }

    if (storage) {
      for (auto msg : messages) {
        auto ser_msg = make_serialized_message(msg);
        rosbag2_storage::SerializedBagMessage bag_message;
        bag_message.serialized_data = ser_msg;
        // TODO(Martin-Idel-SI): Make time stamp
        storage->write(bag_message);
      }
    }
  }

  std::shared_ptr<rcutils_char_array_t> make_serialized_message(
    std::string message)
  {
    std_msgs::msg::String::SharedPtr test_message;
    test_message->data = message;

    auto serialized_test_message = std::shared_ptr<rcutils_char_array_t>();
    *serialized_test_message = rmw_get_zero_initialized_serialized_message();
    auto allocator = rcutils_get_default_allocator();
    auto initial_capacity = 8u + static_cast<size_t>(test_message->data.size());
    auto error = rmw_serialized_message_init(
      serialized_test_message.get(),
      initial_capacity,
      &allocator);
    if (error != RCL_RET_OK) {
      throw std::runtime_error("Something went wrong preparing the serialized message");
    }

    auto string_ts =
      rosidl_typesupport_cpp::get_message_type_support_handle<std_msgs::msg::String>();

    error = rmw_serialize(test_message.get(), string_ts, serialized_test_message.get());
    if (error != RMW_RET_OK) {
      throw std::runtime_error("Something went wrong preparing the serialized message");
    }
    return serialized_test_message;
  }

  std::string database_name_;
  static std::string temporary_dir_path_;
};

std::string Rosbag2TestFixture::temporary_dir_path_ = "";  // NOLINT

#endif  // ROSBAG2__ROSBAG2_TEST_FIXTURE_HPP_
