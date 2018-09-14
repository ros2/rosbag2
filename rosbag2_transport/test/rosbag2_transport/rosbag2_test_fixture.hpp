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

#ifndef ROSBAG2_TRANSPORT__ROSBAG2_TEST_FIXTURE_HPP_
#define ROSBAG2_TRANSPORT__ROSBAG2_TEST_FIXTURE_HPP_

#include <gtest/gtest.h>

#include <cstdio>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#ifdef _WIN32
# include <direct.h>
# include <Windows.h>
#endif

#include "rosbag2_transport/rosbag2_play_options.hpp"
#include "rosbag2_storage/storage_factory.hpp"
#include "rosbag2_storage/storage_interfaces/read_only_interface.hpp"
#include "rosbag2_storage/storage_interfaces/read_write_interface.hpp"
#include "test_memory_management.hpp"

using namespace ::testing;  // NOLINT

class Rosbag2TestFixture : public Test
{
public:
  Rosbag2TestFixture()
  : database_name_(std::string(UnitTest::GetInstance()->current_test_info()->name()) + ".db3")
  {
    std::string system_separator = "/";
#ifdef _WIN32
    system_separator = "\\";
#endif
    database_name_ = temporary_dir_path_ + system_separator + database_name_;
    std::cout << "Database name: " << database_name_ << std::endl;

    options_ = rosbag2_transport::Rosbag2PlayOptions();
    options_.read_ahead_queue_size = 1000;
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
    auto storage = factory.open_read_only(db_name, "sqlite3");
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
    const std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> & messages,
    const std::map<std::string, std::string> & topics_and_types)
  {
    rosbag2_storage::StorageFactory factory;
    auto storage = factory.open_read_write(db_name, "sqlite3");
    for (const auto & topic_and_type : topics_and_types) {
      storage->create_topic({topic_and_type.first, topic_and_type.second});
    }

    for (const auto & msg : messages) {
      storage->write(msg);
    }
  }

  template<typename MessageT>
  std::shared_ptr<rosbag2_storage::SerializedBagMessage> serialize_message(
    const std::string & topic, typename MessageT::_data_type message)
  {
    auto msg = std::make_shared<MessageT>();
    msg->data = message;
    auto bag_msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
    bag_msg->serialized_data = memory_management_.serialize_message(msg);
    bag_msg->topic_name = topic;

    return bag_msg;
  }

  template<typename MessageT>
  std::shared_ptr<rosbag2_storage::SerializedBagMessage> serialize_test_message(
    const std::string & topic, std::shared_ptr<MessageT> message)
  {
    auto bag_msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
    bag_msg->serialized_data = memory_management_.serialize_message(message);
    bag_msg->topic_name = topic;

    return bag_msg;
  }

  std::string database_name_;
  static std::string temporary_dir_path_;
  test_helpers::TestMemoryManagement memory_management_;
  rosbag2_transport::Rosbag2PlayOptions options_;
};

std::string Rosbag2TestFixture::temporary_dir_path_ = "";  // NOLINT

#endif  // ROSBAG2_TRANSPORT__ROSBAG2_TEST_FIXTURE_HPP_
