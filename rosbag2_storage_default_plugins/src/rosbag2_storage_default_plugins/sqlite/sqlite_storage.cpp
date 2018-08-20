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

#include "sqlite_storage.hpp"

#include <cstring>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rcutils/logging_macros.h"
#include "rosbag2_storage/serialized_bag_message.hpp"

namespace rosbag2_storage_plugins
{
const char * ROS_PACKAGE_NAME = "rosbag2_storage_default_plugins";

SqliteStorage::SqliteStorage()
: database_(), counter_(0), bag_info_()
{}

SqliteStorage::SqliteStorage(std::shared_ptr<SqliteWrapper> database)
: database_(std::move(database)), counter_(0)
{}

void SqliteStorage::open(
  const std::string & uri, rosbag2_storage::storage_interfaces::IOFlag io_flag)
{
  (void) io_flag;
  try {
    database_ = std::make_unique<SqliteWrapper>(uri);
    bag_info_.uri = uri;
  } catch (const SqliteException & e) {
    throw std::runtime_error("Failed to setup storage. Error: " + std::string(e.what()));
  }

  RCUTILS_LOG_INFO_NAMED(ROS_PACKAGE_NAME, "Opened database '%s'.", uri.c_str());
}

void SqliteStorage::write(std::shared_ptr<const rosbag2_storage::SerializedBagMessage> message)
{
  // TODO(Martin-Idel-SI) The real serialized string message has 8 leading chars in CDR
  std::string msg(&message->serialized_data->buffer[8]);
  std::string insert_message =
    "INSERT INTO messages (data, timestamp) VALUES ('" + msg + "', strftime('%s%f','now'))";
  database_->execute_query(insert_message);

  RCUTILS_LOG_INFO_NAMED(ROS_PACKAGE_NAME, "Stored message");
}

bool SqliteStorage::has_next() const
{
  // TODO(Martin-Idel-SI): improve sqlite_wrapper interface
  std::string message;
  return database_->get_message(message, counter_);
}

std::shared_ptr<rosbag2_storage::SerializedBagMessage> SqliteStorage::read_next()
{
  // TODO(Martin-Idel-SI): improve sqlite_wrapper interface
  std::string message;
  database_->get_message(message, counter_++);
  auto msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  auto payload = new rcutils_char_array_t;
  *payload = rcutils_get_zero_initialized_char_array();
  payload->allocator = rcutils_get_default_allocator();
  auto ret = rcutils_char_array_resize(payload, strlen("Hello World") + 1);
  if (ret != RCUTILS_RET_OK) {
    RCUTILS_LOG_ERROR_NAMED("rosbag2_storage_default_plugins",
      " Failed to destroy serialized bag message");
  }
  strcpy(payload->buffer, message.c_str());  // NOLINT cpplint doesn't like strcpy here

  msg->serialized_data = std::shared_ptr<rcutils_char_array_t>(payload,
      [](rcutils_char_array_t * msg) {
        auto error = rcutils_char_array_fini(msg);
        delete msg;
        if (error != RCUTILS_RET_OK) {
          RCUTILS_LOG_ERROR_NAMED("rosbag2_storage_default_plugins",
          " Failed to destroy serialized bag message");
        }
      });
  return msg;
}

void SqliteStorage::initialize()
{
  std::string create_table = "CREATE TABLE messages(" \
    "id INTEGER PRIMARY KEY AUTOINCREMENT," \
    "data           BLOB    NOT NULL," \
    "timestamp      INT     NOT NULL);";

  database_->execute_query(create_table);
}

rosbag2_storage::BagInfo SqliteStorage::info()
{
  return bag_info_;
}

void SqliteStorage::create_topic()
{
  initialize();
}

}  // namespace rosbag2_storage_plugins


#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(rosbag2_storage_plugins::SqliteStorage,
  rosbag2_storage::storage_interfaces::ReadWriteInterface)
