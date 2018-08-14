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

#ifndef ROSBAG2_STORAGE_DEFAULT_PLUGINS__SQLITE__SQLITE_WRAPPER_HPP_
#define ROSBAG2_STORAGE_DEFAULT_PLUGINS__SQLITE__SQLITE_WRAPPER_HPP_

#include <sqlite3.h>
#include <stdexcept>
#include <string>
#include <vector>

#include "rcutils/types.h"
#include "rosbag2_storage/serialized_bag_message.hpp"

namespace rosbag2_storage_plugins
{

class SqliteException : public std::runtime_error
{
public:
  explicit SqliteException(const std::string & message)
  : runtime_error(message) {}
};

using DBPtr = sqlite3 *;

class SqliteWrapper
{
public:
  explicit SqliteWrapper(const std::string & uri);
  SqliteWrapper();
  virtual ~SqliteWrapper();

  virtual void execute_query(
    const std::string & query,
    int (* callback)(void *, int, char **, char **),
    void * first_callback_argument);

  virtual rosbag2_storage::SerializedBagMessage get_message(size_t index);

  virtual void write_stamped_char_array(
    char * buffer, size_t buffer_length, int64_t time_stamp);

  virtual operator bool();

private:
  DBPtr db_ptr;
};


}  // namespace rosbag2_storage_plugins

#endif  // ROSBAG2_STORAGE_DEFAULT_PLUGINS__SQLITE__SQLITE_WRAPPER_HPP_
