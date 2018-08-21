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

#ifndef ROSBAG2_STORAGE_DEFAULT_PLUGINS__SQLITE__SQLITE_STATEMENT_WRAPPER_HPP_
#define ROSBAG2_STORAGE_DEFAULT_PLUGINS__SQLITE__SQLITE_STATEMENT_WRAPPER_HPP_

#include <sqlite3.h>

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rosbag2_storage/serialized_bag_message.hpp"

namespace rosbag2_storage_plugins
{

class SqliteException : public std::runtime_error
{
public:
  explicit SqliteException(const std::string & message)
  : runtime_error(message) {}
};

class SqliteStatementWrapper
{
public:
  SqliteStatementWrapper();
  SqliteStatementWrapper(sqlite3 * database, std::string query);
  virtual ~SqliteStatementWrapper();

  virtual sqlite3_stmt * get();
  virtual void execute_and_reset();
  virtual void advance_one_row();

  virtual void bind_table_entry(
    std::shared_ptr<rcutils_char_array_t> serialized_data, int64_t timestamp,
    int serialized_data_position_in_statement, int timestamp_position_in_statement);
  std::shared_ptr<rosbag2_storage::SerializedBagMessage>
  read_table_entry(int blob_column, int timestamp_column);


  virtual void reset();
  virtual bool is_prepared();

private:
  virtual void bind_serialized_data(
    int column, std::shared_ptr<rcutils_char_array_t> serialized_data);
  virtual void bind_timestamp(int column, int64_t timestamp);

  sqlite3_stmt * statement_;
  bool is_prepared_;
  std::vector<std::shared_ptr<rcutils_char_array_t>> cached_written_blobls_;
};

}  // namespace rosbag2_storage_plugins

#endif  // ROSBAG2_STORAGE_DEFAULT_PLUGINS__SQLITE__SQLITE_STATEMENT_WRAPPER_HPP_
