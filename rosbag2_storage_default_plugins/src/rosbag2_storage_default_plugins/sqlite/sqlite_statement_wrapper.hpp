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

  template<typename T1, typename T2, typename ... Params>
  void bind(T1 value1, T2 value2, Params ... values);
  void bind(int value);
  void bind(rcutils_time_point_value_t value);
  void bind(double value);
  void bind(std::string value);
  void bind(std::shared_ptr<rcutils_char_array_t> value);

  std::shared_ptr<rosbag2_storage::SerializedBagMessage>
  read_table_entry(int blob_column, int timestamp_column);

  virtual void reset();
  virtual bool is_prepared();

private:
  template<typename T>
  void check_and_report_bind_error(int return_code, T value);
  void check_and_report_bind_error(int return_code);

  sqlite3_stmt * statement_;
  bool is_prepared_;
  int last_bound_parameter_index_;
  std::vector<std::shared_ptr<rcutils_char_array_t>> written_blobs_cache_;
};

template<typename T1, typename T2, typename ... Params>
inline
void SqliteStatementWrapper::bind(T1 value1, T2 value2, Params ... values)
{
  bind(value1);
  bind(value2, values ...);
}

template<>
inline
void SqliteStatementWrapper::check_and_report_bind_error(int return_code, std::string value)
{
  if (return_code != SQLITE_OK) {
    throw SqliteException("SQLite error when binding parameter " +
            std::to_string(last_bound_parameter_index_) + " to value '" + value +
            "'. Return code: " + std::to_string(return_code));
  }
}

template<typename T>
inline
void SqliteStatementWrapper::check_and_report_bind_error(int return_code, T value)
{
  check_and_report_bind_error(return_code, std::to_string(value));
}

}  // namespace rosbag2_storage_plugins

#endif  // ROSBAG2_STORAGE_DEFAULT_PLUGINS__SQLITE__SQLITE_STATEMENT_WRAPPER_HPP_
