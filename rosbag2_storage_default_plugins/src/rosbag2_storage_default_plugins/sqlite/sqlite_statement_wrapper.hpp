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

#include <stdexcept>
#include <string>
#include <utility>

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
  // default constructor needed for testing.
  SqliteStatementWrapper();
  SqliteStatementWrapper(sqlite3 * database, std::string query);
  virtual ~SqliteStatementWrapper();

  virtual sqlite3_stmt * get();
  virtual void step();
  virtual void step_next_row();
  virtual void bind_text(int column, char * buffer, size_t buffer_length);
  virtual void bind_int(int column, int64_t int_to_bind);

  virtual size_t read_text_size(int column);

  virtual void read_text(int column, char * buffer, size_t size);
  virtual int64_t read_int(int column);

private:
  sqlite3_stmt * statement_;
};

}  // namespace rosbag2_storage_plugins

#endif  // ROSBAG2_STORAGE_DEFAULT_PLUGINS__SQLITE__SQLITE_STATEMENT_WRAPPER_HPP_
