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

#ifndef ROSBAG2_STORAGE_DEFAULT_PLUGINS__SQLITE__MOCK_SQLITE_STATEMENT_WRAPPER_HPP_
#define ROSBAG2_STORAGE_DEFAULT_PLUGINS__SQLITE__MOCK_SQLITE_STATEMENT_WRAPPER_HPP_

#include <gmock/gmock.h>

#include <string>
#include <utility>

#include "../../../src/rosbag2_storage_default_plugins/sqlite/sqlite_statement_wrapper.hpp"

class MockSqliteStatementWrapper : public rosbag2_storage_plugins::SqliteStatementWrapper
{
public:
  MOCK_METHOD0(get, sqlite3_stmt * ());
  MOCK_METHOD0(step, void());
  MOCK_METHOD0(step_next_row, void());
  MOCK_METHOD3(bind_text, void(int column, char * buffer, size_t buffer_length));
  MOCK_METHOD2(bind_int, void(int column, int64_t int_to_bind));
  MOCK_METHOD1(read_text_size, size_t(int column));
  MOCK_METHOD3(read_text, void(int column, char * buffer, size_t size));
  MOCK_METHOD1(read_int, int64_t(int column));
};

#endif  // ROSBAG2_STORAGE_DEFAULT_PLUGINS__SQLITE__MOCK_SQLITE_STATEMENT_WRAPPER_HPP_
