/*
 *  Copyright (c) 2018,  Bosch Software Innovations GmbH.
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */

#include <gtest/gtest.h>
#include <sqlite3.h>

#include <string>
#include <vector>

using namespace ::testing;  // NOLINT

class Rosbag2TestFixture : public Test
{
public:
  Rosbag2TestFixture()
  : database_name_(UnitTest::GetInstance()->current_test_info()->name())
  {}

  ~Rosbag2TestFixture() override
  {
    std::remove(database_name_.c_str());
  }

  std::vector<std::string> getMessages(sqlite3 * db, std::string table = "messages")
  {
    std::vector<std::string> table_msgs;
    sqlite3_stmt * statement;
    std::string query = "SELECT * FROM " + table;
    sqlite3_prepare_v2(db, query.c_str(), -1, &statement, nullptr);
    int result = sqlite3_step(statement);
    while (result == SQLITE_ROW) {
      table_msgs.emplace_back(
        std::string(reinterpret_cast<const char *>(sqlite3_column_text(statement, 1))));
      result = sqlite3_step(statement);
    }
    sqlite3_finalize(statement);

    return table_msgs;
  }

  std::string database_name_;
};
