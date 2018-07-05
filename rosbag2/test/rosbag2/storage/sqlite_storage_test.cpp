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
#include <memory>

#include "../../../src/rosbag2/storage/sqlite/sqlite_storage.hpp"

using namespace ::testing;
using namespace rosbag2;

TEST(SqliteStorage, write_single_message_to_storage) {
  std::remove("test_database");
  auto storage = std::make_shared<SqliteStorage>();
  storage->open("test_database");
  storage->insertMessage("test_message");

  storage.reset();
  sqlite::DBPtr db = sqlite::open("test_database");
  sqlite3_stmt * stmt;
  sqlite3_prepare_v2(db, "SELECT * FROM messages", -1, &stmt, nullptr);
  int result = sqlite3_step(stmt);
  std::string text;
  if (result == SQLITE_ROW) {
    text = std::string(reinterpret_cast<const char *>(sqlite3_column_text(stmt, 1)));
  }
  sqlite3_finalize(stmt);
  sqlite::close(db);

  EXPECT_EQ(text, "test_message");
}
