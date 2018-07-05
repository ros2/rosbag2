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

using namespace ::testing;  // NOLINT
using namespace rosbag2;  // NOLINT

TEST(SqliteStorage, write_single_message_to_storage) {
  std::remove("test_database");
  SqliteStorage storage("test_database");
  storage.open();
  storage.insertMessage("test_message");
  storage.close();

  sqlite::DBPtr db = sqlite::open("test_database");
  auto messages = sqlite::getMessages(db);
  sqlite::close(db);

  ASSERT_EQ(messages.size(), size_t(1));
  EXPECT_EQ(messages[0], "test_message");
}
