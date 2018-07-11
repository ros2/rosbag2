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
#include <gmock/gmock.h>
#include <memory>
#include <string>
#include <vector>
#include <fstream>

#include "../../../src/rosbag2/storage/sqlite/sqlite_storage.hpp"
#include "mock_sqlite_wrapper.hpp"
#include "../rosbag2_test_fixture.hpp"

using namespace ::testing;  // NOLINT
using namespace rosbag2;  // NOLINT

TEST_F(Rosbag2TestFixture, write_single_message_to_storage) {
  auto sqlite_wrapper = std::make_shared<MockSqliteWrapper>();
  std::string message_to_write = "test_message";
  std::string query =
    "INSERT INTO messages (data, timestamp) VALUES ('" + message_to_write +
    "', strftime('%s%f','now'))";

  EXPECT_CALL(*sqlite_wrapper, execute_query(query));

  auto storage = std::make_unique<SqliteStorage>(sqlite_wrapper);
  storage->write(message_to_write);
  storage.reset();
}
