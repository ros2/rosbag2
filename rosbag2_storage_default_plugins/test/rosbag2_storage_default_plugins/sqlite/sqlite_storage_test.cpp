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

#include <gmock/gmock.h>

#include <memory>
#include <string>
#include <vector>
#include <fstream>

#include "../../../src/rosbag2_storage_default_plugins/sqlite/sqlite_storage.hpp"
#include "mock_sqlite_wrapper.hpp"

using namespace ::testing;  // NOLINT
using namespace rosbag2_storage_plugins;  // NOLINT

TEST(SqliteStorageTest, write_single_message_to_storage) {
  auto sqlite_wrapper = std::make_shared<MockSqliteWrapper>();

  EXPECT_CALL(*sqlite_wrapper, write_blob(_));

  rosbag2_storage::SerializedBagMessage test_message;
  auto storage = std::make_unique<SqliteStorage>(sqlite_wrapper);
  storage->write(test_message);
  storage.reset();
}

TEST(SqliteStorageTest, read_messages_from_storage) {
  auto sqlite_wrapper = std::make_shared<NiceMock<MockSqliteWrapper>>();

  rosbag2_storage::SerializedBagMessage test_message;
  auto storage = std::make_unique<SqliteStorage>(sqlite_wrapper);
  storage->write(test_message);
  storage->write(test_message);

  EXPECT_CALL(*sqlite_wrapper, get_message(_, 0));
  EXPECT_CALL(*sqlite_wrapper, get_message(_, 1));

  storage->read_next();
  storage->read_next();
  storage.reset();
}
