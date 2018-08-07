/*
 *  Copyright (c) 2018, Bosch Software Innovations GmbH.
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

#include "storage_test_fixture.hpp"

using namespace ::testing;  // NOLINT

TEST_F(StorageTestFixture, string_messages_are_written_and_read_to_and_from_sqlite3_storage) {
  std::vector<std::string> string_messages = {"test_message 1", "test_message 2", "test_message 3"};

  write_messages_to_sqlite(string_messages);
  auto read_messages = read_all_messages_from_sqlite();

  ASSERT_THAT(read_messages, SizeIs(3));
  EXPECT_THAT(read_messages[0], Eq(string_messages[0]));
  EXPECT_THAT(read_messages[1], Eq(string_messages[1]));
  EXPECT_THAT(read_messages[2], Eq(string_messages[2]));
}
