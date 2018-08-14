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

#ifndef ROSBAG2_STORAGE_DEFAULT_PLUGINS__SQLITE__MOCK_SQLITE_WRAPPER_HPP_
#define ROSBAG2_STORAGE_DEFAULT_PLUGINS__SQLITE__MOCK_SQLITE_WRAPPER_HPP_

#include <gmock/gmock.h>

#include <string>
#include <vector>

#include "rosbag2_storage/serialized_bag_message.hpp"
#include "../../../src/rosbag2_storage_default_plugins/sqlite/sqlite_wrapper.hpp"

ACTION_P(SetSecondArgumentVoidPointerToInt, value)
{
  *static_cast<size_t *>(arg2) = *value;
}

class MockSqliteWrapper : public rosbag2_storage_plugins::SqliteWrapper
{
public:
  MOCK_METHOD3(
    execute_query,
    void(const std::string &, int (* callback)(void *, int, char **, char **), void *));
  MOCK_METHOD1(get_message, rosbag2_storage::SerializedBagMessage(size_t index));
  MOCK_METHOD3(
    write_stamped_char_array, void(char * buffer, size_t buffer_length, int64_t time_stamp));
};

#endif  // ROSBAG2_STORAGE_DEFAULT_PLUGINS__SQLITE__MOCK_SQLITE_WRAPPER_HPP_
