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

#ifndef ROSBAG2_STORAGE_DEFAULT_PLUGINS__SQLITE__MOCK_SQLITE_WRAPPER_HPP_
#define ROSBAG2_STORAGE_DEFAULT_PLUGINS__SQLITE__MOCK_SQLITE_WRAPPER_HPP_

#include <gmock/gmock.h>

#include <string>
#include <vector>

#include "../../../src/rosbag2_storage_default_plugins/sqlite/sqlite_wrapper.hpp"

class MockSqliteWrapper : public rosbag2_storage_plugins::SqliteWrapper
{
public:
  MOCK_METHOD1(execute_query, void(const std::string &));
  MOCK_METHOD2(get_message, bool(std::string &, size_t));
};

#endif  // ROSBAG2_STORAGE_DEFAULT_PLUGINS__SQLITE__MOCK_SQLITE_WRAPPER_HPP_
