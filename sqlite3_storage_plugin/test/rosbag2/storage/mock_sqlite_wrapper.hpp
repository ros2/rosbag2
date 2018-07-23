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

#ifndef ROSBAG2__STORAGE__MOCK_SQLITE_WRAPPER_HPP_
#define ROSBAG2__STORAGE__MOCK_SQLITE_WRAPPER_HPP_

#include <gmock/gmock.h>

#include <string>
#include <vector>

#include "../../../src/rosbag2/storage/sqlite/sqlite_wrapper.hpp"

class MockSqliteWrapper : public rosbag2::SqliteWrapper
{
public:
  MOCK_METHOD1(execute_query, void(const std::string &));
  MOCK_METHOD0(get_messages, std::vector<std::string>());
};

#endif  // ROSBAG2__STORAGE__MOCK_SQLITE_WRAPPER_HPP_
