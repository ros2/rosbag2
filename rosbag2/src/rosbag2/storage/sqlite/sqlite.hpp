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

#ifndef ROSBAG2__STORAGE__SQLITE__SQLITE_HPP_
#define ROSBAG2__STORAGE__SQLITE__SQLITE_HPP_

#include <sqlite3.h>
#include <stdexcept>
#include <string>
#include <vector>

namespace rosbag2
{
namespace sqlite
{

class sql_exception: public std::runtime_error {
public:
  explicit sql_exception(const std::string & message): runtime_error(message) {}
};

using DBPtr = sqlite3 *;

DBPtr open(const std::string & filename);

void execute_query(DBPtr db, const std::string & query);

void close(DBPtr db);

std::vector<std::string> getMessages(DBPtr db, std::string table = "messages");


}  // namespace sqlite
}  // namespace rosbag2

#endif  // ROSBAG2__STORAGE__SQLITE__SQLITE_HPP_
