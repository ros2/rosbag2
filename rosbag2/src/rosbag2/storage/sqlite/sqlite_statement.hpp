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

#ifndef ROSBAG2__STORAGE__SQLITE__SQLITE_STATEMENT_HPP_
#define ROSBAG2__STORAGE__SQLITE__SQLITE_STATEMENT_HPP_

#include <sqlite3.h>
#include <string>

namespace rosbag2
{

using DBPtr = sqlite3 *;

class SqliteStatement
{
public:
  SqliteStatement(DBPtr dbPtr, const std::string & query);
  ~SqliteStatement();

  sqlite3_stmt * get() const {return statement;}

private:
  sqlite3_stmt * statement;
};

}  // namespace rosbag2

#endif  // ROSBAG2__STORAGE__SQLITE__SQLITE_STATEMENT_HPP_
