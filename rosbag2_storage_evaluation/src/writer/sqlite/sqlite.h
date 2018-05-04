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

#ifndef ROS2_ROSBAG_EVALUATION_SQLITE_H
#define ROS2_ROSBAG_EVALUATION_SQLITE_H

#include <sqlite3.h>
#include <memory>
#include <vector>

namespace ros2bag
{
namespace sqlite
{

using DBPtr = sqlite3 *;
using ForeignKeyDef = std::tuple<std::string, std::string, std::string>;

DBPtr open_db(std::string const & name);

void close_db(DBPtr db);

void set_pragma(DBPtr db, std::string const & pragma, std::string const & value);

void create_table(
  DBPtr db,
  std::string const & name,
  std::vector<std::string> const & fields,
  std::vector<ForeignKeyDef> const & foreign_keys = {}
);

void create_index(DBPtr db, std::string const & table, std::string const & key);

using StatementPtr = sqlite3_stmt *;

StatementPtr new_insert_stmt(
  DBPtr db, std::string const & table, std::vector<std::string> const & fields);

void finalize(StatementPtr statement);

void exec(DBPtr db, std::string const & statement);

}
}

#endif //ROS2_ROSBAG_EVALUATION_SQLITE_WRITER_H
