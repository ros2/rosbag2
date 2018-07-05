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

#include <string>
#include <vector>

#include "sqlite.hpp"

namespace rosbag2
{
namespace sqlite
{

DBPtr open(const std::string & filename)
{
  DBPtr database;

  int rc = sqlite3_open(filename.c_str(), &database);
  if(rc) {
    throw std::runtime_error("Can't open database: " + std::string(sqlite3_errmsg(database)));
  }

  return database;
}

void execute_query(DBPtr db, const std::string & query)
{
  char * error_msg = 0;
  int return_code = sqlite3_exec(db, query.c_str(), nullptr, nullptr, &error_msg);

  if(return_code != SQLITE_OK){
    auto error = "SQL error: " + std::string(error_msg);
    sqlite3_free(error_msg);
    throw std::runtime_error(error);
  }
}

void close(DBPtr db)
{
  sqlite3_close(db);
}

std::vector<std::string> getMessages(DBPtr db, std::string table)
{
  std::vector<std::string> table_msgs;
  sqlite3_stmt * statement;
  std::string query = "SELECT * FROM " + table;
  sqlite3_prepare_v2(db, query.c_str(), -1, &statement, nullptr);
  int result = sqlite3_step(statement);
  while (result == SQLITE_ROW) {
    table_msgs.emplace_back(
      std::string(reinterpret_cast<const char *>(sqlite3_column_text(statement, 1))));
    result = sqlite3_step(statement);
  }
  sqlite3_finalize(statement);

  return table_msgs;
}

}  // namespace sqlite
}  //namespace rosbag2

