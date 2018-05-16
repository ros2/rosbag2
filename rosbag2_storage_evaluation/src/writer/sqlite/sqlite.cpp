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

#include "writer/sqlite/sqlite.h"

#include <iostream>
#include <sstream>
#include <string>

#include "common/strings.h"
#include "common/vectors.h"

using namespace ros2bag::sqlite;

DBPtr ros2bag::sqlite::open_db(std::string const & name)
{
  DBPtr db;
  sqlite3_open(name.c_str(), &db);
  return db;
}

void ros2bag::sqlite::close_db(DBPtr db)
{
  sqlite3_close(db);
}

void ros2bag::sqlite::set_pragma(DBPtr db, std::string const & pragma, std::string const & value)
{
  std::string statement = "PRAGMA " + pragma + "=" + value;
  sqlite3_exec(db, statement.c_str(), nullptr, nullptr, nullptr);
}

void ros2bag::sqlite::create_table(
  DBPtr db,
  std::string const & name,
  std::vector<std::string> const & fields,
  std::vector<ForeignKeyDef> const & foreign_keys
)
{
  std::vector<std::string> definitions = fields;

  std::string foreign_key_column, parent_table, parent_column;
  for (auto const & key : foreign_keys) {
    std::tie(foreign_key_column, parent_table, parent_column) = key;

    std::ostringstream constraint;
    constraint << "FOREIGN KEY (" << foreign_key_column << ") "
               << "REFERENCES " << parent_table << " (" << parent_column << ")";

    definitions.push_back(constraint.str());
  }

  std::string statement(
    "CREATE TABLE IF NOT EXISTS "
      + name
      + ros2bag::strings::join(definitions, ",", "(", ")")
      + ";");
  sqlite3_exec(db, statement.c_str(), nullptr, nullptr, nullptr);
}

void ros2bag::sqlite::create_index(DBPtr db, std::string const & table, std::string const & key)
{
  std::string index_name = key + "_INDEX";
  std::string statement(
    "CREATE INDEX IF NOT EXISTS " + index_name + " ON " + table + "(" + key + ");");
  sqlite3_exec(db, statement.c_str(), nullptr, nullptr, nullptr);
}

StatementPtr ros2bag::sqlite::new_insert_stmt(DBPtr db, std::string const & table,
  std::vector<std::string> const & fields)
{
  StatementPtr stmt;

  std::string placeholder = "?";
  std::string sql = "INSERT INTO "
    + table
    + ros2bag::strings::join(fields, ",", "(", ")")
    + " VALUES"
    + ros2bag::strings::join(ros2bag::vectors::repeat(fields.size(), placeholder), ",", "(", ")")
    + ";";

  sqlite3_prepare_v2(db, sql.c_str(), static_cast<int>(sql.size()), &stmt, nullptr);

  return stmt;
}

void ros2bag::sqlite::finalize(StatementPtr statement)
{
  sqlite3_finalize(statement);
}

void ros2bag::sqlite::exec(DBPtr db, std::string const & statement)
{
  sqlite3_exec(db, statement.c_str(), nullptr, nullptr, nullptr);
}
