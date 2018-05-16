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

#include "writer/sqlite/sqlite_writer.h"

#include "generators/message.h"
#include "writer/sqlite/sqlite.h"

using namespace ros2bag;

void SqliteWriter::open()
{
  if (!is_open()) {
    open_ = true;
    db_ = sqlite::open_db(filename_);
    initialize_tables(db_);
    set_pragmas();
    prepare_statements(db_);
  }
}

void SqliteWriter::close()
{
  if (is_open()) {
    open_ = false;
    if (in_transaction_) {
      end_transaction();
    }
    sqlite::close_db(db_);
  }
}

void SqliteWriter::write(MessagePtr message)
{
  if (messages_per_transaction_ == 0) {
    write_to_database(message);
    return;
  }

  ++number_of_message_in_current_transaction_;

  if (number_of_message_in_current_transaction_ == 1) {
    begin_transaction();
  }

  write_to_database(message);

  if (number_of_message_in_current_transaction_ == messages_per_transaction_) {
    end_transaction();
    number_of_message_in_current_transaction_ = 0;
  }
}

void SqliteWriter::begin_transaction()
{
  sqlite::exec(db_, "BEGIN TRANSACTION");
  in_transaction_ = true;
}

void SqliteWriter::end_transaction()
{
  sqlite::exec(db_, "END TRANSACTION");
  in_transaction_ = false;
}

void SqliteWriter::create_index()
{
  std::string table, index_column;
  for (auto const & index : indices_) {
    std::tie(table, index_column) = index;
    sqlite::create_index(db_, table, index_column);
  }
}

void SqliteWriter::set_pragmas()
{
  std::string name, value;
  for (auto const & pragma : pragmas_) {
    std::tie(name, value) = pragma;
    sqlite::set_pragma(db_, name, value);
  }
}

