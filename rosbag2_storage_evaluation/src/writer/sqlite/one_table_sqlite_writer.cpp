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

#include "writer/sqlite/one_table_sqlite_writer.h"

#include "generators/message.h"
#include "writer/sqlite/sqlite.h"

using namespace ros2bag;


void OneTableSqliteWriter::close()
{
  if (is_open()) {
    sqlite::finalize(insert_message_stmt_);
    SqliteWriter::close();
  }
}

void OneTableSqliteWriter::write_to_database(MessagePtr message)
{
  sqlite3_bind_int64(insert_message_stmt_,
    1, message->timestamp().time_since_epoch().count());
  sqlite3_bind_text(insert_message_stmt_,
    2, message->topic().c_str(), static_cast<int>(message->topic().size()), nullptr);
  sqlite3_bind_blob(insert_message_stmt_,
    3, message->blob()->data(), static_cast<int>(message->blob()->size()), nullptr);

  sqlite3_step(insert_message_stmt_);
  sqlite3_reset(insert_message_stmt_);
}

void OneTableSqliteWriter::initialize_tables(sqlite::DBPtr db)
{
  sqlite::create_table(db, "MESSAGES", {
    "TIMESTAMP INTEGER NOT NULL",
    "TOPIC TEXT NOT NULL",
    "DATA BLOB NOT NULL"
  });
}

void OneTableSqliteWriter::prepare_statements(sqlite::DBPtr db)
{
  insert_message_stmt_ = sqlite::new_insert_stmt(db, "MESSAGES", {"TIMESTAMP", "TOPIC", "DATA"});
}
