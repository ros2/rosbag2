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

#include "writer/sqlite/separate_topic_table_sqlite_writer.h"
#include "writer/sqlite/sqlite.h"

using namespace ros2bag;

SeparateTopicTableSqliteWriter::SeparateTopicTableSqliteWriter(
  std::string const & filename,
  unsigned int const messages_per_transaction,
  Indices const & indices,
  Pragmas const & pragmas
) : SqliteWriter(
  filename, messages_per_transaction, {{"MESSAGES", "TIMESTAMP"},
                                       {"MESSAGES", "TOPIC_ID"},
                                       {"TOPICS",   "TOPIC"}},
  pragmas)
{}

void SeparateTopicTableSqliteWriter::close()
{
  if (is_open()) {
    sqlite::finalize(insert_message_stmt_);
    sqlite::finalize(insert_topic_stmt_);
    SqliteWriter::close();
  }
}

void SeparateTopicTableSqliteWriter::initialize_tables(sqlite::DBPtr db)
{
  sqlite::create_table(db, "TOPICS", {
    "ID INTEGER PRIMARY KEY", // This is an alias for ROWID
    "TOPIC TEXT NOT NULL"
  });

  sqlite::create_table(db, "MESSAGES", {
    "TIMESTAMP INTEGER NOT NULL",
    "TOPIC_ID INTEGER NOT NULL",
    "DATA BLOB NOT NULL"
  }, {sqlite::ForeignKeyDef{"TOPIC_ID", "TOPICS", "ID"}});
}

void SeparateTopicTableSqliteWriter::write_to_database(MessagePtr message)
{
  std::string const topic = message->topic();
  long topic_id = topic_ids_[topic];
  if (!topic_id) {
    topic_id = insert_topic(topic);
    topic_ids_[topic] = topic_id;
  }

  insert_message(message->timestamp(), topic_id, message->blob());
}

long SeparateTopicTableSqliteWriter::insert_topic(std::string const & topic)
{
  // Note: the »TOPIC_ID« is set automatically by SQLite
  sqlite3_bind_text(
    insert_topic_stmt_, 1, topic.c_str(), static_cast<int>(topic.length()), nullptr);

  sqlite3_step(insert_topic_stmt_);
  long id = sqlite3_last_insert_rowid(db());

  sqlite3_reset(insert_topic_stmt_);
  return id;
}

void SeparateTopicTableSqliteWriter::insert_message(
  Message::Timestamp timestamp, long topic_id, BlobPtr blob
)
{
  sqlite3_bind_int64(insert_message_stmt_, 1, timestamp.time_since_epoch().count());
  sqlite3_bind_int64(insert_message_stmt_, 2, topic_id);
  sqlite3_bind_blob(insert_message_stmt_, 3, blob->data(), static_cast<int>(blob->size()), nullptr);

  sqlite3_step(insert_message_stmt_);
  sqlite3_reset(insert_message_stmt_);
}


void SeparateTopicTableSqliteWriter::prepare_statements(sqlite::DBPtr db)
{
  insert_message_stmt_ = sqlite::new_insert_stmt(db, "MESSAGES", {"TIMESTAMP", "TOPIC_ID", "DATA"});
  // Note: the »TOPIC_ID« is set automatically by SQLite
  insert_topic_stmt_ = sqlite::new_insert_stmt(db, "TOPICS", {"TOPIC"});
}

void SeparateTopicTableSqliteWriter::reset()
{
  topic_ids_.clear();
}
