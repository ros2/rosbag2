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

#ifndef ROS2_ROSBAG_EVALUATION_SEPARATE_TOPIC_TABLE_SQLITE_WRITER_H
#define ROS2_ROSBAG_EVALUATION_SEPARATE_TOPIC_TABLE_SQLITE_WRITER_H

#include <map>
#include "writer/sqlite/one_table_sqlite_writer.h"
#include "writer/sqlite/sqlite_writer.h"

namespace ros2bag
{
class SeparateTopicTableSqliteWriter : public SqliteWriter
{
public:
  SeparateTopicTableSqliteWriter(
    std::string const & filename,
    unsigned int const messages_per_transaction,
    Indices const & indices,
    Pragmas const & pragmas);

  ~SeparateTopicTableSqliteWriter() override
  {
    SeparateTopicTableSqliteWriter::close();
  }

  void close() override;

  void reset() override;

private:
  sqlite::StatementPtr insert_message_stmt_;
  sqlite::StatementPtr insert_topic_stmt_;
  std::map<std::string, long> topic_ids_;

  void initialize_tables(sqlite::DBPtr db) final;

  void write_to_database(MessagePtr message) final;

  void prepare_statements(sqlite::DBPtr db) final;

  void insert_message(Message::Timestamp timestamp, long topic_id, BlobPtr blob);

  long insert_topic(std::string const & topic);
};
}
#endif //ROS2_ROSBAG_EVALUATION_SEPARATE_TOPIC_TABLE_SQLITE_WRITER_H
