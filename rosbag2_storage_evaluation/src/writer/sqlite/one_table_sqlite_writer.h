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

#ifndef ROS2_ROSBAG_EVALUATION_ONE_TABLE_SQLITE_WRITER_H
#define ROS2_ROSBAG_EVALUATION_ONE_TABLE_SQLITE_WRITER_H

#include <ostream>
#include <sqlite3.h>

#include "writer/message_writer.h"
#include "writer/sqlite/sqlite.h"
#include "writer/sqlite/sqlite_writer.h"

namespace ros2bag
{

class OneTableSqliteWriter : public SqliteWriter
{
public:
  explicit OneTableSqliteWriter(
    std::string const & filename,
    unsigned int const messages_per_transaction = 0,
    Indices const & indices = {{"MESSAGES", "TOPIC"},
                               {"MESSAGES", "TIMESTAMP"}},
    Pragmas const & pragmas = {{"journal_mode", "MEMORY"},
                               {"synchronous",  "OFF"}}
  ) : SqliteWriter(filename, messages_per_transaction, indices, pragmas)
  {}

  ~OneTableSqliteWriter() override
  {
    OneTableSqliteWriter::close();
  }

  void close() override;

  void reset() override
  {}

protected:
  void initialize_tables(sqlite::DBPtr db) final;

  void write_to_database(MessagePtr message) final;

  void prepare_statements(sqlite::DBPtr db) final;

private:
  sqlite::StatementPtr insert_message_stmt_;
};

}

#endif //ROS2_ROSBAG_EVALUATION_ONE_TABLE_SQLITE_WRITER_H
