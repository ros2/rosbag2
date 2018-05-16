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

#ifndef ROS2_ROSBAG_EVALUATION_SQLITE_WRITER_H
#define ROS2_ROSBAG_EVALUATION_SQLITE_WRITER_H

#include <ostream>
#include <sqlite3.h>
#include <set>
#include <map>

#include "writer/message_writer.h"
#include "writer/sqlite/sqlite.h"

namespace ros2bag
{
using Pragmas = std::map<std::string, std::string>;
using Indices = std::vector<std::pair<std::string, std::string>>;

class SqliteWriter : public MessageWriter
{
public:
  explicit SqliteWriter(
    std::string const & filename,
    unsigned int const messages_per_transaction = 0,
    Indices const & indices = {},
    Pragmas const & pragmas = {{"journal_mode", "MEMORY"},
                               {"synchronous",  "OFF"}}
  ) : filename_(filename)
    , messages_per_transaction_(messages_per_transaction)
    , number_of_message_in_current_transaction_(0)
    , db_(nullptr)
    , insert_message_stmt_(nullptr)
    , open_(false)
    , in_transaction_(false)
    , indices_(indices)
    , pragmas_(pragmas)
  {}

  ~SqliteWriter() override
  {
    SqliteWriter::close();
  }

  void open() final;

  void close() override;

  void write(MessagePtr message) final;

  void create_index() final;

protected:
  sqlite::DBPtr db() const
  {
    return db_;
  }

  bool is_open() const
  {
    return open_;
  }

  virtual void initialize_tables(sqlite::DBPtr db) = 0;

  virtual void write_to_database(MessagePtr message) = 0;

  virtual void prepare_statements(sqlite::DBPtr db) = 0;

private:
  std::string const filename_;
  unsigned int const messages_per_transaction_;
  unsigned int number_of_message_in_current_transaction_;
  sqlite::DBPtr db_;
  sqlite::StatementPtr insert_message_stmt_;
  bool open_;
  bool in_transaction_;
  Pragmas pragmas_;
  Indices indices_;

  void set_pragmas();

  void begin_transaction();

  void end_transaction();
};

}

#endif //ROS2_ROSBAG_EVALUATION_SQLITE_WRITER_H
