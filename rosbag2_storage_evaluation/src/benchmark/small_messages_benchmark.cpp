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
#include <utility>
#include "benchmark/writer/sqlite/sqlite_writer_benchmark.h"
#include "generators/message_generator.h"
#include "profiler/profiler.h"
#include "writer/sqlite/one_table_sqlite_writer.h"

using namespace ros2bag;

void run_benchmark(
  std::string const & description,
  std::shared_ptr<MessageWriter> writer,
  std::string const & db_name,
  unsigned int number_of_messages,
  unsigned int message_blob_size,
  unsigned int transaction_size,
  bool with_header = false)
{
  std::vector<std::pair<std::string, std::string>> meta_data = {
    {"description",               description},
    {"number of messages",        std::to_string(number_of_messages)},
    {"message blob size (bytes)", std::to_string(message_blob_size)},
    {"transaction size",          std::to_string(transaction_size)}
  };

  MessageGenerator::Specification specification = {std::make_tuple("topic", message_blob_size)};

  SqliteWriterBenchmark benchmark(
    std::make_unique<MessageGenerator>(number_of_messages, specification),
    std::move(writer),
    std::make_unique<Profiler>(meta_data, db_name));

  std::remove(db_name.c_str());
  benchmark.run();
  std::remove(db_name.c_str());

  write_csv_file("small_messages_benchmark.csv", benchmark, with_header);
}

void run_benchmark_repeatedly(
  unsigned int times,
  std::string const & description,
  std::shared_ptr<MessageWriter> writer,
  std::string const & db_name,
  unsigned int number_of_messages,
  unsigned int message_blob_size,
  unsigned int transaction_size,
  bool with_header = false)
{
  for (int i = 0; i < times; ++i) {
    run_benchmark(
      description,
      writer,
      db_name,
      number_of_messages,
      message_blob_size,
      transaction_size,
      with_header);
    with_header = false;
  }
}

int main(int argc, char ** argv)
{
  /**
   * We write a total of 1GB to the Bagfile
   */
  std::string db_name = "small_messages_writer_benchmark.db";
  unsigned int msg_size_bytes = 10;
  unsigned int msg_count = 100000000;
  unsigned int transaction_size = 10000;

  auto const write_header = true;

  run_benchmark_repeatedly(5,
    "OneTableSqlite",
    std::make_shared<OneTableSqliteWriter>(
      db_name,
      transaction_size,
      Indices({{"MESSAGES", "TIMESTAMP"},
               {"MESSAGES", "TOPIC"}}),
      // Setting to "journal_mode" to "OFF" increases writing speed, but turns off transactions.
      Pragmas({{"journal_mode", "MEMORY"},
               {"synchronous",  "OFF"}})
    ),
    db_name,
    msg_count,
    msg_size_bytes,
    transaction_size, write_header);

  run_benchmark_repeatedly(5,
    "SeparateTopicTableSqlite",
    std::make_shared<SeparateTopicTableSqliteWriter>(
      db_name,
      transaction_size,
      Indices({{"MESSAGES", "TIMESTAMP"},
               {"MESSAGES", "TOPIC_ID"},
               {"TOPICS",   "TOPIC"}}),
      // Setting to "journal_mode" to "OFF" increases writing speed, but turns off transactions.
      Pragmas({{"foreign_keys", "ON"},
               {"journal_mode", "MEMORY"},
               {"synchronous",  "OFF"}})
    ),
    db_name,
    msg_count,
    msg_size_bytes,
    transaction_size);

  return EXIT_SUCCESS;
}
