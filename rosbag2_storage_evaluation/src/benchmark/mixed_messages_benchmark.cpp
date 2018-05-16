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
#include "benchmark/writer/sqlite/sqlite_writer_benchmark.h"
#include "generators/message_generator.h"
#include "profiler/profiler.h"
#include "writer/sqlite/one_table_sqlite_writer.h"

using namespace ros2bag;

void run_benchmark(
  std::string const & description,
  std::shared_ptr<MessageWriter> writer,
  std::string const & db_name,
  unsigned int loop_count,
  unsigned int number_of_small_messages,
  unsigned int small_message_blob_size,
  unsigned int number_of_medium_messages,
  unsigned int medium_message_blob_size,
  unsigned int number_of_big_messages,
  unsigned int big_message_blob_size,
  unsigned int transaction_size,
  bool with_header = false)
{

  std::vector<std::pair<std::string, std::string>> meta_data = {
    {"description",                      description},
    {"number of small messages",         std::to_string(number_of_small_messages * loop_count)},
    {"small message blob size (bytes)",  std::to_string(small_message_blob_size)},
    {"number of medium messages",        std::to_string(number_of_medium_messages * loop_count)},
    {"medium message blob size (bytes)", std::to_string(medium_message_blob_size)},
    {"number of big messages",           std::to_string(number_of_big_messages * loop_count)},
    {"big message blob size (bytes)",    std::to_string(big_message_blob_size)},
    {"transaction size",                 std::to_string(transaction_size)}
  };

  MessageGenerator::Specification specification;
  for (auto i = 0; i < number_of_small_messages; ++i) {
    specification.emplace_back("topic/small/" + std::to_string(i), small_message_blob_size);
  }
  for (auto i = 0; i < number_of_medium_messages; ++i) {
    specification.emplace_back("topic/medium/" + std::to_string(i), medium_message_blob_size);
  }
  for (auto i = 0; i < number_of_big_messages; ++i) {
    specification.emplace_back("topic/big/" + std::to_string(i), big_message_blob_size);
  }

  SqliteWriterBenchmark benchmark(
    std::make_unique<MessageGenerator>(loop_count, specification),
    std::move(writer),
    std::make_unique<Profiler>(meta_data, db_name));

  std::remove(db_name.c_str());
  benchmark.run();
  std::remove(db_name.c_str());

  write_csv_file("mixed_messages_benchmark.csv", benchmark, with_header);
}

void run_benchmark_repeatedly(
  unsigned int times,
  std::string const & description,
  std::shared_ptr<MessageWriter> writer,
  std::string const & db_name,
  unsigned int loop_count,
  unsigned int number_of_small_messages,
  unsigned int small_message_blob_size,
  unsigned int number_of_medium_messages,
  unsigned int medium_message_blob_size,
  unsigned int number_of_big_messages,
  unsigned int big_message_blob_size,
  unsigned int transaction_size,
  bool with_header = false)
{
  for (int i = 0; i < times; ++i) {
    run_benchmark(
      description,
      writer,
      db_name,
      loop_count,
      number_of_small_messages,
      small_message_blob_size,
      number_of_medium_messages,
      medium_message_blob_size,
      number_of_big_messages,
      big_message_blob_size,
      transaction_size,
      with_header);
    with_header = false;
  }
}

int main(int argc, char ** argv)
{
  /**
   * We write a total of 10GB to the Bagfile.
   * Stream:
   * *
   */
  std::string db_name = "mixed_messages_benchmark.db";
  unsigned int const transaction_size = 10000;


  auto const write_header = true;

  auto const small_messages = 1000;
  auto const small_message_blob_size = 10;

  auto const medium_messages = 100;
  auto const medium_message_blob_size = 1000;

  auto const big_messages = 1;
  auto const big_message_blob_size = 30000000; // 30 MB

  auto const loop_count = 300; // gives roughly 10GB



  run_benchmark_repeatedly(3,
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
    loop_count,
    small_messages,
    small_message_blob_size,
    medium_messages,
    medium_message_blob_size,
    big_messages,
    big_message_blob_size, transaction_size, write_header);

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
    loop_count,
    small_messages,
    small_message_blob_size,
    medium_messages,
    medium_message_blob_size,
    big_messages,
    big_message_blob_size, transaction_size, write_header);

  return EXIT_SUCCESS;
}

