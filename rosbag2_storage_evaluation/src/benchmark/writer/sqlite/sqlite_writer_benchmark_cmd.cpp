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

#include "benchmark/writer/sqlite/sqlite_writer_benchmark.h"
#include "writer/sqlite/one_table_sqlite_writer.h"

using namespace ros2bag;

int main(int argc, char ** argv)
{
  if (argc != 5) {
    std::cerr << "Usage: benchmark <database file name> <number of messages> <message blob size> "
              << "<messages per transaction>"
              << std::endl;
    return EXIT_FAILURE;
  }

  std::string database_name = argv[1];
  unsigned int number_messages = static_cast<unsigned int>(std::stol(argv[2]));
  unsigned int message_blob_size = static_cast<unsigned int>(std::stol(argv[3]));
  unsigned int messages_per_transaction = static_cast<unsigned int>(std::stol(argv[4]));

  MessageGenerator::Specification specification = {std::make_tuple("topic", message_blob_size)};

  std::vector<std::pair<std::string, std::string>> meta_data = {};
  SqliteWriterBenchmark benchmark(
    std::make_unique<MessageGenerator>(number_messages, specification),
    std::make_unique<OneTableSqliteWriter>(database_name, messages_per_transaction),
    std::make_unique<Profiler>(meta_data, database_name));

  benchmark.run();

  write_csv_file("sqlite3_writer_benchmark.csv", benchmark, true);

  return EXIT_SUCCESS;
}
