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

#include "trivial_writer_benchmark.h"

#include <algorithm>
#include <fstream>

#include "writer/stream/message_stream_writer.h"

using namespace ros2bag;

void TrivialWriterBenchmark::run() const
{
  generator_->reset();
  writer_->reset();


  Profiler::TickProgress throughput_tick = profiler_->measure_progress(
    "write_throughput", generator_->total_msg_count());
  profiler_->take_time_for("started writing messages");

  writer_->open();
  while (generator_->has_next()) {
    writer_->write(generator_->next());
    throughput_tick();
  }
  writer_->close();

  profiler_->take_time_for("finished writing messages");
  profiler_->track_disk_usage();
}

void TrivialWriterBenchmark::write_csv(std::ostream & out_stream, bool with_header) const
{
  if (with_header) {
    out_stream << profiler_->csv_header() << std::endl;
  }
  out_stream << profiler_->csv_entry() << std::endl;
}

int main(int argc, char ** argv)
{
  if (argc != 4) {
    std::cerr << "Usage: benchmark <text file name> <number of messages> <message blob size>"
              << std::endl;
    return EXIT_FAILURE;
  }

  std::string file_name = argv[1];
  unsigned int number_messages = static_cast<unsigned int>(std::stol(argv[2]));
  unsigned int message_blob_size = static_cast<unsigned int>(std::stol(argv[3]));

  std::vector<std::pair<std::string, std::string>> meta_data = {};
  MessageGenerator::Specification specification = {std::make_tuple("topic", message_blob_size)};
  std::ofstream file_stream(file_name);

  TrivialWriterBenchmark benchmark(
    std::make_unique<MessageGenerator>(number_messages, specification),
    std::make_unique<MessageStreamWriter>(file_stream),
    std::make_unique<Profiler>(meta_data, file_name));

  benchmark.run();

  write_csv_file("trivial_writer_benchmark.csv", benchmark, true);

  return EXIT_SUCCESS;
}
