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

#include <fstream>

#include "writer/sqlite/one_table_sqlite_writer.h"

using namespace ros2bag;

void SqliteWriterBenchmark::run() const
{
  generator_->reset();
  writer_->reset();

  profiler_->take_time_for("start writing time");

  Profiler::TickProgress throughput_tick = profiler_->measure_progress(
    "write_throughput", generator_->total_msg_count());

  writer_->open();
  while (generator_->has_next()) {
    writer_->write(generator_->next());
    throughput_tick();
  }

  profiler_->take_time_for("end writing time");

  profiler_->take_time_for("start indexing time");

  writer_->create_index();
  writer_->close();

  profiler_->take_time_for("end indexing time");
  profiler_->track_disk_usage();
}

void SqliteWriterBenchmark::write_csv(std::ostream & out_stream, bool with_header) const
{
  if (with_header) {
    out_stream << profiler_->csv_header() << std::endl;
  }
  out_stream << profiler_->csv_entry() << std::endl;
}
