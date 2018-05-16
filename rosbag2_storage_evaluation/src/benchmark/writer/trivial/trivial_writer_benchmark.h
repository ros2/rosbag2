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

#ifndef ROS2_ROSBAG_EVALUATION_TRIVIAL_WRITER_BENCHMARK_H
#define ROS2_ROSBAG_EVALUATION_TRIVIAL_WRITER_BENCHMARK_H

#include <memory>

#include "benchmark/benchmark.h"
#include "generators/message_generator.h"
#include "profiler/profiler.h"
#include "writer/message_writer.h"

namespace ros2bag
{

class TrivialWriterBenchmark : public Benchmark
{
public:
  TrivialWriterBenchmark(
    std::unique_ptr<MessageGenerator> generator,
    std::unique_ptr<MessageWriter> writer,
    std::unique_ptr<Profiler> profiler)
    : generator_(std::move(generator)), writer_(std::move(writer)), profiler_(std::move(profiler))
  {}

  ~TrivialWriterBenchmark() override = default;

  void run() const override;

  void write_csv(std::ostream & out_stream, bool with_header) const override;

private:
  std::unique_ptr<MessageGenerator> generator_;
  std::unique_ptr<MessageWriter> writer_;
  std::unique_ptr<Profiler> profiler_;
};

}

#endif //ROS2_ROSBAG_EVALUATION_TRIVIAL_WRITER_BENCHMARK_H
