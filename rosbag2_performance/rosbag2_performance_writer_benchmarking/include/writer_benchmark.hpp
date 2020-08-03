// Copyright 2020, Robotec.ai sp. z o.o.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROSBAG2_PERFORMAMCE_WRITER_BENCHMARKING__WRITER_BENCHMARK_HPP_
#define ROSBAG2_PERFORMAMCE_WRITER_BENCHMARKING__WRITER_BENCHMARK_HPP_

#include <string>
#include <vector>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/writers/sequential_writer.hpp"

#include "message_queue.hpp"
#include "byte_producer.hpp"

class WriterBenchmark : public rclcpp::Node
{
public:
  WriterBenchmark();
  void start_benchmark();

private:
  void create_producers(const ProducerConfig & config, unsigned int instances);
  void create_writer();
  void start_producers();

  ProducerConfig _config;
  unsigned int _maxCacheSize;
  std::string _dbFolder;
  std::shared_ptr<rosbag2_cpp::writers::SequentialWriter> _writer;
  std::vector<std::thread> _producerThreads;
  std::vector<std::unique_ptr<ByteProducer>> _producers;
  std::vector<std::shared_ptr<ByteMessageQueue>> _queues;
};

#endif  // ROSBAG2_PERFORMAMCE_WRITER_BENCHMARKING__WRITER_BENCHMARK_HPP_
