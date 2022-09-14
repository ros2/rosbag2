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

#ifndef ROSBAG2_PERFORMANCE_BENCHMARKING__WRITER_BENCHMARK_HPP_
#define ROSBAG2_PERFORMANCE_BENCHMARKING__WRITER_BENCHMARK_HPP_

#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rosbag2_compression/compression_options.hpp"
#include "rosbag2_cpp/writer.hpp"

#include "rosbag2_performance_benchmarking/byte_producer.hpp"
#include "rosbag2_performance_benchmarking/message_queue.hpp"
#include "rosbag2_performance_benchmarking/publisher_group_config.hpp"
#include "rosbag2_performance_benchmarking/bag_config.hpp"

class WriterBenchmark : public rclcpp::Node
{
public:
  explicit WriterBenchmark(const std::string & name);
  void start_benchmark();

private:
  void create_producers();
  void create_writer();
  void start_producers();
  void write_results() const;
  int get_message_count_from_metadata() const;

  std::vector<PublisherGroupConfig> configurations_;
  std::string results_file_;
  BagConfig bag_config_;

  std::vector<std::thread> producer_threads_;
  std::vector<std::unique_ptr<ByteProducer>> producers_;
  std::vector<std::shared_ptr<MessageQueue<rclcpp::SerializedMessage>>> queues_;
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  rclcpp::Clock system_clock_;
};

#endif  // ROSBAG2_PERFORMANCE_BENCHMARKING__WRITER_BENCHMARK_HPP_
