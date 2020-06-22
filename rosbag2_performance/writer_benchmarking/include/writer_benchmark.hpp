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

#ifndef WRITER_BENCHMARK_HPP_
#define WRITER_BENCHMARK_HPP_

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
  void startBenchmark();

private:
  void createProducers(const ProducerConfig & config, unsigned int instances);
  void createWriter();
  void startProducers();

  ProducerConfig mConfig;
  unsigned int mMaxCacheSize;
  std::string mDbFolder;
  std::shared_ptr<rosbag2_cpp::writers::SequentialWriter> mWriter;
  std::vector<std::thread> mProducerThreads;
  std::vector<std::shared_ptr<ByteProducer>> mProducers;
  std::vector<std::shared_ptr<MessageQueue<std_msgs::msg::ByteMultiArray>>> mQueues;
};

#endif  // WRITER_BENCHMARK_HPP_
