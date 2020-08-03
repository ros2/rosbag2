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

#ifndef ROSBAG2_PERFORMANCE_WRITER_BENCHMARKING__BYTE_PRODUCER_HPP_
#define ROSBAG2_PERFORMANCE_WRITER_BENCHMARKING__BYTE_PRODUCER_HPP_

#include <chrono>
#include <memory>
#include <thread>
#include <vector>

#include "rclcpp/utilities.hpp"

#include "std_msgs/msg/byte_multi_array.hpp"

#include "rosbag2_performance_writer_benchmarking/message_queue.hpp"

struct ProducerConfig
{
  unsigned int frequency;
  unsigned int max_count;
  unsigned int message_size;
};

inline auto generate_random_message(const ProducerConfig & config)
{
  // Reuses the same random message
  auto message = std::make_shared<std_msgs::msg::ByteMultiArray>();

  message->data.reserve(config.message_size);
  for (auto i = 0u; i < config.message_size; ++i) {
    message->data.emplace_back(std::rand() % 255);
  }

  return message;
}

class ByteProducer
{
public:
  ByteProducer(const ProducerConfig & config, std::shared_ptr<ByteMessageQueue> queue)
  : configuration_(config),
    queue_(queue),
    sleep_time_(configuration_.frequency == 0 ? 1 : 1000 / configuration_.frequency),
    message_(generate_random_message(configuration_))
  {}

  void run()
  {
    for (auto i = 0u; i < configuration_.max_count; ++i) {
      if (!rclcpp::ok()) {
        break;
      }
      queue_->push(message_);
      std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time_));
    }
    queue_->set_complete();
  }

private:
  ProducerConfig configuration_;
  std::shared_ptr<ByteMessageQueue> queue_;
  unsigned int sleep_time_;  // in milliseconds
  // for simplification, this pointer will be reused
  std::shared_ptr<std_msgs::msg::ByteMultiArray> message_;
};

#endif  // ROSBAG2_PERFORMANCE_WRITER_BENCHMARKING__BYTE_PRODUCER_HPP_
