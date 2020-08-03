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

#ifndef ROSBAG2_PERFORMAMCE_WRITER_BENCHMARKING__BYTE_PRODUCER_HPP_
#define ROSBAG2_PERFORMAMCE_WRITER_BENCHMARKING__BYTE_PRODUCER_HPP_

#include <chrono>
#include <memory>
#include <thread>
#include <vector>
#include "message_queue.hpp"
#include "rclcpp/utilities.hpp"

struct ProducerConfig
{
  unsigned int frequency;
  unsigned int max_count;
  unsigned int message_size;
};

class ByteProducer
{
public:
  ByteProducer(const ProducerConfig & config, std::shared_ptr<ByteMessageQueue> queue)
  : _configuration(config), _queue(queue)
  {
    generate_random_message();
    _msSleepTime = _configuration.frequency == 0 ? 1 : 1000 / _configuration.frequency;
  }

  void run()
  {
    for (unsigned int i = 0; i < _configuration.max_count; ++i) {
      if (!rclcpp::ok()) {
        break;
      }
      _queue->push(_message);
      std::this_thread::sleep_for(std::chrono::milliseconds(_msSleepTime));
    }
    _queue->set_complete();
  }

private:
  std::vector<uint8_t> random_bytearray_data(size_t size)
  {
    std::vector<uint8_t> byte(size, 0);
    for (size_t i = 0; i < size; ++i) {
      byte[i] = std::rand() % 255;
    }
    return byte;
  }

  void generate_random_message()
  {   // Reuses the same random message
    _message = std::make_shared<std_msgs::msg::ByteMultiArray>();
    _message->data = random_bytearray_data(_configuration.message_size);
  }

  // for simplification, this pointer will be reused
  std::shared_ptr<std_msgs::msg::ByteMultiArray> _message;
  ProducerConfig _configuration;
  std::shared_ptr<ByteMessageQueue> _queue;
  unsigned int _msSleepTime;
};

#endif  // ROSBAG2_PERFORMAMCE_WRITER_BENCHMARKING__BYTE_PRODUCER_HPP_
