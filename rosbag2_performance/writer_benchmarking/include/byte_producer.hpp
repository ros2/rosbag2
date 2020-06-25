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

#ifndef BYTE_PRODUCER_HPP_
#define BYTE_PRODUCER_HPP_

#include <chrono>
#include <memory>
#include <thread>
#include <vector>
#include "message_queue.hpp"
#include "rclcpp/rclcpp.hpp"

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
  : mConfiguration(config), mQueue(queue)
  {
    generateRandomMessage();
    mMsSleepTime = mConfiguration.frequency == 0 ? 1 : 1000 / mConfiguration.frequency;
  }

  void run()
  {
    for (unsigned int i = 0; i < mConfiguration.max_count; ++i) {
      if (!rclcpp::ok()) {
        break;
      }
      mQueue->push(mMessage);
      std::this_thread::sleep_for(std::chrono::milliseconds(mMsSleepTime));
    }
    mQueue->setComplete();
  }

private:
  std::vector<uint8_t> randomByteArrayData(size_t size)
  {
    std::vector<uint8_t> byte(size, 0);
    for (size_t i = 0; i < size; ++i) {
      byte[i] = std::rand() % 255;
    }
    return byte;
  }

  void generateRandomMessage()
  {   // Reuses the same random message to remove generation time from benchmarks
    mMessage = std::make_shared<std_msgs::msg::ByteMultiArray>();
    mMessage->data = randomByteArrayData(mConfiguration.message_size);
  }

  // for simplification, this pointer will be reused
  std::shared_ptr<std_msgs::msg::ByteMultiArray> mMessage;
  std::mutex mMutex;

  ProducerConfig mConfiguration;
  std::shared_ptr<ByteMessageQueue> mQueue;

  unsigned int mMsSleepTime;
};

#endif  // BYTE_PRODUCER_HPP_
