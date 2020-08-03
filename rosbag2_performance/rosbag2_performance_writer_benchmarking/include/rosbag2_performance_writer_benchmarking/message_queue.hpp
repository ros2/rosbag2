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

#ifndef ROSBAG2_PERFORMANCE_WRITER_BENCHMARKING__MESSAGE_QUEUE_HPP_
#define ROSBAG2_PERFORMANCE_WRITER_BENCHMARKING__MESSAGE_QUEUE_HPP_

#include <atomic>
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <utility>

#include "std_msgs/msg/byte_multi_array.hpp"

template<typename T>
class MessageQueue
{
public:
  MessageQueue(int max_size, std::string topic_name)
  : max_size_(max_size),
    topic_name_(std::move(topic_name)),
    unsuccessful_insert_count_(0)
  {}

  void push(std::shared_ptr<T> elem)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (queue_.size() > max_size_) {  // We skip the element and consider it "lost"
      ++unsuccessful_insert_count_;
      std::cerr << "X" << std::flush;
      return;
    }
    queue_.push(elem);
  }

  bool is_complete() const
  {
    return complete_;
  }

  void set_complete()
  {
    complete_ = true;
  }

  bool is_empty()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.empty();
  }

  std::shared_ptr<T> pop_and_return()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (queue_.empty()) {
      throw std::out_of_range("Queue is empty, cannot pop. Check if empty first");
    }
    auto elem = queue_.front();
    queue_.pop();
    return elem;
  }

  unsigned int get_missed_elements_count() const
  {
    return unsuccessful_insert_count_;
  }

  std::string topic_name() const
  {
    return topic_name_;
  }

private:
  bool complete_ = false;
  unsigned int max_size_;
  std::string topic_name_;
  std::atomic<unsigned int> unsuccessful_insert_count_;
  std::queue<std::shared_ptr<T>> queue_;
  mutable std::mutex mutex_;
};

typedef MessageQueue<std_msgs::msg::ByteMultiArray> ByteMessageQueue;

#endif  // ROSBAG2_PERFORMANCE_WRITER_BENCHMARKING__MESSAGE_QUEUE_HPP_
