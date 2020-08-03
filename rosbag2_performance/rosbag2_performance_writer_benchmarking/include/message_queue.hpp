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

#ifndef ROSBAG2_PERFORMAMCE_WRITER_BENCHMARKING__MESSAGE_QUEUE_HPP_
#define ROSBAG2_PERFORMAMCE_WRITER_BENCHMARKING__MESSAGE_QUEUE_HPP_

#include <string>
#include <queue>
#include <utility>
#include <mutex>
#include <atomic>
#include <iostream>
#include <memory>

#include "std_msgs/msg/byte_multi_array.hpp"

template<typename T>
class MessageQueue
{
public:
  MessageQueue(int maxSize, std::string topicName)
  : _maxSize(maxSize), _topicName(topicName), _unsuccessfulInsertCount(0) {}

  void push(std::shared_ptr<T> elem)
  {
    std::lock_guard<std::mutex> lock(_mutex);
    if (_queue.size() > _maxSize) {  // We skip the element and consider it "lost"
      ++_unsuccessfulInsertCount;
      std::cerr << "X" << std::flush;
      return;
    }
    _queue.push(elem);
  }

  bool is_complete() const
  {
    return _complete;
  }

  void set_complete()
  {
    _complete = true;
  }

  bool is_empty()
  {
    std::lock_guard<std::mutex> lock(_mutex);
    return _queue.empty();
  }

  std::shared_ptr<T> pop_and_return()
  {
    std::lock_guard<std::mutex> lock(_mutex);
    if (_queue.empty()) {
      throw std::out_of_range("Queue is empty, cannot pop. Check if empty first");
    }
    auto elem = _queue.front();
    _queue.pop();
    return elem;
  }

  unsigned int get_missed_elements_count() const
  {
    return _unsuccessfulInsertCount;
  }

  std::string topic_name() const
  {
    return _topicName;
  }

private:
  bool _complete = false;
  unsigned int _maxSize;
  std::string _topicName;
  std::atomic<unsigned int> _unsuccessfulInsertCount;
  std::queue<std::shared_ptr<T>> _queue;
  mutable std::mutex _mutex;
};

typedef MessageQueue<std_msgs::msg::ByteMultiArray> ByteMessageQueue;

#endif  // ROSBAG2_PERFORMAMCE_WRITER_BENCHMARKING__MESSAGE_QUEUE_HPP_
