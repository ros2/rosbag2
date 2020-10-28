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

#ifndef ROSBAG2_CPP__WRITERS__CIRCULAR_BUFFER_HPP
#define ROSBAG2_CPP__WRITERS__CIRCULAR_BUFFER_HPP

#include <stdexcept>
#include <vector>

#include "rosbag2_cpp/visibility_control.hpp"

namespace rosbag2_cpp
{

namespace writers
{

template<typename T>
class ROSBAG2_CPP_PUBLIC CircularBuffer
{
public:
  CircularBuffer(const uint32_t size)
  {
    if (size <= 0) {
      throw std::invalid_argument("Invalid circular buffer size.");
    }
    size_ = size;
    array_.resize(size_);
  }

  uint32_t size()
  {
    return size_;
  }

  uint32_t failed_counter()
  {
    return failed_counter_;
  }

  void increment_failed_counter() {
    failed_counter_++;
  }

  uint32_t elements_num()
  {
    return elements_num_;
  }

  bool enqueue(T item)
  {
    if (is_full()) {
      failed_counter_++;
      return false;
    }
    array_[rear_index_] = item;
    rear_index_ = (rear_index_ + 1) % size_;
    elements_num_++;
    return true;
  }

  void dequeue()
  {
    if (is_empty()) {
      return;
    }
    front_index_ = (front_index_ + 1) % size_;
    elements_num_--;
  }

  T front()
  {
    return is_empty() ? nullptr : array_[front_index_];
  }

  bool is_empty()
  {
    return rear_index_ == front_index_;
  }

  bool is_full()
  {
    return (rear_index_ + 1) % size_ == front_index_;
  }

  void clear()
  {
    array_.clear();
  }

private:
  std::vector<T> array_;
  uint32_t front_index_ = 0u, rear_index_ = 0u, size_ = 0u, failed_counter_ = 0u, elements_num_ = 0u;
};

}  // namespace writers
}  // namespace rosbag2_cpp

#endif  // ROSBAG2_CPP__WRITERS__CIRCULAR_BUFFER_HPP
