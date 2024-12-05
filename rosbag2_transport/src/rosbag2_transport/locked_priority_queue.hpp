// Copyright 2024 Apex.AI, Inc.
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

#ifndef ROSBAG2_TRANSPORT__LOCKED_PRIORITY_QUEUE_HPP_
#define ROSBAG2_TRANSPORT__LOCKED_PRIORITY_QUEUE_HPP_

#include <functional>
#include <mutex>
#include <optional>
#include <queue>
#include <vector>

#include "rcpputils/thread_safety_annotations.hpp"
#include "rcpputils/unique_lock.hpp"

/// \brief `std::priority_queue` wrapper with locks.
/// \tparam T the element type
/// \tparam Container the underlying container type
/// \tparam Compare the comparator
/// \see std::priority_queue
template<
  typename T,
  typename Container = std::vector<T>,
  typename Compare = std::less<typename Container::value_type>
>
class LockedPriorityQueue
{
public:
  /// \brief Constructor.
  /// \param compare the comparator object
  explicit LockedPriorityQueue(const Compare & compare)
  : queue_(compare)
  {}

  LockedPriorityQueue() = delete;
  LockedPriorityQueue(const LockedPriorityQueue &) = delete;
  LockedPriorityQueue & operator=(const LockedPriorityQueue &) = delete;
  LockedPriorityQueue(const LockedPriorityQueue &&) = delete;
  LockedPriorityQueue & operator=(const LockedPriorityQueue &&) = delete;

  /// \brief Insert element and sort queue.
  /// \param element the element
  void push(const T & element)
  {
    rcpputils::unique_lock<std::mutex> lk(queue_mutex_);
    queue_.push(element);
  }

  /// \brief Remove the top element.
  void pop()
  {
    rcpputils::unique_lock<std::mutex> lk(queue_mutex_);
    queue_.pop();
  }

  /// \brief Check if the queue is empty.
  /// \return whether the queue is empty
  [[nodiscard]] bool empty() const
  {
    rcpputils::unique_lock<std::mutex> lk(queue_mutex_);
    return queue_.empty();
  }

  /// \brief Get the number of elements.
  /// \return the number of elements
  [[nodiscard]] std::size_t size() const
  {
    rcpputils::unique_lock<std::mutex> lk(queue_mutex_);
    return queue_.size();
  }

  /// Remove all elements from the queue.
  void purge()
  {
    rcpputils::unique_lock<std::mutex> lk(queue_mutex_);
    while (!queue_.empty()) {
      queue_.pop();
    }
  }

  /// \brief Try to take the top element from the queue.
  /// \return the top element, or `std::nullopt` if the queue is empty
  [[nodiscard]] std::optional<T> take()
  {
    rcpputils::unique_lock<std::mutex> lk(queue_mutex_);
    if (queue_.empty()) {
      return std::nullopt;
    }
    T e = queue_.top();
    queue_.pop();
    return e;
  }

private:
  mutable std::mutex queue_mutex_;
  std::priority_queue<T, Container, Compare> queue_ RCPPUTILS_TSA_GUARDED_BY(queue_mutex_);
};

#endif  // ROSBAG2_TRANSPORT__LOCKED_PRIORITY_QUEUE_HPP_
