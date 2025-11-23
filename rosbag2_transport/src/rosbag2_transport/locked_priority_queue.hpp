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
#include <utility>

#include "rcpputils/thread_safety_annotations.hpp"
#include "rcpputils/unique_lock.hpp"

#ifdef __APPLE__
// no-op: Clang on macOS does not support thread-safety annotations
#pragma push_macro("RCPPUTILS_TSA_GUARDED_BY")
#undef RCPPUTILS_TSA_GUARDED_BY
#define RCPPUTILS_TSA_GUARDED_BY(x)
#endif

/// \brief `std::priority_queue` wrapper with locks and stable sorting.
/// \details This class wraps a `std::priority_queue` and provides locking for thread-safe
///   access. It uses std::vector as underlying container for the `std::priority_queue` and
///   adds a strictly increasing insertion sequence number to each element to
///   ensure a stable sort when two elements are equivalent according to the comparator.
///   This is useful when multiple elements has the same priorities in the queue, and we want to
///   ensure that elements with the same priority are popped in the order they were pushed.
///   For example, this is useful when multiple messages have the same timestamp, and we need to
///   preserve the original messages order.
/// \note The insertion sequence number is a builtin feature of this class, and users should not
///   provide it themselves.
/// \tparam T The element type
/// \see std::priority_queue
template<typename T>
class LockedPriorityQueue
{
public:
  using Comparator = std::function<bool(const T &, const T &)>;

  /// \brief Constructor.
  /// \param compare the comparator object
  explicit LockedPriorityQueue(const Comparator & compare)
  : queue_(StableComparator{compare})
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
    queue_.emplace(element, ++insert_sequence_number_);
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
    insert_sequence_number_ = 0;
  }

  /// \brief Try to take the top element from the queue.
  /// \return the top element, or `std::nullopt` if the queue is empty
  [[nodiscard]] std::optional<T> take()
  {
    rcpputils::unique_lock<std::mutex> lk(queue_mutex_);
    if (queue_.empty()) {
      return std::nullopt;
    }
    T e = queue_.top().first;
    queue_.pop();
    return e;
  }

private:
  using Container = std::vector<std::pair<T, size_t>>;

  /// \brief Internal wrapper comparator around the user-provided comparator.
  /// \details This comparator uses the user-provided comparator to compare the `T` values.
  ///   If the `T` values are equivalent according to the user comparator, it uses
  ///   the insertion sequence number to break ties, ensuring that earlier inserted elements
  ///   are considered "less than" later inserted elements.
  struct StableComparator
  {
    Comparator user_comp;
    bool operator()(const std::pair<T, size_t> & l, const std::pair<T, size_t> & r) const
    {
      const auto & [l_t_value, l_insertion_seq_num] = l;
      const auto & [r_t_value, r_insertion_seq_num] = r;
      if (user_comp(l_t_value, r_t_value)) {
        return true;
      }
      if (user_comp(r_t_value, l_t_value)) {
        return false;
      }
      // If values are equal according to user's comparator, use sequence numbers for stability
      // Tie-breaker: earlier insertion comes first
      return l_insertion_seq_num > r_insertion_seq_num;
    }
  };

  mutable std::mutex queue_mutex_;
  std::priority_queue<typename Container::value_type, Container, StableComparator> queue_
  RCPPUTILS_TSA_GUARDED_BY(queue_mutex_);

  size_t insert_sequence_number_{0} RCPPUTILS_TSA_GUARDED_BY(queue_mutex_);
};

#ifdef __APPLE__
#pragma pop_macro("RCPPUTILS_TSA_GUARDED_BY")
#endif

#endif  // ROSBAG2_TRANSPORT__LOCKED_PRIORITY_QUEUE_HPP_
