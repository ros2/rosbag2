// Copyright 2026 Dexory
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

#ifndef ROSBAG2_CPP__CACHE__TRANSIENT_LOCAL_MESSAGES_CACHE_HPP_
#define ROSBAG2_CPP__CACHE__TRANSIENT_LOCAL_MESSAGES_CACHE_HPP_

#include <cstddef>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "rosbag2_cpp/visibility_control.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"

namespace rosbag2_cpp
{
namespace cache
{

/// \brief Thread-safe cache for transient-local messages with per-topic FIFO queues.
///
/// Maintains per-topic circular queues of the last N messages, where N is a configurable
/// depth per topic. Used to prepend transient-local messages on bag splits and snapshots.
class ROSBAG2_CPP_PUBLIC TransientLocalMessagesCache
{
public:
  /// \brief Add a topic to the cache with a specified queue depth.
  /// If the topic already exists, its queue depth will be updated. If the new depth is smaller
  /// than the current number of cached messages, the oldest messages will be discarded.
  /// \param topic_name The name of the topic.
  /// \param queue_depth The maximum number of messages to store for this topic.
  /// \throws std::invalid_argument if the queue depth is zero.
  void add_topic(const std::string & topic_name, size_t queue_depth);

  /// \brief Remove a topic and all its cached messages from the cache.
  /// \param topic_name The name of the topic to remove. No-op if the topic does not exist.
  void remove_topic(const std::string & topic_name);

  /// \brief Check whether a topic is registered in the cache.
  /// \param topic_name The name of the topic.
  /// \return true if the topic exists in the cache, false otherwise.
  bool has_topic(const std::string & topic_name) const;

  /// \brief Push a message into the cache for a given topic.
  /// If the queue is full, the oldest message will be discarded.
  /// \param topic_name The name of the topic.
  /// \param message The serialized message to cache.
  /// \throws std::runtime_error if the topic has not been registered via add_topic().
  void push(
    const std::string & topic_name,
    rosbag2_storage::SerializedBagMessageConstSharedPtr message);

  /// \brief Retrieve all cached messages across all topics, sorted by timestamp.
  /// Messages are sorted by recv_timestamp first, then send_timestamp, then topic_name.
  /// \return A vector of cached messages sorted by timestamp.
  std::vector<rosbag2_storage::SerializedBagMessageSharedPtr>
  get_messages_sorted_by_timestamp() const;

  /// \brief Clear all cached messages from all topics without removing the topic registrations.
  void clear();

  /// \brief Get the total number of cached messages across all topics.
  /// \return The total number of cached messages.
  size_t size() const;

private:
  struct TopicQueue
  {
    size_t max_depth{0};
    std::deque<rosbag2_storage::SerializedBagMessageConstSharedPtr> messages;
  };

  mutable std::mutex mutex_;
  std::unordered_map<std::string, TopicQueue> topic_queues_;
};

}  // namespace cache
}  // namespace rosbag2_cpp

#endif  // ROSBAG2_CPP__CACHE__TRANSIENT_LOCAL_MESSAGES_CACHE_HPP_
