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

#include "rosbag2_cpp/cache/transient_local_messages_cache.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace rosbag2_cpp
{
namespace cache
{

void TransientLocalMessagesCache::add_topic(const std::string & topic_name, size_t queue_depth)
{
  if (queue_depth == 0) {
    throw std::invalid_argument("Transient-local queue depth must be greater than 0");
  }

  std::lock_guard<std::mutex> lock(mutex_);
  auto & topic_queue = topic_queues_[topic_name];
  topic_queue.max_depth = queue_depth;
  while (topic_queue.messages.size() > topic_queue.max_depth) {
    topic_queue.messages.pop_front();
  }
}

void TransientLocalMessagesCache::remove_topic(const std::string & topic_name)
{
  std::lock_guard<std::mutex> lock(mutex_);
  topic_queues_.erase(topic_name);
}

bool TransientLocalMessagesCache::has_topic(const std::string & topic_name) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return topic_queues_.count(topic_name) > 0;
}

void TransientLocalMessagesCache::push(
  const std::string & topic_name,
  rosbag2_storage::SerializedBagMessageConstSharedPtr message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = topic_queues_.find(topic_name);
  if (it == topic_queues_.end()) {
    throw std::runtime_error(
            "Cannot push message to unregistered topic '" + topic_name +
            "'. Call add_topic() first.");
  }

  auto & queue = it->second;
  queue.messages.push_back(std::move(message));
  while (queue.messages.size() > queue.max_depth) {
    queue.messages.pop_front();
  }
}

std::vector<rosbag2_storage::SerializedBagMessageSharedPtr>
TransientLocalMessagesCache::get_messages_sorted_by_timestamp() const
{
  std::vector<rosbag2_storage::SerializedBagMessageSharedPtr> messages;
  messages.reserve(this->size());
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & [_, topic_queue] : topic_queues_) {
      for (const auto & msg : topic_queue.messages) {
        messages.emplace_back(std::make_shared<rosbag2_storage::SerializedBagMessage>(*msg));
      }
    }
  }

  std::stable_sort(
    messages.begin(), messages.end(),
    [](const auto & left, const auto & right) {
      if (left->recv_timestamp != right->recv_timestamp) {
        return left->recv_timestamp < right->recv_timestamp;
      }
      if (left->send_timestamp != right->send_timestamp) {
        return left->send_timestamp < right->send_timestamp;
      }
      return left->topic_name < right->topic_name;
    });

  return messages;
}

void TransientLocalMessagesCache::clear()
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto & [_, topic_queue] : topic_queues_) {
    topic_queue.messages.clear();
  }
}

size_t TransientLocalMessagesCache::size() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  size_t total_size = 0;
  for (const auto & [_, topic_queue] : topic_queues_) {
    total_size += topic_queue.messages.size();
  }
  return total_size;
}

}  // namespace cache
}  // namespace rosbag2_cpp
