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

#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

#include "rosbag2_cpp/cache/message_cache.hpp"
#include "rosbag2_cpp/cache/message_cache_interface.hpp"
#include "rosbag2_cpp/logging.hpp"

namespace rosbag2_cpp
{
namespace cache
{

MessageCache::MessageCache(size_t max_buffer_size)
{
  producer_buffer_ = std::make_shared<MessageCacheBuffer>(max_buffer_size);
  consumer_buffer_ = std::make_shared<MessageCacheBuffer>(max_buffer_size);
}

MessageCache::~MessageCache()
{
  log_dropped();
}

void MessageCache::push(std::shared_ptr<const rosbag2_storage::SerializedBagMessage> msg)
{
  // While pushing, we keep track of inserted and dropped messages as well
  bool pushed = false;
  {
    std::lock_guard<std::mutex> lock(producer_buffer_mutex_);
    if (!flushing_) {
      pushed = producer_buffer_->push(msg);
    }
  }
  if (!pushed) {
    messages_dropped_per_topic_[msg->topic_name]++;
  }

  notify_buffer_consumer();
}

std::shared_ptr<CacheBufferInterface> MessageCache::consumer_buffer()
{
  consumer_buffer_mutex_.lock();
  swap_buffers();
  return consumer_buffer_;
}

void MessageCache::release_consumer_buffer()
{
  consumer_buffer_mutex_.unlock();
}

void MessageCache::swap_buffers()
{
  std::unique_lock<std::mutex> producer_lock(producer_buffer_mutex_);
  std::lock_guard<std::recursive_mutex> consumer_lock(consumer_buffer_mutex_);
  if (!flushing_) {
    // Required condition check to protect against spurious wakeups
    cache_condition_var_.wait(
      producer_lock, [this] {
        return primary_buffer_can_be_swapped_ || flushing_;
      });
    primary_buffer_can_be_swapped_ = false;
  }
  std::swap(producer_buffer_, consumer_buffer_);
}

void MessageCache::begin_flushing()
{
  {
    std::lock_guard<std::mutex> lock(producer_buffer_mutex_);
    flushing_ = true;
  }
  cache_condition_var_.notify_one();
}

void MessageCache::done_flushing()
{
  flushing_ = false;
}

void MessageCache::log_dropped()
{
  uint64_t total_lost = 0;
  std::string log_text("Cache buffers lost messages per topic: ");

  // worse performance than sorting key vector (neglible), but cleaner
  std::map<std::string, uint32_t> messages_dropped_per_topic_sorted(
    messages_dropped_per_topic_.begin(), messages_dropped_per_topic_.end());

  std::for_each(
    messages_dropped_per_topic_.begin(),
    messages_dropped_per_topic_.end(),
    [&total_lost, &log_text](const auto & e) {
      uint32_t lost = e.second;
      if (lost > 0) {
        log_text += "\n\t" + e.first + ": " + std::to_string(lost);
        total_lost += lost;
      }
    });

  if (total_lost > 0) {
    log_text += "\nTotal lost: " + std::to_string(total_lost);
    ROSBAG2_CPP_LOG_WARN_STREAM(log_text);
  }

  size_t remaining = producer_buffer_->size() + consumer_buffer_->size();
  if (remaining > 0) {
    ROSBAG2_CPP_LOG_WARN_STREAM(
      "Cache buffers were unflushed with " << remaining << " remaining messages"
    );
  }
}

void MessageCache::notify_buffer_consumer()
{
  {
    std::lock_guard<std::mutex> lock(producer_buffer_mutex_);
    primary_buffer_can_be_swapped_ = true;
  }
  cache_condition_var_.notify_one();
}

}  // namespace cache
}  // namespace rosbag2_cpp
