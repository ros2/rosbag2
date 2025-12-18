// Copyright 2021 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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
#include <memory>
#include <string>
#include <utility>

#include "rosbag2_cpp/cache/circular_message_cache.hpp"
#include "rosbag2_cpp/cache/message_cache_circular_buffer.hpp"
#include "rosbag2_cpp/cache/cache_buffer_interface.hpp"
#include "rosbag2_cpp/logging.hpp"

namespace rosbag2_cpp
{
namespace cache
{

CircularMessageCache::CircularMessageCache(size_t max_buffer_size)
{
  producer_buffer_ = std::make_shared<MessageCacheCircularBuffer>(max_buffer_size);
  consumer_buffer_ = std::make_shared<MessageCacheCircularBuffer>(max_buffer_size);
}

CircularMessageCache::~CircularMessageCache()
{
  // Unblock wait_for_data on destruction
  flushing_ = true;
  cache_condition_var_.notify_one();
}

bool CircularMessageCache::push(std::shared_ptr<const rosbag2_storage::SerializedBagMessage> msg)
{
  std::lock_guard<std::mutex> cache_lock(producer_buffer_mutex_);
  return producer_buffer_->push(msg);
}

void CircularMessageCache::push_transient_local(
  std::shared_ptr<const rosbag2_storage::SerializedBagMessage> msg)
{
  std::lock_guard<std::mutex> cache_lock(transient_local_buffer_mutex_);
  // Store/update the latest message for this topic
  // This ensures we always have the most recent state for transient local topics
  transient_local_messages_[msg->topic_name] = std::move(msg);
}

std::shared_ptr<CacheBufferInterface> CircularMessageCache::get_consumer_buffer()
{
  consumer_buffer_mutex_.lock();
  return consumer_buffer_;
}

void CircularMessageCache::release_consumer_buffer()
{
  consumer_buffer_mutex_.unlock();
}

void CircularMessageCache::begin_flushing()
{
  {
    std::lock_guard<std::mutex> lock(producer_buffer_mutex_);
    flushing_ = true;
  }
  cache_condition_var_.notify_one();
}

void CircularMessageCache::done_flushing()
{
  flushing_ = false;
}

void CircularMessageCache::notify_data_ready()
{
  {
    std::lock_guard<std::mutex> lock(producer_buffer_mutex_);
    data_ready_ = true;
  }
  cache_condition_var_.notify_one();
}

void CircularMessageCache::wait_for_data()
{
  std::unique_lock<std::mutex> producer_lock(producer_buffer_mutex_);
  if (!flushing_) {
    // Required condition check to protect against spurious wakeups
    cache_condition_var_.wait(
      producer_lock, [this] {
        return data_ready_ || flushing_;
      });
  }
}

void CircularMessageCache::swap_buffers()
{
  std::lock_guard<std::mutex> producer_lock(producer_buffer_mutex_);
  // Swap buffers only if data is ready. Data not ready when we are calling flushing on exit and
  // we should not dump buffer on exit if snapshot has not been triggered.
  if (data_ready_) {
    std::lock_guard<std::mutex> consumer_lock(consumer_buffer_mutex_);
    std::lock_guard<std::mutex> transient_lock(transient_local_buffer_mutex_);
    consumer_buffer_->clear();
    std::swap(producer_buffer_, consumer_buffer_);

    // Merge latest transient local message for each topic into consumer buffer
    // Update timestamps to match the current snapshot time window to avoid timeline issues
    const auto & consumer_data = consumer_buffer_->data();
    rcutils_time_point_value_t snapshot_start_time = 0;

    // Use the front message timestamp as the snapshot start time for transient local messages.
    // This may not be the absolute earliest timestamp in the buffer (messages can arrive
    // out of order), but it's close enough - typically within 1-2ms of the true minimum.
    // This avoids the overhead of searching through all messages for the exact minimum.
    if (!consumer_data.empty()) {
      snapshot_start_time = consumer_data.front()->recv_timestamp;
    }

    // Add transient local messages with updated timestamp
    for (const auto & topic_msg_pair : transient_local_messages_) {
      if (snapshot_start_time > 0) {
        // Create a copy with updated timestamp to match snapshot window
        auto updated_msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
        *updated_msg = *topic_msg_pair.second;
        updated_msg->recv_timestamp = snapshot_start_time;
        updated_msg->send_timestamp = snapshot_start_time;
        consumer_buffer_->push(updated_msg);
      } else {
        // If buffer is empty, use original timestamps
        consumer_buffer_->push(topic_msg_pair.second);
      }
    }
    data_ready_ = false;
  }
}

}  // namespace cache
}  // namespace rosbag2_cpp
