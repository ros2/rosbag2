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

void CircularMessageCache::push(std::shared_ptr<const rosbag2_storage::SerializedBagMessage> msg)
{
  std::lock_guard<std::mutex> cache_lock(producer_buffer_mutex_);
  producer_buffer_->push(msg);
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
    consumer_buffer_->clear();
    std::swap(producer_buffer_, consumer_buffer_);
    data_ready_ = false;
  }
}

}  // namespace cache
}  // namespace rosbag2_cpp
