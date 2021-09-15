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

void CircularMessageCache::push(std::shared_ptr<const rosbag2_storage::SerializedBagMessage> msg)
{
  std::lock_guard<std::mutex> cache_lock(producer_buffer_mutex_);
  producer_buffer_->push(msg);
}

std::shared_ptr<CacheBufferInterface> CircularMessageCache::consumer_buffer()
{
  consumer_buffer_mutex_.lock();
  return consumer_buffer_;
}

void CircularMessageCache::return_consumer_buffer()
{
  consumer_buffer_mutex_.unlock();
}

void CircularMessageCache::swap_buffers()
{
  std::lock_guard<std::mutex> producer_lock(producer_buffer_mutex_);
  std::lock_guard<std::mutex> consumer_lock(consumer_buffer_mutex_);
  consumer_buffer_->clear();
  std::swap(producer_buffer_, consumer_buffer_);
}

}  // namespace cache
}  // namespace rosbag2_cpp
