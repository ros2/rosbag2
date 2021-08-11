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
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

#include "rosbag2_cpp/cache/circular_message_cache.hpp"
#include "rosbag2_cpp/cache/circular_message_cache_buffer.hpp"
#include "rosbag2_cpp/logging.hpp"

namespace rosbag2_cpp
{
namespace cache
{

CircularMessageCache::CircularMessageCache(uint64_t max_buffer_size)
{
  primary_buffer_ = std::make_shared<CircularMessageCacheBuffer>(max_buffer_size);
  secondary_buffer_ = std::make_shared<CircularMessageCacheBuffer>(max_buffer_size);
}

void CircularMessageCache::push(std::shared_ptr<const rosbag2_storage::SerializedBagMessage> msg)
{
  {
    std::lock_guard<std::mutex> cache_lock(cache_mutex_);
    if (!flushing_) {
      primary_buffer_->push(msg);
    }
  }
}

std::shared_ptr<CircularMessageCacheBuffer> CircularMessageCache::consumer_buffer()
{
  return secondary_buffer_;
}

void CircularMessageCache::swap_buffers()
{
  {
    std::lock_guard<std::mutex> cache_lock(cache_mutex_);
    std::swap(primary_buffer_, secondary_buffer_);
  }
}

CircularMessageCache::~CircularMessageCache()
{
}

}  // namespace cache
}  // namespace rosbag2_cpp
