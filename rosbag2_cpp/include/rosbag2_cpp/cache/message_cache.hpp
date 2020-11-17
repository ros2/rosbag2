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

#ifndef ROSBAG2_CPP__CACHE__MESSAGE_CACHE_HPP_
#define ROSBAG2_CPP__CACHE__MESSAGE_CACHE_HPP_

#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rosbag2_cpp/cache/message_cache_buffer.hpp"
#include "rosbag2_cpp/visibility_control.hpp"

#include "rosbag2_storage/serialized_bag_message.hpp"

namespace rosbag2_cpp
{
namespace cache
{
// This class is responsible for implementing message cache, using cache buffers
// and providing synchronisation API for producer-consumer pattern
class ROSBAG2_CPP_PUBLIC MessageCache
{
public:
  MessageCache(const uint64_t & max_buffer_size);

  ~MessageCache();

  // Push data into primary buffer
  bool push(std::shared_ptr<const rosbag2_storage::SerializedBagMessage> msg);

  // Summarize dropped/remaining messages
  void log_dropped();

  // Notify of data in the primary buffer
  void allow_swap();

  // Consumer API: wait until there is data to consume and swap
  void wait_for_swap();

  // Consumer API: get current buffer to consume
  std::shared_ptr<MessageCacheBuffer> consumer_buffer();

private:
  // Double buffers
  std::shared_ptr<MessageCacheBuffer> primary_buffer_;
  std::shared_ptr<MessageCacheBuffer> secondary_buffer_;

  // Dropped messages per topic. Used for printing in alphabetic order
  std::map<std::string, uint32_t> messages_dropped_per_topic_;

  // Double buffers sync
  std::condition_variable swap_ready_;
  std::mutex buffer_mutex_;
};

}  // namespace cache
}  // namespace rosbag2_cpp

#endif  // ROSBAG2_CPP__CACHE__MESSAGE_CACHE_HPP_
