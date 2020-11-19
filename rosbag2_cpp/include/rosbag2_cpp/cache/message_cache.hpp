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

#include <atomic>
#include <condition_variable>
#include <unordered_map>
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
// and providing synchronization API for producer-consumer pattern
class ROSBAG2_CPP_PUBLIC MessageCache
{
public:
  MessageCache(const uint64_t & max_buffer_size);

  ~MessageCache();

  // Puts msg into primary buffer. With full cache, msg is ignored and counted as lost
  void push(std::shared_ptr<const rosbag2_storage::SerializedBagMessage> msg);

  // Summarize dropped/remaining messages
  void log_dropped();

  // Producer API: notify consumer to wake-up (primary buffer has data)
  void notify_buffer_consumer();

  // Set the cache to consume-only mode for final buffer flush before closing
  void finalize();

  // Notify that flushing is complete
  void notify_flushing_done();

  /**
  * Consumer API: wait until primary buffer is ready and swap it with consumer buffer.
  * The caller thread (consumer thread) will sleep on a conditional variable
  * until it can be awaken, which is to happen when:
  * a) data was inserted into the producer buffer, consuming can continue after a swap
  * b) we are flushing the data (in case we missed the last notification when consuming)
  **/
  void wait_for_buffer();

  // Consumer API: get current buffer to consume
  std::shared_ptr<MessageCacheBuffer> consumer_buffer();

  // Exposes counts of messages dropped per topic
  std::unordered_map<std::string, uint32_t> messages_dropped() const;

private:
  // Double buffers
  std::shared_ptr<MessageCacheBuffer> primary_buffer_;
  std::shared_ptr<MessageCacheBuffer> secondary_buffer_;

  // Dropped messages per topic. Used for printing in alphabetic order
  std::unordered_map<std::string, uint32_t> messages_dropped_per_topic_;

  // Double buffers sync (following cpp core guidelines for condition variables)
  bool primary_buffer_can_be_swapped_ {false};
  std::condition_variable cache_condition_var_;
  std::mutex cache_mutex_;

  // Cache is no longer accepting messages and is in the process of flushing
  std::atomic_bool flushing_ {false};
};

}  // namespace cache
}  // namespace rosbag2_cpp

#endif  // ROSBAG2_CPP__CACHE__MESSAGE_CACHE_HPP_
