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
#include "rosbag2_cpp/cache/message_cache_interface.hpp"
#include "rosbag2_cpp/cache/cache_buffer_interface.hpp"
#include "rosbag2_cpp/visibility_control.hpp"

#include "rosbag2_storage/serialized_bag_message.hpp"

// This is necessary because of using stl types here. It is completely safe, because
// a) the member is not accessible from the outside
// b) there are no inline functions.
#ifdef _WIN32
# pragma warning(push)
# pragma warning(disable:4251)
#endif

namespace rosbag2_cpp
{
namespace cache
{
/**
* This class is responsible for implementing message cache, using two cache
* buffers and providing synchronization API for producer-consumer pattern.
*
* Double buffering is a part of producer-consumer pattern and optimizes for
* the consumer performance (which can be a bottleneck, e.g. disk writes).
*
* Two instances of MessageCacheBuffer are used, one for producer and one for
* the consumer. Buffers are switched through swap_buffers function, which
* involves synchronization and a simple pointer switch.
*
* The cache can enter a flushing state, intended as a finalization state,
* where all the remaining data is going to be processed: no new messages are
* accepted and buffer switching can be done unconditionally on consumer demand.
*
* The cache holds infomation about dropped messages (per topic). These are
* messages that were pushed to the cache when it was full. Such situation signals
* performance issues, most likely with the CacheConsumer consumer callback.
*/
class ROSBAG2_CPP_PUBLIC MessageCache
  : public MessageCacheInterface
{
public:
  explicit MessageCache(size_t max_buffer_size);

  ~MessageCache();

  /// Puts msg into primary buffer. With full cache, msg is ignored and counted as lost
  void push(std::shared_ptr<const rosbag2_storage::SerializedBagMessage> msg) override;

  /// Summarize dropped/remaining messages
  void log_dropped() override;

  /// Set the cache to consume-only mode for final buffer flush before closing
  void finalize() override;

  /// Notify that flushing is complete
  void notify_flushing_done() override;

  /**
  * Consumer API: wait until primary buffer is ready and swap it with consumer buffer.
  * The caller thread (consumer thread) will sleep on a conditional variable
  * until it can be awaken, which is to happen when:
  * a) data was inserted into the producer buffer, consuming can continue after a swap
  * b) we are flushing the data (in case we missed the last notification when consuming)
  **/
  void swap_buffers() override;

  /// Consumer API: get current buffer to consume
  std::shared_ptr<CacheBufferInterface> consumer_buffer() override;

  /// Exposes counts of messages dropped per topic
  std::unordered_map<std::string, uint32_t> messages_dropped() const;

private:
  /// Producer API: notify consumer to wake-up (primary buffer has data)
  void notify_buffer_consumer();

  /// Double buffers
  std::shared_ptr<MessageCacheBuffer> primary_buffer_;
  std::shared_ptr<MessageCacheBuffer> secondary_buffer_;

  /// Dropped messages per topic. Used for printing in alphabetic order
  std::unordered_map<std::string, uint32_t> messages_dropped_per_topic_;

  /// Double buffers sync (following cpp core guidelines for condition variables)
  bool primary_buffer_can_be_swapped_ {false};
  std::condition_variable cache_condition_var_;
  std::mutex cache_mutex_;

  /// Cache is no longer accepting messages and is in the process of flushing
  std::atomic_bool flushing_ {false};
};

}  // namespace cache
}  // namespace rosbag2_cpp

#ifdef _WIN32
# pragma warning(pop)
#endif

#endif  // ROSBAG2_CPP__CACHE__MESSAGE_CACHE_HPP_
