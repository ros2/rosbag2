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

#include "rcpputils/thread_safety_annotations.hpp"

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
* This is a "greedy consumer" implementation -  every time the consumer asks
* for a buffer to consume, the buffers are swapped so that the latest data
* goes to the consumer right away.
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

  ~MessageCache() override;

  /// Puts msg into primary buffer. With full cache, msg is ignored and counted as lost
  void push(std::shared_ptr<const rosbag2_storage::SerializedBagMessage> msg) override;

  /// Gets a consumer buffer.
  /// In this greedy implementation, swap buffers before providing the buffer.
  std::shared_ptr<CacheBufferInterface>
  get_consumer_buffer() override RCPPUTILS_TSA_ACQUIRE(consumer_buffer_mutex_);

  /// \brief Signals that the consumer is done consuming, unlocking the buffer so it may be swapped.
  void release_consumer_buffer() override RCPPUTILS_TSA_RELEASE(consumer_buffer_mutex_);

  /// \brief Blocks current thread and going to wait on condition variable until notify_data_ready
  /// will be called.
  void wait_for_data() override;

  /**
  * Consumer API: wait until primary buffer is ready and swap it with consumer buffer.
  * The caller thread (consumer thread) will sleep on a conditional variable
  * until it can be awaken, which is to happen when:
  * a) data was inserted into the producer buffer, consuming can continue after a swap
  * b) we are flushing the data (in case we missed the last notification when consuming)
  **/
  void swap_buffers() override;

  /// Set the cache to consume-only mode for final buffer flush before closing
  void begin_flushing() override;

  /// Notify that flushing is complete
  void done_flushing() override;

  /// Summarize dropped/remaining messages
  void log_dropped() override;

  /// Producer API: notify consumer to wake-up (primary buffer has data)
  void notify_data_ready() override;

protected:
  /// Dropped messages per topic. Used for printing in alphabetic order
  std::unordered_map<std::string, uint32_t> messages_dropped_per_topic_;

private:
  /// Double buffers
  std::shared_ptr<MessageCacheBuffer> producer_buffer_;
  std::mutex producer_buffer_mutex_;
  std::shared_ptr<MessageCacheBuffer> consumer_buffer_;
  std::mutex consumer_buffer_mutex_;

  /// Double buffers sync (following cpp core guidelines for condition variables)
  bool data_ready_ {false};
  std::condition_variable cache_condition_var_;

  /// Cache is no longer accepting messages and is in the process of flushing
  std::atomic_bool flushing_ {false};
};

}  // namespace cache
}  // namespace rosbag2_cpp

#ifdef _WIN32
# pragma warning(pop)
#endif

#endif  // ROSBAG2_CPP__CACHE__MESSAGE_CACHE_HPP_
