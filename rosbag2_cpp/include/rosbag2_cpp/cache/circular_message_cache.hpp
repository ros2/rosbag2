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

#ifndef ROSBAG2_CPP__CACHE__CIRCULAR_MESSAGE_CACHE_HPP_
#define ROSBAG2_CPP__CACHE__CIRCULAR_MESSAGE_CACHE_HPP_

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>

#include "rcpputils/thread_safety_annotations.hpp"

#include "rosbag2_cpp/cache/message_cache_circular_buffer.hpp"
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

/// Provides a "deferred-consumption" implementation of the MessageCacheInterface.
/// When a consumer asks for a buffer, it will not receive a new buffer until some control
/// source calls `swap_buffers` manually.
/// This is useful for a snapshot mode, where no data is written to disk until asked for,
/// then the full circular buffer is dumped all at once, giving historical context.
class ROSBAG2_CPP_PUBLIC CircularMessageCache
  : public MessageCacheInterface
{
public:
  explicit CircularMessageCache(size_t max_buffer_size);

  ~CircularMessageCache() override;

  /// Puts msg into circular buffer, replacing the oldest msg when buffer is full
  void push(std::shared_ptr<const rosbag2_storage::SerializedBagMessage> msg) override;

  /// Get current buffer to consume.
  /// Locks consumer buffer until release_consumer_buffer is called.
  /// This may be repeatedly empty if `swap_buffers` has not been called.
  std::shared_ptr<CacheBufferInterface>
  get_consumer_buffer() override RCPPUTILS_TSA_ACQUIRE(consumer_buffer_mutex_);

  /// Unlock access to the consumer buffer.
  void release_consumer_buffer() override RCPPUTILS_TSA_RELEASE(consumer_buffer_mutex_);

  /// \brief Blocks current thread and going to wait on condition variable until notify_data_ready
  /// will be called.
  void wait_for_data() override;

  /// Swap the primary and secondary buffer before consumption.
  /// NOTE: If swap_buffers is called again before consuming via get_consumer_buffer,
  /// that data will be cleared for use by the producer.
  void swap_buffers() override;

  /// Signal wait_for_data to wake up and unblock consumer thread on exit or during bag split
  void begin_flushing() override;

  /// Notify that flushing is complete
  void done_flushing() override;

  /// Snapshot API: notify cache consumer to wake-up for dumping buffer
  void notify_data_ready() override;

private:
  std::shared_ptr<MessageCacheCircularBuffer> producer_buffer_;
  std::mutex producer_buffer_mutex_;
  std::shared_ptr<MessageCacheCircularBuffer> consumer_buffer_;
  std::mutex consumer_buffer_mutex_;

  bool data_ready_ {false};
  std::condition_variable cache_condition_var_;

  /// Synchronization flag. Needs for unblock wait_for_data on exit.
  std::atomic_bool flushing_ {false};
};

}  // namespace cache
}  // namespace rosbag2_cpp

#ifdef _WIN32
# pragma warning(pop)
#endif

#endif  // ROSBAG2_CPP__CACHE__CIRCULAR_MESSAGE_CACHE_HPP_
