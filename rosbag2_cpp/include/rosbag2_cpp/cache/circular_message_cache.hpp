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

class ROSBAG2_CPP_PUBLIC CircularMessageCache
{
public:
  explicit CircularMessageCache(uint64_t max_buffer_size);

  ~CircularMessageCache();

  /// Puts msg into circular buffer, replacing the oldest msg when buffer is full
  void push(std::shared_ptr<const rosbag2_storage::SerializedBagMessage> msg);

  /// Summarize dropped/remaining messages
  void log_dropped();

  /// Producer API: notify consumer to wake-up (primary buffer has data)
  void notify_buffer_consumer();

  /// Set the cache to consume-only mode for final buffer flush before closing
  void finalize();

  /// Notify that flushing is complete
  void notify_flushing_done();

  /**
  * Consumer API: wait until primary buffer is ready and swap it with consumer buffer.
  * The caller thread (consumer thread) will sleep on a conditional variable
  * until it can be awaken, which is to happen when:
  * a) data was inserted into the producer buffer, consuming can continue after a swap
  * b) we are flushing the data (in case we missed the last notification when consuming)
  **/
  void wait_for_buffer();

  /// Consumer API: get current buffer to consume
  std::shared_ptr<MessageCacheBuffer> consumer_buffer();

  /// Exposes counts of messages dropped per topic
  std::unordered_map<std::string, uint32_t> messages_dropped() const;

private:
  /// Double buffers
  std::shared_ptr<CircularMessageCacheBuffer> primary_buffer_;
  std::shared_ptr<CircularMessageCacheBuffer> secondary_buffer_;

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

#endif  // ROSBAG2_CPP__CACHE__CIRCULAR_MESSAGE_CACHE_HPP_
