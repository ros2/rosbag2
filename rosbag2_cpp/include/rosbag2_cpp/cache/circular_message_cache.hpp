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

#include <memory>
#include <mutex>
#include <string>

#include "rosbag2_cpp/cache/message_cache_circular_buffer.hpp"
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

class ROSBAG2_CPP_PUBLIC CircularMessageCache
{
public:
  explicit CircularMessageCache(uint64_t max_buffer_size);

  /// Puts msg into circular buffer, replacing the oldest msg when buffer is full
  void push(std::shared_ptr<const rosbag2_storage::SerializedBagMessage> msg);

  /// get current buffer to consume
  std::shared_ptr<MessageCacheCircularBuffer> consumer_buffer();

  /// Swap the primary and secondary buffer before consumption.
  /// NOTE: consumer_buffer() should be called sequentially after
  /// swap_buffer() to ensure expected behavior. Calling swap_buffer()
  /// while accessing consumer_buffer() will be undefined behavior.
  void swap_buffers();

private:
  /// Double buffers
  std::shared_ptr<MessageCacheCircularBuffer> primary_buffer_;
  std::shared_ptr<MessageCacheCircularBuffer> secondary_buffer_;

  /// Double buffers sync
  std::mutex cache_mutex_;
};

}  // namespace cache
}  // namespace rosbag2_cpp

#ifdef _WIN32
# pragma warning(pop)
#endif

#endif  // ROSBAG2_CPP__CACHE__CIRCULAR_MESSAGE_CACHE_HPP_
