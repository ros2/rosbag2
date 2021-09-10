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

#ifndef ROSBAG2_CPP__CACHE__MESSAGE_CACHE_BUFFER_HPP_
#define ROSBAG2_CPP__CACHE__MESSAGE_CACHE_BUFFER_HPP_

#include <atomic>
#include <memory>
#include <vector>

#include "rosbag2_cpp/visibility_control.hpp"
#include "rosbag2_cpp/cache/cache_buffer_interface.hpp"
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
* This class implements a single buffer for message cache. The buffer is byte size
* limited and won't accept any messages when current buffer byte size is already
* over the limit set by max_cache_size. This means that buffer can at times use
* more memory than max_cache_size, but never by more than a single message. When
* the buffer is full, the next incoming message is dropped.
*
* Note that it could be reused as a template with any class that has
* ->byte_size() - like interface
*/
class ROSBAG2_CPP_PUBLIC MessageCacheBuffer
  : public CacheBufferInterface
{
public:
  explicit MessageCacheBuffer(const uint64_t max_cache_size);

  /**
  * If buffer size got some space left, we push message regardless of its size, but if
  * this results in exceeding buffer size, we mark buffer to drop all new incoming messages.
  * This flag is cleared when buffers are swapped.
  */
  bool push(CacheBufferInterface::buffer_element_t msg) override;

  /// Clear buffer
  void clear() override;

  /// Get number of elements in the buffer
  size_t size() override;

  /// Get buffer data
  const std::vector<CacheBufferInterface::buffer_element_t> & data() override;

private:
  std::vector<CacheBufferInterface::buffer_element_t> buffer_;
  uint64_t buffer_bytes_size_ {0u};
  const uint64_t max_bytes_size_;

  /// set when buffer is full and should drop messages instead of inserting them
  std::atomic_bool drop_messages_ {false};
};

}  // namespace cache
}  // namespace rosbag2_cpp

#ifdef _WIN32
# pragma warning(pop)
#endif

#endif  // ROSBAG2_CPP__CACHE__MESSAGE_CACHE_BUFFER_HPP_
