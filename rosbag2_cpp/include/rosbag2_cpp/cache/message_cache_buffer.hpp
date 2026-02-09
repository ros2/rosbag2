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

/// \brief This class implements a single buffer for message cache.
/// \details The buffer is limited by total byte size of the stored messages or by total duration
/// of the stored messages. When the buffer is full, the next incoming message is dropped.
/// \note When both max_cache_size and max_cache_duration are set, the buffer will drop
/// messages when either of the limits is exceeded.
/// \note If max_cache_size is set to zero, the buffer will be only bounded by max_cache_duration.
/// \note If max_cache_duration is set to zero, the buffer will be only bounded by
/// max_cache_size.
/// \note if max_cache_size more than zero, but an individual message exceeds the buffer size,
/// the message still will be added to the buffer. This means that buffer can at times use more
/// memory than max_cache_size, but never by more than a single message. When the buffer is full,
/// the next incoming message is dropped.
class ROSBAG2_CPP_PUBLIC MessageCacheBuffer
  : public CacheBufferInterface
{
public:
  /// \brief Parametrized constructor
  /// \param max_cache_size Maximum amount of memory which could be occupied by the messages stored
  /// in the buffer. Note. If max_cache_size is zero, the buffer will be only bounded by the
  /// max_cache_duration.
  /// \param max_cache_duration Maximum duration in seconds of message sequence allowed to be
  /// stored in the buffer. Note. If max_cache_duration is zero, the buffer will be only bounded by
  /// the max_cache_size.
  /// \throws std::invalid_argument if both max_cache_size and max_cache_duration are zero.
  explicit MessageCacheBuffer(size_t max_cache_size, uint32_t max_cache_duration = 0);

  /// \brief Pushes a new message into the buffer
  /// \details If buffer size got some space left, we push message regardless of its size, but if
  /// this results in exceeding buffer size or duration limits, we mark buffer to drop all new
  /// incoming messages. This flag is cleared when buffers are swapped.
  /// \param msg Shared pointer to the rosbag2_storage::SerializedBagMessage to add to the buffer.
  /// \return True if message was successfully pushed, otherwise false.
  bool push(CacheBufferInterface::buffer_element_t msg) override;

  /// Clear buffer
  void clear() override;

  /// Get number of elements in the buffer
  size_t size() override;

  /// Get buffer data
  const std::vector<CacheBufferInterface::buffer_element_t> & data() override;

private:
  std::vector<CacheBufferInterface::buffer_element_t> buffer_;

  /// Current size in bytes of messages stored in the buffer
  size_t buffer_bytes_size_ {0u};

  /// Maximum size in bytes of messages allowed to be stored in the buffer.
  const size_t max_bytes_size_;

  /// Maximum duration in nanoseconds of message sequence allowed to be stored in the buffer
  /// Note. If max_cache_duration_ is zero, the buffer will be only bounded by the max_bytes_size_.
  const uint64_t max_cache_duration_ns_;

  /// Set when buffer is full and should drop messages instead of inserting them
  std::atomic_bool drop_messages_ {false};
};

}  // namespace cache
}  // namespace rosbag2_cpp

#ifdef _WIN32
# pragma warning(pop)
#endif

#endif  // ROSBAG2_CPP__CACHE__MESSAGE_CACHE_BUFFER_HPP_
