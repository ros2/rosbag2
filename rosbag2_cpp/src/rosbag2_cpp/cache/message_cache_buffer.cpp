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

#include <memory>
#include <vector>

#include "rcutils/time.h"
#include "rosbag2_cpp/cache/cache_buffer_interface.hpp"
#include "rosbag2_cpp/cache/message_cache_buffer.hpp"
#include "rosbag2_cpp/logging.hpp"

namespace rosbag2_cpp
{
namespace cache
{
MessageCacheBuffer::MessageCacheBuffer(size_t max_cache_size, uint32_t max_cache_duration)
: max_bytes_size_(max_cache_size), max_cache_duration_ns_(RCUTILS_S_TO_NS(max_cache_duration))
{
  if (max_bytes_size_ == 0 && max_cache_duration_ns_ == 0) {
    throw std::invalid_argument("Invalid arguments for the MessageCacheBuffer. "
                                "Both max_bytes_size and max_cache_duration are zero.");
  }
  buffer_.reserve(512);  // Reserve some space to avoid reallocations. 16x512 = 8192 bytes.
}

bool MessageCacheBuffer::push(CacheBufferInterface::buffer_element_t msg)
{
  if (!msg || !msg->serialized_data) {
    ROSBAG2_CPP_LOG_ERROR("Attempted to push null message into cache buffer. Dropping message!");
    return false;
  }

  rcutils_time_point_value_t prospected_buffer_duration = 0;
  // Calculate the prospected buffer duration if we add this message to the buffer in case of
  // time-limited buffering.
  if (max_cache_duration_ns_ > 0 && buffer_.size() > 0) {  //  If we have at least 1 message
    // Note: the oldest messages are at the front of the vector
    prospected_buffer_duration = msg->recv_timestamp - buffer_.front()->recv_timestamp;
    if (prospected_buffer_duration < 0) {
      ROSBAG2_CPP_LOG_ERROR_STREAM("Can't calculate prospected cache buffer duration: "
        << "oldest message timestamp " << buffer_.front()->recv_timestamp
        << " is earlier than new message timestamp " << msg->recv_timestamp
        << ". Dropping new message!");
      // Note: We drop the new message in this case, because we can't determine
      // whether adding it would exceed the time limit. However, the next message may have
      // a correct timestamp and be added to the buffer.
      return false;
    }
  }

  bool pushed = false;

  if (!drop_messages_) {
    // Check if cache is limited by time and adding this message would exceed the time limit
    if (max_cache_duration_ns_ > 0) {
      // Note: The casting to uint64_t is safe here, as we have already checked that
      // prospected_buffer_duration is not negative.
      if (static_cast<uint64_t>(prospected_buffer_duration) > max_cache_duration_ns_) {
        drop_messages_ = true;
      }
    }

    // Note: Allow at least one message to be added even if it exceeds the size limit.
    if (max_bytes_size_ > 0 && !buffer_.empty()) {
      if (buffer_bytes_size_ + msg->serialized_data->buffer_length > max_bytes_size_) {
        drop_messages_ = true;
      }
    }

    if (!drop_messages_) {
      buffer_bytes_size_ += msg->serialized_data->buffer_length;
      buffer_.push_back(msg);
      pushed = true;
    }
  }
  return pushed;
}

void MessageCacheBuffer::clear()
{
  buffer_.clear();
  buffer_bytes_size_ = 0u;
  drop_messages_ = false;
}

size_t MessageCacheBuffer::size()
{
  return buffer_.size();
}

const std::vector<CacheBufferInterface::buffer_element_t> & MessageCacheBuffer::data()
{
  return buffer_;
}

}  // namespace cache
}  // namespace rosbag2_cpp
