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

#include <deque>
#include <memory>
#include <vector>

#include "rcutils/time.h"
#include "rosbag2_cpp/cache/cache_buffer_interface.hpp"
#include "rosbag2_cpp/cache/message_cache_circular_buffer.hpp"
#include "rosbag2_cpp/logging.hpp"

namespace rosbag2_cpp
{
namespace cache
{

MessageCacheCircularBuffer::MessageCacheCircularBuffer(
  size_t max_cache_size,
  uint32_t max_cache_duration)
: max_bytes_size_(max_cache_size), max_cache_duration_ns_(RCUTILS_S_TO_NS(max_cache_duration))
{
  if (max_bytes_size_ == 0 && max_cache_duration_ns_ == 0) {
    throw std::invalid_argument("Invalid arguments for the MessageCacheCircularBuffer. "
                                "Both max_bytes_size and max_cache_duration are zero.");
  }
}

bool MessageCacheCircularBuffer::push(CacheBufferInterface::buffer_element_t msg)
{
  if (!msg || !msg->serialized_data) {
    ROSBAG2_CPP_LOG_ERROR("Attempted to push null message into circular buffer. Dropping message!");
    return false;
  }

  // Drop message if it exceeds the buffer size
  if (max_bytes_size_ > 0 && msg->serialized_data->buffer_length > max_bytes_size_) {
    ROSBAG2_CPP_LOG_WARN("Last message exceeds snapshot buffer size. Dropping message!");
    return false;
  }

  // Remove any old items until there is room for a new message
  while (max_bytes_size_ > 0 &&
    buffer_bytes_size_ > (max_bytes_size_ - msg->serialized_data->buffer_length))
  {
    buffer_bytes_size_ -= buffer_.front()->serialized_data->buffer_length;
    buffer_.pop_front();
  }
  // Remove old messages until the time span between the oldest and newest message
  // is less than or equal to max_cache_duration_ns_.
  if (max_cache_duration_ns_ > 0 && buffer_.size() > 0) {
    // Note: the oldest messages are at the front of the deque
    auto prospected_buffer_duration = msg->recv_timestamp - buffer_.front()->recv_timestamp;
    if (prospected_buffer_duration < 0) {
      ROSBAG2_CPP_LOG_ERROR_STREAM("Can't calculate prospected circular cache buffer duration: "
        << "oldest message timestamp " << buffer_.front()->recv_timestamp
        << " is earlier than new message timestamp " << msg->recv_timestamp
        << ". Dropping new message!");
      // Note: We drop the new message in this case, because we can't determine
      // whether adding it would exceed the time limit. However, the next message may have
      // a correct timestamp and be added to the buffer.
      return false;
    }

    auto prospected_buffer_duration_exceed_limit = [&]() {
        if (prospected_buffer_duration < 0) {
          ROSBAG2_CPP_LOG_ERROR_STREAM("Can't calculate prospected circular cache buffer duration: "
            << "oldest message timestamp " << buffer_.front()->recv_timestamp
            << " is earlier than new message timestamp " << msg->recv_timestamp
            << ". Dropping oldest message!");
          return true;
        }
        return static_cast<uint64_t>(prospected_buffer_duration) > max_cache_duration_ns_;
      };

    while (buffer_.size() > 0 && prospected_buffer_duration_exceed_limit()) {
      buffer_bytes_size_ -= buffer_.front()->serialized_data->buffer_length;
      buffer_.pop_front();
      // Note: the oldest messages are at the front of the deque
      prospected_buffer_duration = msg->recv_timestamp - buffer_.front()->recv_timestamp;
    }
  }
  // Add a new message to the end of the buffer
  buffer_bytes_size_ += msg->serialized_data->buffer_length;
  buffer_.push_back(msg);

  return true;
}

void MessageCacheCircularBuffer::clear()
{
  buffer_.clear();
  buffer_bytes_size_ = 0u;
}

size_t MessageCacheCircularBuffer::size()
{
  return buffer_.size();
}

const std::vector<CacheBufferInterface::buffer_element_t> & MessageCacheCircularBuffer::data()
{
  // Copy data to vector to maintain same interface as MessageCacheBuffer
  msg_vector_ = std::vector<CacheBufferInterface::buffer_element_t>(
    buffer_.begin(), buffer_.end());
  return msg_vector_;
}

}  // namespace cache
}  // namespace rosbag2_cpp
