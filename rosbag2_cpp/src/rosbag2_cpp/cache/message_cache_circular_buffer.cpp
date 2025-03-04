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

#include "rosbag2_cpp/logging.hpp"
#include "rosbag2_cpp/cache/cache_buffer_interface.hpp"
#include "rosbag2_cpp/cache/message_cache_circular_buffer.hpp"

namespace rosbag2_cpp
{
namespace cache
{

MessageCacheCircularBuffer::MessageCacheCircularBuffer(
  size_t max_cache_size,
  int64_t max_cache_duration_ns)
: max_bytes_size_(max_cache_size), max_cache_duration_(max_cache_duration_ns)
{
  if (max_bytes_size_ == 0 && max_cache_duration_ == 0) {
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
  if (buffer_bytes_size_ > 0 && msg->serialized_data->buffer_length > max_bytes_size_) {
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
  // Remove any old items until the difference between last and newest message timestamp
  // will be less than or equal to the max_cache_duration_.
  if (buffer_.size() > 1) {
    auto current_buffer_duration = buffer_.front()->recv_timestamp - buffer_.back()->recv_timestamp;
    while (max_cache_duration_ > 0 && current_buffer_duration > max_cache_duration_) {
      buffer_.pop_front();
      current_buffer_duration = buffer_.front()->recv_timestamp - buffer_.back()->recv_timestamp;
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
