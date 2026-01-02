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

#include "rosbag2_cpp/logging.hpp"
#include "rosbag2_cpp/cache/cache_buffer_interface.hpp"
#include "rosbag2_cpp/cache/message_cache_buffer.hpp"

namespace rosbag2_cpp
{
namespace cache
{
MessageCacheBuffer::MessageCacheBuffer(size_t max_cache_size, int64_t max_cache_duration_ns)
: max_bytes_size_(max_cache_size), max_cache_duration_(max_cache_duration_ns)
{
  if (max_bytes_size_ == 0 && max_cache_duration_ == 0) {
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

  if (max_cache_duration_ > 0 && buffer_.size() > 1) {  // If we have at least 2 messages
    auto current_buffer_duration = buffer_.front()->recv_timestamp - buffer_.back()->recv_timestamp;
    if (current_buffer_duration > max_cache_duration_) {
      drop_messages_ = true;
    }
  }

  bool pushed = false;
  if (!drop_messages_) {
    buffer_bytes_size_ += msg->serialized_data->buffer_length;
    buffer_.push_back(msg);
    pushed = true;
  }

  if (max_bytes_size_ > 0 && buffer_bytes_size_ >= max_bytes_size_) {
    drop_messages_ = true;
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
