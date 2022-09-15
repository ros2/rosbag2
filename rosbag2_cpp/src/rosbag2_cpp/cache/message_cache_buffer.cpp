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

#include "rosbag2_cpp/cache/cache_buffer_interface.hpp"
#include "rosbag2_cpp/cache/message_cache_buffer.hpp"

namespace rosbag2_cpp
{
namespace cache
{

MessageCacheBuffer::MessageCacheBuffer(size_t max_cache_size)
: max_bytes_size_(max_cache_size)
{
}

bool MessageCacheBuffer::push(CacheBufferInterface::buffer_element_t msg)
{
  bool pushed = false;
  if (!full_) {
    buffer_bytes_size_ += msg->serialized_data->buffer_length;
    buffer_.push_back(msg);
    pushed = true;
  }

  if (buffer_bytes_size_ >= max_bytes_size_) {
    full_ = true;
  }
  return pushed;
}

void MessageCacheBuffer::clear()
{
  buffer_.clear();
  buffer_bytes_size_ = 0u;
  full_ = false;
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
