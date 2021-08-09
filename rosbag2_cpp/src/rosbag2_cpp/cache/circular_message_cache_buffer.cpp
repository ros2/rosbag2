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

#include <memory>
#include <list>

#include "rosbag2_cpp/cache/circular_message_cache_buffer.hpp"

namespace rosbag2_cpp
{
namespace cache
{
/**
* This class implements a circular buffer message cache. Since the buffer
* size is limited by total byte size of the storage messages rather than
* a fix number of messages, a linked list is used instead of a vector since
* older messages can always be dropped from the front and new messages added
* to the end. This buffer may consume more byte than max_cache_size, however
* it will only exceed the limit by the excess data in the most recent message.
*/
CircularMessageCacheBuffer::CircularMessageCacheBuffer(const uint64_t max_cache_size)
: max_bytes_size_(max_cache_size)
{
}

void CircularMessageCacheBuffer::push(buffer_element_t msg)
{
  if (buffer_full_) {
    // Remove old items until there is room for new message
    while(buffer_bytes_size_ >= max_bytes_size_ + msg->serialized_data->buffer_length) {
      buffer_bytes_size_ -= buffer_.front()->serialized_data->buffer_length;
      buffer_.pop_front();
    }
  }
  // Add new message to end of buffer
  buffer_bytes_size_ += msg->serialized_data->buffer_length;
  buffer_.push_back(msg);

  if (buffer_bytes_size_ >= max_bytes_size_) {
    buffer_full_ = true;
  }
}

void CircularMessageCacheBuffer::clear()
{
  buffer_.clear();
  buffer_bytes_size_ = 0u;
  buffer_full_ = false;
}

size_t CircularMessageCacheBuffer::size()
{
  return buffer_.size();
}

const std::list<CircularMessageCacheBuffer::buffer_element_t> & CircularMessageCacheBuffer::data()
{
  return buffer_;
}

}  // namespace cache
}  // namespace rosbag2_cpp
