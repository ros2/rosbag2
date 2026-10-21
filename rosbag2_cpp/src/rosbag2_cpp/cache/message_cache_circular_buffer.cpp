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

#include <algorithm>
#include <deque>
#include <memory>
#include <stdexcept>
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

  // A message may arrive with a receive timestamp slightly older than the newest cached one.
  // If it is still inside the retained window it is accepted below. One older than the whole
  // window would be evicted right away, so reject it up front; such a large negative delta
  // usually indicates a clock jump such as a sim-time reset.
  if (max_cache_duration_ns_ > 0 && !buffer_.empty() &&
    msg->recv_timestamp < buffer_.back()->recv_timestamp &&
    static_cast<uint64_t>(buffer_.back()->recv_timestamp) -
    static_cast<uint64_t>(msg->recv_timestamp) > max_cache_duration_ns_)
  {
    ROSBAG2_CPP_LOG_WARN_STREAM("Late message is outside the circular cache duration: "
      << "message timestamp " << msg->recv_timestamp
      << " is older than newest cached message timestamp " << buffer_.back()->recv_timestamp
      << " by more than the cache duration. Dropping message!");
    return false;
  }

  // Keep the deque ordered by receive timestamp, not arrival order, so that front() is always
  // the oldest sample and duration eviction can be measured against back().
  auto insertion_point = std::upper_bound(
    buffer_.begin(), buffer_.end(), msg->recv_timestamp,
    [](int64_t timestamp, const auto & buffered) {
      return timestamp < buffered->recv_timestamp;
    });
  buffer_.insert(insertion_point, msg);
  buffer_bytes_size_ += msg->serialized_data->buffer_length;

  // Retain the most recent timestamp window, including slightly late arrivals.
  // Messages outside that window are evicted from the front, which is now
  // guaranteed to contain the earliest timestamp.
  if (max_cache_duration_ns_ > 0) {
    const auto newest_timestamp = buffer_.back()->recv_timestamp;
    while (!buffer_.empty() &&
      static_cast<uint64_t>(newest_timestamp) -
      static_cast<uint64_t>(buffer_.front()->recv_timestamp) > max_cache_duration_ns_)
    {
      buffer_bytes_size_ -= buffer_.front()->serialized_data->buffer_length;
      buffer_.pop_front();
    }
  }

  // Enforce the byte limit after insertion so a late arrival cannot evict
  // newer messages before it is itself rejected by the duration window.
  while (max_bytes_size_ > 0 && buffer_bytes_size_ > max_bytes_size_) {
    buffer_bytes_size_ -= buffer_.front()->serialized_data->buffer_length;
    buffer_.pop_front();
  }

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
