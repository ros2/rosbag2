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

#ifndef ROSBAG2_CPP__WRITERS__CACHE__MESSAGE_CACHE_BUFFER_HPP_
#define ROSBAG2_CPP__WRITERS__CACHE__MESSAGE_CACHE_BUFFER_HPP_

#include <memory>
#include <vector>

#include "rosbag2_cpp/visibility_control.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"

namespace rosbag2_cpp
{
namespace writers
{
namespace cache
{
// This class implements a single buffer for message cache. Note that it could
// be reused as a template with any class that has ->byte_size() - like interface
class ROSBAG2_CPP_PUBLIC MessageCacheBuffer
{
public:
  explicit MessageCacheBuffer(const uint64_t max_cache_size);

  // If buffer size got some space left, we push message regardless of its size, but if
  // this results in exceeding buffer size, we mark buffer to drop all new incoming messages.
  // This flag is cleared when buffers are swapped.
  bool push(
    const std::shared_ptr<const rosbag2_storage::SerializedBagMessage> & msg);

  // Clear buffer
  void clear();

  // Get number of elements in the buffer
  size_t size();

  // Get buffer data
  std::vector<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>> & data();

private:
  std::vector<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>> buffer_;
  uint64_t buffer_bytes_size_ {0u};
  const uint64_t max_bytes_size_;
  bool drop_messages_ {false};
};

}  // namespace cache
}  // namespace writers
}  // namespace rosbag2_cpp

#endif  // ROSBAG2_CPP__WRITERS__CACHE__MESSAGE_CACHE_BUFFER_HPP_
