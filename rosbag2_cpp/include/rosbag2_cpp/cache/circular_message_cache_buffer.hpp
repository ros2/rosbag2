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

#ifndef ROSBAG2_CPP__CACHE__CIRCULAR_MESSAGE_CACHE_BUFFER_HPP_
#define ROSBAG2_CPP__CACHE__CIRCULAR_MESSAGE_CACHE_BUFFER_HPP_

#include <memory>
#include <list>

#include "rosbag2_cpp/visibility_control.hpp"
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

class ROSBAG2_CPP_PUBLIC CircularMessageCacheBuffer
{
public:
  explicit CircularMessageCacheBuffer(const uint64_t max_cache_size);

  using buffer_element_t = std::shared_ptr<const rosbag2_storage::SerializedBagMessage>;

  /**
  * If buffer size has some space left, we push the message regardless of its size,
  *  but if this results in exceeding buffer size, we begin dropping old messages.
  */
  void push(buffer_element_t msg);

  /// Clear buffer
  void clear();

  /// Get number of elements in the buffer
  size_t size();

  /// Get buffer data
  const std::list<buffer_element_t> & data();

private:
  std::list<buffer_element_t> buffer_;
  uint64_t buffer_bytes_size_ {0u};
  const uint64_t max_bytes_size_;
};

}  // namespace cache
}  // namespace rosbag2_cpp

#ifdef _WIN32
# pragma warning(pop)
#endif

#endif  // ROSBAG2_CPP__CACHE__CIRCULAR_MESSAGE_CACHE_BUFFER_HPP_
