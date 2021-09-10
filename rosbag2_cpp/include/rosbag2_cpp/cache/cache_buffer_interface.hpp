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

#ifndef ROSBAG2_CPP__CACHE__CACHE_BUFFER_INTERFACE_HPP_
#define ROSBAG2_CPP__CACHE__CACHE_BUFFER_INTERFACE_HPP_

#include <memory>
#include <vector>

#include "rosbag2_cpp/visibility_control.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"

namespace rosbag2_cpp
{
namespace cache
{
/**
* This class provides the interface for all CacheBuffer implementations. The
* role of these buffers is store messages that are received via topic subscriptions
* before they are written to the bagfile.
*
* Any class that implements CacheBufferInterface is reponsible for handling
* synchronization that is needed for the buffer to operate properly.
*/
class ROSBAG2_CPP_PUBLIC CacheBufferInterface
{
public:
  using buffer_element_t = std::shared_ptr<const rosbag2_storage::SerializedBagMessage>;
  virtual ~CacheBufferInterface() {}

  /**
   *   Pushes a SerializedBagMessage into the cache buffer.
   *
   *   \param msg SerializedBagMessage to add to the buffer.
   *   \return whether message was successfully added to the cache.
   */
  virtual bool push(buffer_element_t msg) = 0;

  /**
   *   Clears the buffer by removing all remaining messages.
   */
  virtual void clear() = 0;

  /**
   *   Check the buffer message count.
   *
   *   \return number of elements in the buffer.
   */
  virtual size_t size() = 0;

  /**
   *   Get the data/messages stored in the buffer. This should only be
   *   called once no more messages are being added to the buffer.
   *
   *   \return a vector containing messages in the buffer.
   */
  virtual const std::vector<buffer_element_t> & data() = 0;
};

}  // namespace cache
}  // namespace rosbag2_cpp

#endif  // ROSBAG2_CPP__CACHE__CACHE_BUFFER_INTERFACE_HPP_
