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

#ifndef ROSBAG2_CPP__CACHE__MESSAGE_CACHE_INTERFACE_HPP_
#define ROSBAG2_CPP__CACHE__MESSAGE_CACHE_INTERFACE_HPP_

#include <memory>

#include "rosbag2_cpp/visibility_control.hpp"
#include "rosbag2_cpp/cache/cache_buffer_interface.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"

namespace rosbag2_cpp
{
namespace cache
{
/**
* This class provides the interface for all MessageCache implementations.
* The interface is designed around double buffered MessageCache implementations
* due to the inherent performance benefit that they provide.
*
* Any class that implements the MessageCacheInterface should make use of a class
* derived from the CacheBufferInterface as its buffer(s). Any synchronization that
* is required to manage more than one CacheBuffer should be handled in the class that
* implements this interface.
*/
class ROSBAG2_CPP_PUBLIC MessageCacheInterface
{
public:
  virtual ~MessageCacheInterface() {}

  /// Push a bag message into the producer buffer.
  virtual void push(std::shared_ptr<const rosbag2_storage::SerializedBagMessage> msg) = 0;

  /// Get a pointer to the buffer that can be used for consuming the cached messages.
  /// This call locks access to the buffer, `swap_buffers` and `consumer_buffer` will block until
  /// `release_consumer_buffer` is called to unlock access to the buffer.
  /// Consumer should call `release_consumer_buffer` when they are done consuming the messages.
  /// \return a pointer to the consumer buffer interface.
  virtual std::shared_ptr<CacheBufferInterface> consumer_buffer() = 0;

  /// Signals that tne consumer is done consuming, unlocking the buffer so it may be swapped.
  virtual void release_consumer_buffer() = 0;

  /// Swap producer and consumer buffers.
  /// Note: this will block if `consumer_buffer` has been called but `release_consumer_buffer`
  /// has not been called yet to signal end of consuming.
  virtual void swap_buffers() = 0;

  /// Go into a read-only state to drain the final consumer buffer without letting in new data.
  virtual void begin_flushing() {}

  /// Call after `begin_flushing`, once the final consumer buffer is emptied.
  virtual void done_flushing() {}

  /// Print a log message with details of any dropped messages.
  virtual void log_dropped() {}
};

}  // namespace cache
}  // namespace rosbag2_cpp

#endif  // ROSBAG2_CPP__CACHE__MESSAGE_CACHE_INTERFACE_HPP_
