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

class ROSBAG2_CPP_PUBLIC MessageCacheInterface
{
public:
  virtual ~MessageCacheInterface() {}

  virtual void push(std::shared_ptr<const rosbag2_storage::SerializedBagMessage> msg) = 0;

  virtual std::shared_ptr<CacheBufferInterface> consumer_buffer() = 0;

  virtual void swap_buffers() = 0;

  virtual void notify_flushing_done() {}

  virtual void log_dropped() {}

  virtual void finalize() {}
};

}  // namespace cache
}  // namespace rosbag2_cpp

#endif  // ROSBAG2_CPP__CACHE__MESSAGE_CACHE_INTERFACE_HPP_
