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

#ifndef ROSBAG2_CPP__CACHE_INTERFACES__BASE_CACHE_BUFFER_INTERFACE_HPP_
#define ROSBAG2_CPP__CACHE_INTERFACES__BASE_CACHE_BUFFER_INTERFACE_HPP_

#include <memory>
#include <vector>

#include "rosbag2_cpp/visibility_control.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"

namespace rosbag2_cpp
{
using buffer_element_t = std::shared_ptr<const rosbag2_storage::SerializedBagMessage>;
namespace cache_interfaces
{

class ROSBAG2_CPP_PUBLIC BaseCacheBufferInterface
{
public:
  virtual ~BaseCacheBufferInterface() {}

  virtual bool push(buffer_element_t msg) = 0;

  virtual void clear() = 0;

  virtual size_t size() = 0;

  virtual const std::vector<buffer_element_t> & data() = 0;
};

}  // namespace cache_interfaces
}  // namespace rosbag2_cpp

#endif  // ROSBAG2_CPP__CACHE_INTERFACES__BASE_CACHE_BUFFER_INTERFACE_HPP_
