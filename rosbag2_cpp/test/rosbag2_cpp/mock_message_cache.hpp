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

#ifndef ROSBAG2_CPP__MOCK_MESSAGE_CACHE_HPP_
#define ROSBAG2_CPP__MOCK_MESSAGE_CACHE_HPP_

#include <gmock/gmock.h>

#include <memory>
#include <string>
#include <unordered_map>

#include "rosbag2_cpp/cache/message_cache.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"

class MockMessageCache : public rosbag2_cpp::cache::MessageCache
{
public:
  explicit MockMessageCache(uint64_t max_buffer_size)
  : rosbag2_cpp::cache::MessageCache(max_buffer_size) {}

  MOCK_METHOD0(log_dropped, void());
};

#endif  // ROSBAG2_CPP__MOCK_MESSAGE_CACHE_HPP_
