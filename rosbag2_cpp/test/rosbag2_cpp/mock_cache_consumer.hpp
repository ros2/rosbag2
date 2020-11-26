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

#ifndef ROSBAG2_CPP__MOCK_CACHE_CONSUMER_HPP_
#define ROSBAG2_CPP__MOCK_CACHE_CONSUMER_HPP_

#include <gmock/gmock.h>
#include <memory>

#include "mock_message_cache.hpp"
#include "rosbag2_cpp/cache/cache_consumer.hpp"

class MockCacheConsumer : public rosbag2_cpp::cache::CacheConsumer
{
public:
  MockCacheConsumer(
    std::shared_ptr<MockMessageCache> message_cache,
    rosbag2_cpp::cache::CacheConsumer::consume_callback_function_t consume_callback)
  : rosbag2_cpp::cache::CacheConsumer(message_cache, consume_callback) {}
};

#endif  // ROSBAG2_CPP__MOCK_CACHE_CONSUMER_HPP_
