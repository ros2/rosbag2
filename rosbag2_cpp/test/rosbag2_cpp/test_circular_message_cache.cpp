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

#include <gmock/gmock.h>

#include <chrono>
#include <cmath>
#include <numeric>
#include <memory>
#include <string>
#include <utility>
#include <list>
#include <thread>

#include "rosbag2_storage/ros_helper.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"

#include "rosbag2_cpp/cache/circular_message_cache.hpp"

using namespace testing;  // NOLINT

namespace
{
std::shared_ptr<rosbag2_storage::SerializedBagMessage> make_test_msg()
{
  static uint32_t counter = 0;
  std::string msg_content = "Hello" + std::to_string(counter++);
  auto msg_length = msg_content.length();
  auto message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  message->topic_name = "test_topic";
  message->serialized_data = rosbag2_storage::make_serialized_message(
    msg_content.c_str(), msg_length);
  return message;
}

std::string deserialize_message(std::shared_ptr<rcutils_uint8_array_t> serialized_message)
{
  std::unique_ptr<uint8_t[]> copied(new uint8_t[serialized_message->buffer_length + 1]);
  std::copy(
    serialized_message->buffer,
    serialized_message->buffer + serialized_message->buffer_length,
    copied.get());
  copied.get()[serialized_message->buffer_length] = '\0';
  std::string message_content(reinterpret_cast<char *>(copied.get()));
  return message_content;
}
}  // namespace

class CircularMessageCacheTest : public Test
{
public:
  CircularMessageCacheTest() {}

  virtual ~CircularMessageCacheTest() = default;

  const uint64_t cache_size_ {1 * 500};  // ~0.5 Kb cache
};

TEST_F(CircularMessageCacheTest, circular_message_cache_overwrites_old) {
  const uint32_t message_count = 100;
  uint64_t size_bytes_so_far = 0;

  auto circular_message_cache = std::make_shared<rosbag2_cpp::cache::CircularMessageCache>(
    cache_size_);

  for (uint32_t i = 0; i < message_count; ++i) {
    auto msg = make_test_msg();
    size_t serialized_data_size = msg->serialized_data->buffer_length;
    circular_message_cache->push(msg);
    size_bytes_so_far += serialized_data_size;
  }
  // Swap cache
  circular_message_cache->swap_buffers();

  auto consumer_buffer = circular_message_cache->consumer_buffer();
  auto message_list = consumer_buffer->data();
  std::string first_message = deserialize_message(message_list.front()->serialized_data);

  // Old messages should be dropped
  EXPECT_THAT(first_message, StrNe("Hello0"));

  uint64_t message_data_size = 0;

  for (auto it = message_list.begin(); it != message_list.end(); it++) {
    message_data_size += (*it)->serialized_data->buffer_length;
  }

  int cache_size_diff = cache_size_ - message_data_size;

  // Actual stored data size should be roughly the desired cache size
  EXPECT_THAT(std::abs(cache_size_diff), Lt(10));
}

TEST_F(CircularMessageCacheTest, circular_message_cache_ensure_empty) {
  const uint32_t message_count = 100;
  uint64_t size_bytes_so_far = 0;

  auto circular_message_cache = std::make_shared<rosbag2_cpp::cache::CircularMessageCache>(
    cache_size_);

  for (uint32_t i = 0; i < message_count; ++i) {
    auto msg = make_test_msg();
    size_t serialized_data_size = msg->serialized_data->buffer_length;
    circular_message_cache->push(msg);
    size_bytes_so_far += serialized_data_size;
  }
  // Swap filled cache to secondary
  circular_message_cache->swap_buffers();
  EXPECT_THAT(circular_message_cache->consumer_buffer()->size(), Ne(0u));

  // Swap back to primary (expected to empty buffer)
  circular_message_cache->swap_buffers();

  // Swap back to secondary without adding messages
  circular_message_cache->swap_buffers();
  // Cache should have been emptied
  EXPECT_THAT(circular_message_cache->consumer_buffer()->size(), Eq(0u));
}
