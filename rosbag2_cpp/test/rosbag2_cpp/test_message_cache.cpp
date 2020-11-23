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

#include <gmock/gmock.h>

#include <chrono>
#include <numeric>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include <thread>

#include "rosbag2_storage/ros_helper.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"

#include "mock_cache_consumer.hpp"
#include "mock_message_cache.hpp"

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

uint32_t sum_up(const std::unordered_map<std::string, uint32_t> & map)
{
  return std::accumulate(
    std::begin(map),
    std::end(map),
    0,
    [](const std::size_t previous, const auto & element) {
      return previous + element.second;
    }
  );
}
}  // namespace

class MessageCacheTest : public Test
{
public:
  MessageCacheTest() {}

  virtual ~MessageCacheTest() = default;

  const uint64_t cache_size_ {1 * 1000};  // ~1 Kb cache
};

TEST_F(MessageCacheTest, message_cache_writes_full_producer_buffer) {
  const uint32_t message_count = 100;
  uint64_t size_bytes_so_far = 0;
  uint32_t should_be_dropped_count = 0;
  uint32_t consumed_message_count {0};

  auto mock_message_cache = std::make_shared<NiceMock<MockMessageCache>>(
    cache_size_);

  for (uint32_t i = 0; i < message_count; ++i) {
    auto msg = make_test_msg();
    size_t serialized_data_size = msg->serialized_data->buffer_length;
    mock_message_cache->push(msg);
    if (cache_size_ < size_bytes_so_far) {
      should_be_dropped_count++;
    }
    size_bytes_so_far += serialized_data_size;
  }

  auto total_actually_dropped = sum_up(mock_message_cache->messages_dropped());
  EXPECT_EQ(should_be_dropped_count, total_actually_dropped);

  auto cb = [&consumed_message_count](
    const std::vector<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>> & msgs) {
      consumed_message_count += msgs.size();
    };

  auto mock_cache_consumer = std::make_unique<NiceMock<MockCacheConsumer>>(
    mock_message_cache,
    cb);

  using namespace std::chrono_literals;
  std::this_thread::sleep_for(20ms);

  mock_cache_consumer->close();
  EXPECT_EQ(consumed_message_count, message_count - should_be_dropped_count);
}

TEST_F(MessageCacheTest, message_cache_changing_callback) {
  const uint32_t message_count = 50;

  size_t callback1_counter = 0;
  size_t callback2_counter = 0;

  auto mock_message_cache = std::make_shared<NiceMock<MockMessageCache>>(
    cache_size_);

  auto mock_cache_consumer = std::make_unique<NiceMock<MockCacheConsumer>>(
    mock_message_cache,
    [&callback1_counter](
      const std::vector<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>> & msgs) {
      callback1_counter += msgs.size();
    });

  rosbag2_cpp::cache::CacheConsumer::consume_callback_function_t cb2 =
    [&callback2_counter](
    const std::vector<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>> & msgs) {
      callback2_counter += msgs.size();
    };

  uint32_t counter = 0;
  for (uint32_t i = 0; i < message_count; ++i) {
    if (counter >= message_count / 2) {
      mock_cache_consumer->change_consume_callback(cb2);
    }
    auto msg = make_test_msg();
    mock_message_cache->push(msg);
    counter++;
  }

  auto total_actually_dropped = sum_up(mock_message_cache->messages_dropped());
  EXPECT_EQ(total_actually_dropped, 0u);

  mock_cache_consumer->close();

  using namespace std::chrono_literals;
  std::this_thread::sleep_for(20ms);

  uint32_t sum_consumed = callback1_counter + callback2_counter;

  EXPECT_EQ(sum_consumed, message_count);
}
