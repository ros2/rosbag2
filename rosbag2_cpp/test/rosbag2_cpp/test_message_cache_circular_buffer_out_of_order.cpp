// Copyright 2026 Abel Mendoza
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

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <memory>
#include <string>

#include "rosbag2_cpp/cache/message_cache_circular_buffer.hpp"
#include "rosbag2_storage/ros_helper.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"

namespace
{
std::shared_ptr<rosbag2_storage::SerializedBagMessage> message_at(int64_t timestamp)
{
  auto message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  message->topic_name = "sensor";
  message->recv_timestamp = timestamp;
  const std::string payload = "sample";
  message->serialized_data =
    rosbag2_storage::make_serialized_message(payload.c_str(), payload.size());
  return message;
}
}  // namespace

TEST(MessageCacheCircularBufferOutOfOrder, retains_slightly_late_message)
{
  rosbag2_cpp::cache::MessageCacheCircularBuffer cache(0, 2);
  ASSERT_TRUE(cache.push(message_at(10'000'000'000LL)));
  EXPECT_TRUE(cache.push(message_at(9'999'782'000LL)));
  EXPECT_EQ(cache.size(), 2u);
}

TEST(MessageCacheCircularBufferOutOfOrder, respects_duration_with_out_of_order_arrivals)
{
  rosbag2_cpp::cache::MessageCacheCircularBuffer cache(0, 2);
  ASSERT_TRUE(cache.push(message_at(10'000'000'000LL)));
  ASSERT_TRUE(cache.push(message_at(11'900'000'000LL)));
  ASSERT_TRUE(cache.push(message_at(10'500'000'000LL)));
  ASSERT_TRUE(cache.push(message_at(12'500'000'000LL)));

  const auto & messages = cache.data();
  ASSERT_FALSE(messages.empty());
  const auto [min_msg, max_msg] = std::minmax_element(
    messages.begin(), messages.end(),
    [](const auto & a, const auto & b) {
      return a->recv_timestamp < b->recv_timestamp;
    });
  EXPECT_LE((*max_msg)->recv_timestamp - (*min_msg)->recv_timestamp, 2'000'000'000LL);
}
