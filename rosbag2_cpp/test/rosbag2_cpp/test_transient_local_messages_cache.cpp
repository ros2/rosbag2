// Copyright 2026 Dexory
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

#include <memory>
#include <string>
#include <vector>

#include "rosbag2_cpp/cache/transient_local_messages_cache.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"

using namespace testing;  // NOLINT

namespace
{
std::shared_ptr<rosbag2_storage::SerializedBagMessage> make_message(
  const std::string & topic,
  rcutils_time_point_value_t recv_timestamp,
  rcutils_time_point_value_t send_timestamp)
{
  auto message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  message->topic_name = topic;
  message->recv_timestamp = recv_timestamp;
  message->send_timestamp = send_timestamp;
  return message;
}
}  // namespace

TEST(TransientLocalMessagesCacheTest, keeps_only_last_n_messages_per_topic)
{
  rosbag2_cpp::cache::TransientLocalMessagesCache cache;
  cache.add_topic("/map", 2);

  cache.push("/map", make_message("/map", 1, 1));
  cache.push("/map", make_message("/map", 2, 2));
  cache.push("/map", make_message("/map", 3, 3));

  auto messages = cache.get_messages_sorted_by_timestamp();
  ASSERT_THAT(messages.size(), Eq(2u));
  EXPECT_THAT(messages[0]->recv_timestamp, Eq(2));
  EXPECT_THAT(messages[1]->recv_timestamp, Eq(3));
}

TEST(TransientLocalMessagesCacheTest, returns_messages_sorted_across_topics)
{
  rosbag2_cpp::cache::TransientLocalMessagesCache cache;
  cache.add_topic("/tf_static", 2);
  cache.add_topic("/map", 2);

  cache.push("/tf_static", make_message("/tf_static", 30, 30));
  cache.push("/map", make_message("/map", 10, 10));
  cache.push("/tf_static", make_message("/tf_static", 20, 20));

  auto messages = cache.get_messages_sorted_by_timestamp();
  ASSERT_THAT(messages.size(), Eq(3u));
  EXPECT_THAT(messages[0]->recv_timestamp, Eq(10));
  EXPECT_THAT(messages[1]->recv_timestamp, Eq(20));
  EXPECT_THAT(messages[2]->recv_timestamp, Eq(30));
}

TEST(TransientLocalMessagesCacheTest, throws_on_push_to_unregistered_topic)
{
  rosbag2_cpp::cache::TransientLocalMessagesCache cache;
  cache.add_topic("/tf_static", 1);

  EXPECT_THROW(
    cache.push("/map", make_message("/map", 1, 1)),
    std::runtime_error);

  auto messages = cache.get_messages_sorted_by_timestamp();
  EXPECT_THAT(messages, IsEmpty());
}
