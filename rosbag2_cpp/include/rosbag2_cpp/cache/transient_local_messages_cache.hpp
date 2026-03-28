// Copyright 2026 Dexory (Tony Najjar)
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

#ifndef ROSBAG2_CPP__CACHE__TRANSIENT_LOCAL_MESSAGES_CACHE_HPP_
#define ROSBAG2_CPP__CACHE__TRANSIENT_LOCAL_MESSAGES_CACHE_HPP_

#include <cstddef>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "rosbag2_cpp/visibility_control.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"

namespace rosbag2_cpp
{
namespace cache
{

class ROSBAG2_CPP_PUBLIC TransientLocalMessagesCache
{
public:
  using MessageSharedPtr = std::shared_ptr<const rosbag2_storage::SerializedBagMessage>;

  void add_topic(const std::string & topic_name, size_t queue_depth);
  void remove_topic(const std::string & topic_name);
  bool has_topic(const std::string & topic_name) const;

  void push(const std::string & topic_name, MessageSharedPtr message);

  std::vector<MessageSharedPtr> get_messages_sorted_by_timestamp() const;

  void clear();
  size_t size() const;

private:
  struct TopicQueue
  {
    size_t max_depth{0};
    std::deque<MessageSharedPtr> messages;
  };

  mutable std::mutex mutex_;
  std::unordered_map<std::string, TopicQueue> topic_queues_;
};

}  // namespace cache
}  // namespace rosbag2_cpp

#endif  // ROSBAG2_CPP__CACHE__TRANSIENT_LOCAL_MESSAGES_CACHE_HPP_
