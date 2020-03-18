// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#ifndef ROSBAG2_TRANSPORT__RECORDER_HPP_
#define ROSBAG2_TRANSPORT__RECORDER_HPP_

#include <future>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "rosbag2_cpp/writer.hpp"

#include "rosbag2_storage/topic_metadata.hpp"

#include "rosbag2_transport/record_options.hpp"

#include "types.hpp"

namespace rosbag2_cpp
{
class Writer;
}

namespace rosbag2_transport
{

class GenericSubscription;
class Rosbag2Node;

class Recorder
{
public:
  explicit Recorder(std::shared_ptr<rosbag2_cpp::Writer> writer, std::shared_ptr<Rosbag2Node> node);

  void record(const RecordOptions & record_options);

private:
  void topics_discovery(
    std::chrono::milliseconds topic_polling_interval,
    const std::vector<std::string> & requested_topics = {});

  TopicNamesToTypes
  get_requested_or_available_topics(const std::vector<std::string> & requested_topics);

  TopicNamesToTypes
  get_missing_topics(const TopicNamesToTypes & topics);

  void subscribe_topics(const TopicNamesToTypes & topics_and_types);

  void subscribe_topic(const rosbag2_storage::TopicMetadata & topic);

  std::shared_ptr<GenericSubscription> create_subscription(
    const std::string & topic_name, const std::string & topic_type);

  void record_messages() const;

  std::shared_ptr<rosbag2_cpp::Writer> writer_;
  std::shared_ptr<Rosbag2Node> node_;
  std::vector<std::shared_ptr<GenericSubscription>> subscriptions_;
  std::unordered_set<std::string> subscribed_topics_;
  std::string serialization_format_;
};

}  // namespace rosbag2_transport

#endif  // ROSBAG2_TRANSPORT__RECORDER_HPP_
