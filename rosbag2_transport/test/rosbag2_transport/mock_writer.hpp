// Copyright 2018, Bosch Software Innovations GmbH.
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

#ifndef ROSBAG2_TRANSPORT__MOCK_WRITER_HPP_
#define ROSBAG2_TRANSPORT__MOCK_WRITER_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rosbag2/writer.hpp"

class MockWriter : public rosbag2::Writer
{
public:
  void open(
    const rosbag2::StorageOptions & options,
    const std::string & input_serialization_format,
    const std::string & output_serialization_format) override
  {
    (void) options;
    (void) input_serialization_format;
    (void) output_serialization_format;
  }

  void create_topic(const rosbag2::TopicMetadata & topic_with_type) override
  {
    topics_.emplace(topic_with_type.name, topic_with_type);
  }

  void write(std::shared_ptr<rosbag2::SerializedBagMessage> message) override
  {
    messages_.push_back(message);
    messages_per_topic_[message->topic_name] += 1;
  }

  std::vector<std::shared_ptr<rosbag2::SerializedBagMessage>> get_messages()
  {
    return messages_;
  }

  std::map<std::string, size_t> messages_per_topic()
  {
    return messages_per_topic_;
  }

  std::map<std::string, rosbag2::TopicMetadata> get_topics()
  {
    return topics_;
  }

private:
  std::map<std::string, rosbag2::TopicMetadata> topics_;
  std::vector<std::shared_ptr<rosbag2::SerializedBagMessage>> messages_;
  std::map<std::string, size_t> messages_per_topic_;
};

#endif  // ROSBAG2_TRANSPORT__MOCK_WRITER_HPP_
