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

#ifndef ROSBAG2_TRANSPORT__MOCK_SEQUENTIAL_WRITER_HPP_
#define ROSBAG2_TRANSPORT__MOCK_SEQUENTIAL_WRITER_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rosbag2_cpp/writer_interfaces/base_writer_interface.hpp"

class MockSequentialWriter : public rosbag2_cpp::writer_interfaces::BaseWriterInterface
{
public:
  void open(
    const rosbag2_storage::StorageOptions & storage_options,
    const rosbag2_cpp::ConverterOptions & converter_options) override
  {
    snapshot_mode_ = storage_options.snapshot_mode;
    (void) storage_options;
    (void) converter_options;
  }

  void close() override {}

  void create_topic(const rosbag2_storage::TopicMetadata & topic_with_type) override
  {
    topics_.emplace(topic_with_type.name, topic_with_type);
  }

  void remove_topic(const rosbag2_storage::TopicMetadata & topic_with_type) override
  {
    (void) topic_with_type;
  }

  void write(std::shared_ptr<rosbag2_storage::SerializedBagMessage> message) override
  {
    if (!snapshot_mode_) {
      messages_.push_back(message);
    } else {
      snapshot_buffer_.push_back(message);
    }
    messages_per_topic_[message->topic_name] += 1;
  }

  bool take_snapshot() override
  {
    std::swap(snapshot_buffer_, messages_);
    snapshot_buffer_.clear();
    return true;
  }

  void add_event_callbacks(rosbag2_cpp::bag_events::WriterEventCallbacks & /* callbacks */)
  {
    // TODO(gbiggs): Figure out how to mock this appropriately
  }

  const std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> & get_messages()
  {
    return messages_;
  }

  const std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> & get_snapshot_buffer()
  {
    return snapshot_buffer_;
  }

  const std::unordered_map<std::string, size_t> & messages_per_topic()
  {
    return messages_per_topic_;
  }

  const std::unordered_map<std::string, rosbag2_storage::TopicMetadata> & get_topics()
  {
    return topics_;
  }

private:
  std::unordered_map<std::string, rosbag2_storage::TopicMetadata> topics_;
  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages_;
  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> snapshot_buffer_;
  std::unordered_map<std::string, size_t> messages_per_topic_;
  bool snapshot_mode_ = false;
};

#endif  // ROSBAG2_TRANSPORT__MOCK_SEQUENTIAL_WRITER_HPP_
