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

#ifndef ROSBAG2_TRANSPORT__MOCK_SEQUENTIAL_READER_HPP_
#define ROSBAG2_TRANSPORT__MOCK_SEQUENTIAL_READER_HPP_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rosbag2_cpp/reader_interfaces/base_reader_interface.hpp"

class MockSequentialReader : public rosbag2_cpp::reader_interfaces::BaseReaderInterface
{
public:
  void open(
    const rosbag2_storage::StorageOptions & storage_options,
    const rosbag2_cpp::ConverterOptions & converter_options) override
  {
    (void) storage_options;
    (void) converter_options;
    num_read_ = 0;
  }

  void close() override {}

  bool set_read_order(const rosbag2_storage::ReadOrder &) override
  {
    return true;
  }

  bool has_next() override
  {
    if (filter_.topics.empty()) {
      return num_read_ < messages_.size();
    }

    while (num_read_ < messages_.size()) {
      for (const auto & filter_topic : filter_.topics) {
        if (!messages_[num_read_]->topic_name.compare(filter_topic)) {
          return true;
        }
      }
      num_read_++;
    }
    return false;
  }

  std::shared_ptr<rosbag2_storage::SerializedBagMessage> read_next() override
  {
    // "Split" the bag every few messages
    if (num_read_ > 0 && num_read_ % max_messages_per_file_ == 0) {
      auto info = std::make_shared<rosbag2_cpp::bag_events::BagSplitInfo>();
      info->closed_file = "BagFile" + std::to_string(file_number_);
      file_number_++;
      info->opened_file = "BagFile" + std::to_string(file_number_);
      callback_manager_.execute_callbacks(rosbag2_cpp::bag_events::BagEvent::READ_SPLIT, info);
    }
    // filter_ was considered when incrementing num_read_ in has_next()
    return messages_[num_read_++];
  }

  const rosbag2_storage::BagMetadata & get_metadata() const override
  {
    return metadata_;
  }

  std::vector<rosbag2_storage::TopicMetadata> get_all_topics_and_types() const override
  {
    return topics_;
  }

  void get_all_message_definitions(std::vector<rosbag2_storage::MessageDefinition> & definitions)
  override
  {
    definitions.clear();
  }

  void set_filter(const rosbag2_storage::StorageFilter & storage_filter) override
  {
    filter_ = storage_filter;
  }

  void reset_filter() override
  {
    filter_ = rosbag2_storage::StorageFilter();
  }

  void seek(const rcutils_time_point_value_t & timestamp) override
  {
    seek_time_ = timestamp;
    num_read_ = 0;
  }

  void
  add_event_callbacks(const rosbag2_cpp::bag_events::ReaderEventCallbacks & callbacks) override
  {
    if (callbacks.read_split_callback) {
      callback_manager_.add_event_callback(
        callbacks.read_split_callback,
        rosbag2_cpp::bag_events::BagEvent::READ_SPLIT);
    }
  }

  void prepare(
    std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages,
    std::vector<rosbag2_storage::TopicMetadata> topics)
  {
    metadata_.message_count = messages.size();
    if (!messages.empty()) {
      const auto message_timestamp = std::chrono::time_point<std::chrono::high_resolution_clock>(
        std::chrono::nanoseconds(messages[0]->time_stamp));
      metadata_.starting_time = message_timestamp;
    }
    messages_ = std::move(messages);
    topics_ = std::move(topics);
  }

  size_t max_messages_per_file() const
  {
    return max_messages_per_file_;
  }

private:
  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages_;
  rosbag2_storage::BagMetadata metadata_;
  std::vector<rosbag2_storage::TopicMetadata> topics_;
  size_t num_read_;
  rcutils_time_point_value_t seek_time_ = 0;
  rosbag2_storage::StorageFilter filter_;
  rosbag2_cpp::bag_events::EventCallbackManager callback_manager_;
  size_t file_number_ = 0;
  const size_t max_messages_per_file_ = 5;
};

#endif  // ROSBAG2_TRANSPORT__MOCK_SEQUENTIAL_READER_HPP_
