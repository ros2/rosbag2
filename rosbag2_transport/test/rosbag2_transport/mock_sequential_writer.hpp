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
    storage_options_ = storage_options;
    (void) converter_options;
    writer_close_called_ = false;
  }

  void close() override
  {
    writer_close_called_ = true;
  }

  void create_topic(const rosbag2_storage::TopicMetadata & topic_with_type) override
  {
    auto message_definition = rosbag2_storage::MessageDefinition::empty_message_definition_for(
      topic_with_type.type);
    topics_.emplace(topic_with_type.name, std::make_pair(topic_with_type, message_definition));
  }

  void create_topic(
    const rosbag2_storage::TopicMetadata & topic_with_type,
    const rosbag2_storage::MessageDefinition & message_definition) override
  {
    topics_.emplace(topic_with_type.name, std::make_pair(topic_with_type, message_definition));
  }

  void remove_topic(const rosbag2_storage::TopicMetadata & topic_with_type) override
  {
    (void) topic_with_type;
  }

  void write(std::shared_ptr<const rosbag2_storage::SerializedBagMessage> message) override
  {
    std::lock_guard<std::mutex> lock(messages_mutex_);
    if (!storage_options_.snapshot_mode) {
      messages_.push_back(message);
    } else {
      snapshot_buffer_.push_back(message);
    }
    messages_per_topic_[message->topic_name] += 1;
    messages_per_file_ += 1;
    if (messages_per_file_ == max_messages_per_file_) {  // "Split" the bag every few messages
      this->split_bagfile();
    }
  }

  bool take_snapshot() override
  {
    std::lock_guard<std::mutex> lock(messages_mutex_);
    std::swap(snapshot_buffer_, messages_);
    snapshot_buffer_.clear();
    return true;
  }

  void split_bagfile() override
  {
    auto info = std::make_shared<rosbag2_cpp::bag_events::BagSplitInfo>();
    info->closed_file = "BagFile" + std::to_string(file_number_);
    file_number_ += 1;
    info->opened_file = "BagFile" + std::to_string(file_number_);
    callback_manager_.execute_callbacks(rosbag2_cpp::bag_events::BagEvent::WRITE_SPLIT, info);
    messages_per_file_ = 0;
  }

  void
  add_event_callbacks(const rosbag2_cpp::bag_events::WriterEventCallbacks & callbacks) override
  {
    if (callbacks.write_split_callback) {
      callback_manager_.add_event_callback(
        callbacks.write_split_callback,
        rosbag2_cpp::bag_events::BagEvent::WRITE_SPLIT);
    }
  }

  bool has_callback_for_event(rosbag2_cpp::bag_events::BagEvent event) const override
  {
    return callback_manager_.has_callback_for_event(event);
  }

  size_t get_number_of_recorded_messages() const
  {
    std::lock_guard<std::mutex> lock(messages_mutex_);
    return messages_.size();
  }

  std::vector<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>> get_messages()
  {
    std::lock_guard<std::mutex> lock(messages_mutex_);
    auto copy_of_messages = messages_;
    return copy_of_messages;
  }

  size_t get_snapshot_buffer_size() const
  {
    std::lock_guard<std::mutex> lock(messages_mutex_);
    return snapshot_buffer_.size();
  }

  std::vector<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>> get_snapshot_buffer()
  {
    std::lock_guard<std::mutex> lock(messages_mutex_);
    auto copy_of_snapshot_buffer = snapshot_buffer_;
    return copy_of_snapshot_buffer;
  }

  std::unordered_map<std::string, size_t> messages_per_topic()
  {
    std::lock_guard<std::mutex> lock(messages_mutex_);
    auto copy_of_messages_per_topic = messages_per_topic_;
    return copy_of_messages_per_topic;
  }

  size_t get_messages_per_topic(const std::string & topic_name) const
  {
    std::lock_guard<std::mutex> lock(messages_mutex_);
    if (messages_per_topic_.find(topic_name) != messages_per_topic_.end()) {
      return messages_per_topic_.at(topic_name);
    }
    return 0;
  }

  std::unordered_map<
    std::string,
    std::pair<rosbag2_storage::TopicMetadata, rosbag2_storage::MessageDefinition>
  > get_topics()
  {
    return topics_;
  }

  void set_max_messages_per_file(size_t max_messages_per_file)
  {
    max_messages_per_file_ = max_messages_per_file;
  }

  size_t max_messages_per_file() const
  {
    return max_messages_per_file_;
  }

  bool closed_was_called() const
  {
    return writer_close_called_;
  }

  rosbag2_storage::StorageOptions get_storage_options() const
  {
    return storage_options_;
  }

private:
  std::unordered_map<
    std::string,
    std::pair<rosbag2_storage::TopicMetadata, rosbag2_storage::MessageDefinition>
  > topics_;
  std::vector<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>> messages_;
  std::vector<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>> snapshot_buffer_;
  std::unordered_map<std::string, size_t> messages_per_topic_;
  size_t messages_per_file_ = 0;
  mutable std::mutex messages_mutex_;
  rosbag2_cpp::bag_events::EventCallbackManager callback_manager_;
  size_t file_number_ = 0;
  size_t max_messages_per_file_ = 0;
  bool writer_close_called_{false};
  rosbag2_storage::StorageOptions storage_options_;
};

#endif  // ROSBAG2_TRANSPORT__MOCK_SEQUENTIAL_WRITER_HPP_
