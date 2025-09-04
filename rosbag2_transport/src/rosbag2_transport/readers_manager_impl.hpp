// Copyright 2025 Apex.AI, Inc.
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

#ifndef ROSBAG2_TRANSPORT__READERS_MANAGER_IMPL_HPP_
#define ROSBAG2_TRANSPORT__READERS_MANAGER_IMPL_HPP_

#include <memory>
#include <utility>
#include <vector>
#include <limits>
#include <mutex>

#include "rcpputils/unique_lock.hpp"
#include "rcpputils/thread_safety_annotations.hpp"
#include "rcutils/time.h"
#include "rmw/rmw.h"

#include "rosbag2_cpp/bag_events.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"
#include "rosbag2_storage/storage_options.hpp"

namespace rosbag2_transport
{
class ReadersManagerImpl
{
public:
  using reader_storage_options_pair_t =
    std::pair<std::unique_ptr<rosbag2_cpp::Reader>, rosbag2_storage::StorageOptions>;

  explicit ReadersManagerImpl(std::vector<reader_storage_options_pair_t> && readers_with_options)
  : readers_with_options_(std::move(readers_with_options))
  {
    rcpputils::unique_lock lk(reader_mutex_);
    if (readers_with_options_.empty()) {
      throw std::invalid_argument("At least one reader with storage options must be provided.");
    }
    next_messages_cache_.reserve(readers_with_options_.size());
    earliest_timestamp_ = std::numeric_limits<rcutils_time_point_value_t>::max();
    latest_timestamp_ = std::numeric_limits<rcutils_time_point_value_t>::min();
    for (auto & [reader, options] : readers_with_options_) {
      reader->open(options, {"", rmw_get_serialization_format()});
      if (reader->has_next()) {
        next_messages_cache_.emplace_back(reader->read_next());
      }
      // Find the earliest starting time
      const auto metadata = reader->get_metadata();
      const auto metadata_starting_time = std::chrono::duration_cast<std::chrono::nanoseconds>(
        metadata.starting_time.time_since_epoch()).count();
      const auto metadata_bag_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(
        metadata.duration).count();
      if (metadata_starting_time < earliest_timestamp_) {
        earliest_timestamp_ = metadata_starting_time;
      }
      if (metadata_starting_time + metadata_bag_duration > latest_timestamp_) {
        latest_timestamp_ = metadata_starting_time + metadata_bag_duration;
      }
    }
  }

  virtual ~ReadersManagerImpl() = default;

  [[nodiscard]] std::vector<rosbag2_storage::StorageOptions> get_all_storage_options() const
  {
    rcpputils::unique_lock lk(reader_mutex_);
    std::vector<rosbag2_storage::StorageOptions> storage_options{};
    storage_options.reserve(readers_with_options_.size());
    for (const auto & [_, options] : readers_with_options_) {
      storage_options.push_back(options);
    }
    return storage_options;
  }

  [[nodiscard]] bool has_next() const
  {
    rcpputils::unique_lock lk(reader_mutex_);
    return !std::all_of(
      next_messages_cache_.cbegin(),
      next_messages_cache_.cend(),
      [](const auto & reader_next_msg) {return reader_next_msg == nullptr;});
  }

  [[nodiscard]] std::shared_ptr<rosbag2_storage::SerializedBagMessage>
  get_next_message_in_chronological_order()
  {
    // Note: To get next message in chronological order, we need to look at all readers' next
    // messages and pick the earliest one.
    // This is not optimal, but readers do not provide a way to peek at the next message without
    // advancing the reader, so we need to keep a cache of next messages for each reader.
    // This is still better than pushing all messages from all readers into a single priority
    // queue, because that would require pushing all messages from all readers into the queue,
    // while this way we only keep one message per reader in memory at any time.
    rcpputils::unique_lock lk(reader_mutex_);
    std::shared_ptr<rosbag2_storage::SerializedBagMessage> earliest_msg = nullptr;
    size_t earliest_msg_index = 0;
    for (size_t i = 0; i < next_messages_cache_.size(); i++) {
      const auto & message = next_messages_cache_[i];
      if (message != nullptr) {
        if (earliest_msg == nullptr || message->recv_timestamp < earliest_msg->recv_timestamp) {
          earliest_msg = message;
          earliest_msg_index = i;
        }
      }
    }
    if (earliest_msg != nullptr) {
      // Advance the reader that provided the message
      const auto & reader = readers_with_options_[earliest_msg_index].first;
      if (reader->has_next()) {
        next_messages_cache_[earliest_msg_index] = reader->read_next();
      } else {
        next_messages_cache_[earliest_msg_index] = nullptr;
      }
    }
    return earliest_msg;
  }

  void seek(const rcutils_time_point_value_t & timestamp)
  {
    rcpputils::unique_lock lk(reader_mutex_);
    for (auto & [reader, _] : readers_with_options_) {
      reader->seek(timestamp);
    }
    // Clear and refill the next_messages_cache_
    next_messages_cache_.clear();
    next_messages_cache_.resize(readers_with_options_.size(), nullptr);
    size_t i = 0;
    for (const auto & [reader, _] : readers_with_options_) {
      // Refill the next_messages_cache_
      if (reader->has_next()) {
        next_messages_cache_[i] = reader->read_next();
      }
      i++;
    }
  }

  [[nodiscard]] rcutils_time_point_value_t get_earliest_timestamp() const
  {
    return earliest_timestamp_;
  }

  [[nodiscard]] rcutils_time_point_value_t get_latest_timestamp() const
  {
    return latest_timestamp_;
  }

  void set_filter(const rosbag2_storage::StorageFilter & storage_filter)
  {
    rcpputils::unique_lock lk(reader_mutex_);
    for (auto & [reader, _] : readers_with_options_) {
      reader->set_filter(storage_filter);
    }
  }

  [[nodiscard]] std::vector<rosbag2_storage::TopicMetadata> get_all_topics_and_types() const
  {
    std::vector<rosbag2_storage::TopicMetadata> all_topics_and_types{};
    rcpputils::unique_lock lk(reader_mutex_);
    for (const auto & [reader, _] : readers_with_options_) {
      auto bag_topics_and_types = reader->get_all_topics_and_types();
      all_topics_and_types.insert(all_topics_and_types.end(),
                                  bag_topics_and_types.begin(),
                                  bag_topics_and_types.end());
    }
    return all_topics_and_types;
  }

  void add_event_callbacks(rosbag2_cpp::bag_events::ReaderEventCallbacks & callbacks)
  {
    rcpputils::unique_lock lk(reader_mutex_);
    for (auto & [reader, _] : readers_with_options_) {
      reader->add_event_callbacks(callbacks);
    }
  }

private:
  mutable std::mutex reader_mutex_;

  std::vector<reader_storage_options_pair_t>
  readers_with_options_ RCPPUTILS_TSA_GUARDED_BY(reader_mutex_);

  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>>
  next_messages_cache_ RCPPUTILS_TSA_GUARDED_BY(reader_mutex_);

  rcutils_time_point_value_t earliest_timestamp_{0};
  rcutils_time_point_value_t latest_timestamp_{0};
};

}  // namespace rosbag2_transport

#endif  // ROSBAG2_TRANSPORT__READERS_MANAGER_IMPL_HPP_
