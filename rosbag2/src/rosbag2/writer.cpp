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

#include "rosbag2/writer.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "rosbag2/info.hpp"
#include "rosbag2/storage_options.hpp"
#include "rosbag2/writer_interfaces/base_writer_interface.hpp"

#include "rosbag2_storage/filesystem_helper.hpp"

namespace rosbag2
{

Writer::Writer(std::unique_ptr<rosbag2::writer_interfaces::BaseWriterInterface> writer_impl)
: writer_impl_(std::move(writer_impl))
{}

Writer::~Writer()
{
  writer_impl_.reset();
}

void Writer::open(
  const StorageOptions & storage_options, const ConverterOptions & converter_options)
{
  writer_impl_->open(storage_options, converter_options);
}

void Writer::create_topic(const TopicMetadata & topic_with_type)
{
  writer_impl_->create_topic(topic_with_type);
}

void Writer::remove_topic(const TopicMetadata & topic_with_type)
{
  writer_impl_->remove_topic(topic_with_type);
}

void Writer::write(std::shared_ptr<SerializedBagMessage> message)
{
  writer_impl_->write(message);
}

bool Writer::should_split_database() const
{
  return (max_bagfile_size_ != rosbag2_storage::storage_interfaces::MAX_BAGFILE_SIZE_NO_SPLIT) &&
         (storage_->get_current_bagfile_size() > max_bagfile_size_);
}

void Writer::initialize_metadata_()
{
  metadata_.message_count = 0;
  metadata_.topics_with_message_count = {};
  metadata_.starting_time =
    std::chrono::time_point<std::chrono::high_resolution_clock>(
    std::chrono::nanoseconds(INT64_MAX));
  metadata_.duration = std::chrono::nanoseconds(0);
  metadata_.bag_size = 0;
}

void Writer::aggregate_metadata_(rosbag2_storage::BagMetadata metadata)
{
  metadata_.storage_identifier = metadata.storage_identifier;
  metadata_.relative_file_paths.swap(metadata.relative_file_paths);
  metadata_.message_count += metadata.message_count;

  if (metadata_.starting_time > metadata.starting_time &&
    metadata.starting_time !=
    std::chrono::time_point<std::chrono::high_resolution_clock>(std::chrono::nanoseconds(0)))
  {
    metadata_.starting_time = metadata.starting_time;
  }

  metadata_.duration += metadata.duration;
  metadata_.bag_size = metadata.bag_size;
  metadata_.topics_with_message_count.insert(
    metadata_.topics_with_message_count.end(),
    metadata.topics_with_message_count.begin(),
    metadata.topics_with_message_count.end());
}

}  // namespace rosbag2
