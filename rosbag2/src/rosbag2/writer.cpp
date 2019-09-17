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

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "rosbag2/info.hpp"
#include "rosbag2/storage_options.hpp"

namespace rosbag2
{

Writer::Writer(
  std::unique_ptr<rosbag2_storage::StorageFactoryInterface> storage_factory,
  std::shared_ptr<SerializationFormatConverterFactoryInterface> converter_factory,
  std::unique_ptr<rosbag2_storage::MetadataIo> metadata_io)
: storage_factory_(std::move(storage_factory)),
  converter_factory_(std::move(converter_factory)),
  metadata_io_(std::move(metadata_io)),
  converter_(nullptr),
  max_bagfile_size_(rosbag2_storage::storage_interfaces::MAX_BAGFILE_SIZE_NO_SPLIT)
{}

Writer::~Writer()
{
  if (!uri_.empty()) {
    aggregate_metadata_(storage_->get_metadata());
    metadata_io_->write_metadata(uri_, metadata_);
  }

  storage_.reset();  // Necessary to ensure that the storage is destroyed before the factory
  storage_factory_.reset();
}

void Writer::open(
  const StorageOptions & storage_options,
  const ConverterOptions & converter_options)
{
  if (converter_options.output_serialization_format !=
    converter_options.input_serialization_format)
  {
    converter_ = std::make_unique<Converter>(converter_options, converter_factory_);
  }

  storage_ = storage_factory_->open_read_write(storage_options.uri, storage_options.storage_id);
  if (!storage_) {
    throw std::runtime_error("No storage could be initialized. Abort");
  }
  uri_ = storage_options.uri;
  max_bagfile_size_ = storage_options.max_bagfile_size;
  initialize_metadata_();
}

void Writer::create_topic(const TopicMetadata & topic_with_type)
{
  if (!storage_) {
    throw std::runtime_error("Bag is not open. Call open() before writing.");
  }

  if (converter_) {
    converter_->add_topic(topic_with_type.name, topic_with_type.type);
  }

  storage_->create_topic(topic_with_type);
}

void Writer::remove_topic(const TopicMetadata & topic_with_type)
{
  if (!storage_) {
    throw std::runtime_error("Bag is not open. Call open() before removing.");
  }

  storage_->remove_topic(topic_with_type);
}

void Writer::write(std::shared_ptr<SerializedBagMessage> message)
{
  if (!storage_) {
    throw std::runtime_error("Bag is not open. Call open() before writing.");
  }

  storage_->write(converter_ ? converter_->convert(message) : message);

  if (should_split_database()) {
    aggregate_metadata_(storage_->get_metadata());
    storage_->split_database();
  }
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
