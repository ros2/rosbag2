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

#include "rosbag2/sequential_reader.hpp"

#include <cassert>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rcpputils/filesystem_helper.hpp"
#include "rosbag2/info.hpp"
#include "rosbag2/logging.hpp"
#include "rosbag2_storage/metadata_io.hpp"

namespace rosbag2
{

SequentialReader::SequentialReader(
  std::unique_ptr<rosbag2_storage::StorageFactoryInterface> storage_factory,
  std::shared_ptr<SerializationFormatConverterFactoryInterface> converter_factory)
: storage_factory_(std::move(storage_factory)), converter_factory_(std::move(converter_factory)),
  converter_(nullptr)
{}

SequentialReader::~SequentialReader()
{
  storage_.reset();  // Necessary to ensure that the storage is destroyed before the factory
}

void
SequentialReader::open(
  const StorageOptions & storage_options, const ConverterOptions & converter_options)
{
  rosbag2_storage::MetadataIo metadata_io;
  metadata_ = std::make_unique<rosbag2_storage::BagMetadata>(
            metadata_io.read_metadata(storage_options_.uri));
  storage_options_ = storage_options;
  storage_ = storage_factory_->open_read_only(storage_options.uri, storage_options.storage_id);
  if (!storage_) {
    throw std::runtime_error("No storage could be initialized. Abort");
  }
  auto topics = metadata_->topics_with_message_count;
  if (topics.empty()) {
    ROSBAG2_LOG_WARN("Bag file is empty.");
    return;
  }
  file_paths_ = metadata_->relative_file_paths;
  current_file_iterator_ = file_paths_.begin();

  // Currently a bag file can only be played if all topics have the same serialization format.
  auto storage_serialization_format = topics[0].topic_metadata.serialization_format;
  for (const auto & topic : topics) {
    if (topic.topic_metadata.serialization_format != storage_serialization_format) {
      throw std::runtime_error("Topics with different rwm serialization format have been found. "
              "All topics must have the same serialization format.");
    }
  }

  if (converter_options.output_serialization_format != storage_serialization_format) {
    converter_ = std::make_unique<Converter>(
      storage_serialization_format,
      converter_options.output_serialization_format,
      converter_factory_);
    auto topics = storage_->get_all_topics_and_types();
    for (const auto & topic_with_type : topics) {
      converter_->add_topic(topic_with_type.name, topic_with_type.type);
    }
  }
}

std::string SequentialReader::get_current_uri() const
{
  auto current_file = get_current_file();
  auto current_uri = rcpputils::fs::remove_extension(current_file);
  return current_uri.string();
}

std::string SequentialReader::get_current_file() const
{
  return *current_file_iterator_;
}

void SequentialReader::load_next_file()
{
  assert(current_file_iterator_ != file_paths_.end());
  current_file_iterator_++;
}

bool SequentialReader::has_next_file() const
{
  return current_file_iterator_ + 1 != file_paths_.end();
}

bool SequentialReader::has_next()
{
  if (storage_) {
    // If there's no new message, check if there's at least another file to read and update storage
    // to read from there. Otherwise, check if there's another message.
    if (!storage_->has_next()) {
      if (has_next_file()) {
        load_next_file();
        storage_ = storage_factory_->open_read_only(get_current_uri(), storage_options_.storage_id);
      }
    }
    return storage_->has_next();
  }
  throw std::runtime_error("Bag is not open. Call open() before reading.");
}

std::shared_ptr<SerializedBagMessage> SequentialReader::read_next()
{
  if (storage_) {
    auto message = storage_->read_next();
    return converter_ ? converter_->convert(message) : message;
  }
  throw std::runtime_error("Bag is not open. Call open() before reading.");
}

std::vector<TopicMetadata> SequentialReader::get_all_topics_and_types()
{
  if (storage_) {
    return storage_->get_all_topics_and_types();
  }
  throw std::runtime_error("Bag is not open. Call open() before reading.");
}

}  // namespace rosbag2
