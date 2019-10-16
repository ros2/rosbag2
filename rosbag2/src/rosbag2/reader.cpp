// Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rosbag2/info.hpp"
#include "rosbag2/logging.hpp"
#include "rosbag2/reader.hpp"

namespace rosbag2
{

Reader::Reader(
  std::unique_ptr<rosbag2_storage::StorageFactoryInterface> storage_factory,
  std::shared_ptr<SerializationFormatConverterFactoryInterface> converter_factory)
: storage_factory_(std::move(storage_factory)),
  converter_factory_(std::move(converter_factory))
{}

Reader::~Reader()
{
  reset();  // Necessary to ensure that the storage is destroyed before the factory
}

void Reader::reset()
{
  storage_.reset();
}

void Reader::check_topics_serialization_formats(const std::vector<TopicInformation> & topics)
{
  const auto storage_serialization_format = topics.at(0).topic_metadata.serialization_format;
  for (const auto & topic : topics) {
    if (topic.topic_metadata.serialization_format != storage_serialization_format) {
      std::stringstream error_message;
      error_message << "Topic " << topic.topic_metadata.name << "uses RMW serialization format " <<
        topic.topic_metadata.serialization_format << ", but topic " <<
        topics.at(0).topic_metadata.name << "uses RMW serialization format " <<
        storage_serialization_format <<
        ".\nOpening storage formats with mixed RMW serialization formats is currently unsupported.";
      throw std::runtime_error(error_message.str().c_str());
    }
  }
}

void Reader::check_converter_serialization_format(
  const std::string & converter_serialization_format,
  const std::string & storage_serialization_format)
{
  if (converter_serialization_format != storage_serialization_format) {
    ROSBAG2_LOG_WARN_STREAM("Storage serialization format is " <<
      storage_serialization_format <<
      " but converter has serialization format " << converter_serialization_format << ". "
      "Using converter to serialize messages.\nReplay performance may be degraded!");
    converter_ = std::make_unique<Converter>(
      storage_serialization_format,
      converter_serialization_format,
      converter_factory_);
    const auto topics = storage_->get_all_topics_and_types();
    for (const auto & topic_with_type : topics) {
      converter_->add_topic(topic_with_type.name, topic_with_type.type);
    }
  }
}

void Reader::open(
  const StorageOptions & storage_options, const ConverterOptions & converter_options)
{
  open_read_only(storage_options.uri, storage_options.storage_id);

  const auto metadata = get_metadata();
  const auto topics = metadata.topics_with_message_count;
  if (topics.empty()) {
    return;
  }

  check_topics_serialization_formats(topics);

  const auto storage_serialization_format = topics.at(0).topic_metadata.serialization_format;
  const auto converter_serialization_format = converter_options.output_serialization_format;
  check_converter_serialization_format(converter_serialization_format,
    storage_serialization_format);

  ROSBAG2_LOG_DEBUG("Bag opened.");
}

bool Reader::has_next() const
{
  if (has_storage()) {
    return storage_->has_next();
  }
  throw std::runtime_error("Bag is not open. Call open() before reading.");
}

std::shared_ptr<SerializedBagMessage> Reader::read_next()
{
  if (has_storage()) {
    auto message = storage_->read_next();
    return converter_ ? converter_->convert(message) : message;
  }
  throw std::runtime_error("Bag is not open. Call has_next() before reading.");
}

std::vector<TopicMetadata> Reader::get_all_topics_and_types() const
{
  if (has_storage()) {
    return storage_->get_all_topics_and_types();
  }
  throw std::runtime_error("Bag is not open. Call open() before reading.");
}

bool Reader::has_storage() const
{
  return storage_ != nullptr;
}

bool Reader::open_read_only(const std::string & file, const std::string & storage_id)
{
  ROSBAG2_LOG_DEBUG_STREAM("Opening file: " << file);
  storage_ = storage_factory_->open_read_only(file, storage_id);
  if (!has_storage()) {
    std::stringstream error_message;
    error_message << "Failed to open storage file: " << file;
    throw std::runtime_error(error_message.str().c_str());
  }
  return has_storage();
}

rosbag2_storage::BagMetadata Reader::get_metadata() const
{
  return storage_->get_metadata();
}

}  // namespace rosbag2
