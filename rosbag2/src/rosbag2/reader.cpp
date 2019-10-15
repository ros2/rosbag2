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

#include "rosbag2/reader.hpp"

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rosbag2/info.hpp"

namespace rosbag2
{

Reader::Reader(
  std::unique_ptr<rosbag2_storage::StorageFactoryInterface> storage_factory,
  std::shared_ptr<SerializationFormatConverterFactoryInterface> converter_factory)
: storage_factory_(std::move(storage_factory)), converter_factory_(std::move(converter_factory)),
  converter_(nullptr)
{}

Reader::~Reader()
{
  storage_.reset();  // Necessary to ensure that the storage is destroyed before the factory
}

void Reader::reset()
{
  storage_.reset();
}

void Reader::open(
  const StorageOptions & storage_options, const ConverterOptions & converter_options)
{
  storage_ = storage_factory_->open_read_only(storage_options.uri, storage_options.storage_id);
  if (!storage_) {
    throw std::runtime_error("No storage could be initialized. Abort");
  }
  auto topics = get_metadata().topics_with_message_count;
  if (topics.empty()) {
    return;
  }

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

  ROSBAG2_LOG_INFO_STREAM("Reader opened.");
}

bool Reader::has_next()
{
  if (storage_) {
    return storage_->has_next();
  }
  throw std::runtime_error("(has_next) Bag is not open. Call open() before reading.");
}

std::shared_ptr<SerializedBagMessage> Reader::read_next()
{
  if (storage_) {
    auto message = storage_->read_next();
    return converter_ ? converter_->convert(message) : message;
  }
  throw std::runtime_error("(read_next) Bag is not open. Call open() before reading.");
}

std::vector<TopicMetadata> Reader::get_all_topics_and_types()
{
  if (storage_) {
    return storage_->get_all_topics_and_types();
  }
  throw std::runtime_error("(get_all_topic_and_types) Bag is not open. "
          "Call open() before reading.");
}

bool Reader::has_storage()
{
  return storage_ != nullptr;
}

bool Reader::open_read_only(const std::string & file, const std::string & storage_id)
{
  ROSBAG2_LOG_INFO_STREAM("Opening file: " << file);
  storage_ = storage_factory_->open_read_only(file, storage_id);
  return has_storage();
}

rosbag2_storage::BagMetadata Reader::get_metadata()
{
  return storage_->get_metadata();
}

}  // namespace rosbag2
