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

#ifndef ROSBAG2__IMPL__SEQUENTIAL_READER_IMPL_HPP_
#define ROSBAG2__IMPL__SEQUENTIAL_READER_IMPL_HPP_

#include "rosbag2/sequential_reader.hpp"

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace rosbag2
{

class SequentialReader::ReaderImpl
{
public:
  ReaderImpl(
    std::unique_ptr<rosbag2_storage::StorageFactoryInterface> storage_factory,
    std::shared_ptr<SerializationFormatConverterFactoryInterface> converter_factory)
  : storage_factory_(std::move(storage_factory)),
    converter_factory_(std::move(converter_factory)),
    converter_(nullptr) {}

  ~ReaderImpl()
  {
    storage_.reset();
  }

  void open(
    const StorageOptions & storage_options, const ConverterOptions & converter_options)
  {
    storage_ = storage_factory_->open_read_only(storage_options.uri, storage_options.storage_id);
    if (!storage_) {
      throw std::runtime_error("No storage could be initialized. Abort");
    }
    auto topics = storage_->get_metadata().topics_with_message_count;
    if (topics.empty()) {
      return;
    }

    // Currently a bag file can only be played if all topics have the same serialization format.
    check_topics_serialization_formats(topics);
    check_converter_serialization_format(
      converter_options.output_serialization_format,
      topics[0].topic_metadata.serialization_format);
  }

  bool has_next()
  {
    if (storage_) {
      return storage_->has_next();
    }
    throw std::runtime_error("Bag is not open. Call open() before reading.");
  }

  std::shared_ptr<SerializedBagMessage> read_next()
  {
    if (storage_) {
      auto message = storage_->read_next();
      return converter_ ? converter_->convert(message) : message;
    }
    throw std::runtime_error("Bag is not open. Call open() before reading.");
  }

  std::vector<TopicMetadata> get_all_topics_and_types()
  {
    if (storage_) {
      return storage_->get_all_topics_and_types();
    }
    throw std::runtime_error("Bag is not open. Call open() before reading.");
  }

private:
  /**
   * Checks if all topics in the bagfile have the same RMW serialization format.
   * Currently a bag file can only be played if all topics have the same serialization format.
   *
   * \param topics Vector of TopicInformation with metadata.
   * \throws runtime_error if any topic has a different serialization format from the rest.
   */
  virtual void check_topics_serialization_formats(const std::vector<TopicInformation> & topics)
  {
    auto storage_serialization_format = topics[0].topic_metadata.serialization_format;
    for (const auto & topic : topics) {
      if (topic.topic_metadata.serialization_format != storage_serialization_format) {
        throw std::runtime_error(
                "Topics with different rwm serialization format have been found. "
                "All topics must have the same serialization format.");
      }
    }
  }

  /**
   * Checks if the serialization format of the converter factory is the same as that of the storage
   * factory.
   * If not, changes the serialization format of the converter factory to use the serialization
   * format of the storage factory.
   *
   * \param converter_serialization_format
   * \param storage_serialization_format
   */
  virtual void check_converter_serialization_format(
    const std::string & converter_serialization_format,
    const std::string & storage_serialization_format)
  {
    if (converter_serialization_format != storage_serialization_format) {
      converter_ = std::make_unique<Converter>(
        storage_serialization_format,
        converter_serialization_format,
        converter_factory_);
      auto topics = storage_->get_all_topics_and_types();
      for (const auto & topic_with_type : topics) {
        converter_->add_topic(topic_with_type.name, topic_with_type.type);
      }
    }
  }

  std::unique_ptr<rosbag2_storage::StorageFactoryInterface> storage_factory_;
  std::shared_ptr<SerializationFormatConverterFactoryInterface> converter_factory_;
  std::shared_ptr<rosbag2_storage::storage_interfaces::ReadOnlyInterface> storage_ {nullptr};
  std::unique_ptr<Converter> converter_ {nullptr};
};
}  // namespace rosbag2

#endif  // ROSBAG2__IMPL__SEQUENTIAL_READER_IMPL_HPP_
