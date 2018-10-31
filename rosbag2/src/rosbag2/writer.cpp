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
  converter_(nullptr)
{}

Writer::~Writer()
{
  if (!uri_.empty()) {
    metadata_io_->write_metadata(uri_, storage_->get_metadata());
  }

  storage_.reset();  // Necessary to ensure that the storage is destroyed before the factory
  storage_factory_.reset();
}

void Writer::open(
  const StorageOptions & options,
  const std::string & input_serialization_format,
  const std::string & output_serialization_format)
{
  storage_ = storage_factory_->open_read_write(options.uri, options.storage_id);
  if (!storage_) {
    throw std::runtime_error("No storage could be initialized. Abort");
  }
  uri_ = options.uri;

  if (output_serialization_format != input_serialization_format) {
    converter_ = std::make_unique<Converter>(
      input_serialization_format,
      output_serialization_format,
      converter_factory_);
  }
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

void Writer::write(std::shared_ptr<SerializedBagMessage> message)
{
  if (!storage_) {
    throw std::runtime_error("Bag is not open. Call open() before writing.");
  }

  storage_->write(converter_ ? converter_->convert(message) : message);
}

}  // namespace rosbag2
