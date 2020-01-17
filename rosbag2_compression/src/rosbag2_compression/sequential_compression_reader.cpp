// Copyright 2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

#include "rosbag2_compression/sequential_compression_reader.hpp"

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rcpputils/filesystem_helper.hpp"
#include "rosbag2_compression/compression_options.hpp"
#include "rosbag2_compression/zstd_decompressor.hpp"
#include "logging.hpp"


namespace rosbag2_compression
{

SequentialCompressionReader::SequentialCompressionReader(
  std::unique_ptr<rosbag2_storage::StorageFactoryInterface> storage_factory,
  std::shared_ptr<rosbag2_cpp::SerializationFormatConverterFactoryInterface> converter_factory,
  std::unique_ptr<rosbag2_storage::MetadataIo> metadata_io)
: storage_factory_{std::move(storage_factory)},
  converter_factory_{std::move(converter_factory)},
  metadata_io_{std::move(metadata_io)}
{}

SequentialCompressionReader::~SequentialCompressionReader()
{
  reset();
}

void SequentialCompressionReader::reset()
{
  storage_.reset();
}

void SequentialCompressionReader::setup_decompression()
{
  compression_mode_ = rosbag2_compression::compression_mode_from_string(metadata_.compression_mode);
  if (compression_mode_ != rosbag2_compression::CompressionMode::NONE) {
    if (metadata_.compression_format == "zstd") {
      decompressor_ = std::make_unique<rosbag2_compression::ZstdDecompressor>();
      // Decompress the first file so that it is readable.
      ROSBAG2_COMPRESSION_LOG_DEBUG_STREAM("Decompressing " << get_current_file().c_str());
      set_current_file(decompressor_->decompress_uri(get_current_file()));
    } else {
      std::stringstream err;
      err << "Unsupported compression format " << metadata_.compression_format;
      throw std::invalid_argument{err.str()};
    }
  } else {
    throw std::invalid_argument{
            "SequentialCompressionReader requires a CompressionMode that is not NONE!"};
  }
}

void SequentialCompressionReader::open(
  const rosbag2_cpp::StorageOptions & storage_options,
  const rosbag2_cpp::ConverterOptions & converter_options)
{
  if (metadata_io_->metadata_file_exists(storage_options.uri)) {
    metadata_ = metadata_io_->read_metadata(storage_options.uri);
    if (metadata_.relative_file_paths.empty()) {
      ROSBAG2_COMPRESSION_LOG_WARN("No file paths were found in metadata.");
      return;
    }
    file_paths_ = metadata_.relative_file_paths;
    current_file_iterator_ = file_paths_.begin();
    setup_decompression();

    storage_ = storage_factory_->open_read_only(
      *current_file_iterator_, metadata_.storage_identifier);
    if (!storage_) {
      throw std::runtime_error{"No storage could be initialized. Abort"};
    }
  } else {
    throw std::runtime_error{"Compression is not supported for legacy bag files."};
  }
  const auto & topics = metadata_.topics_with_message_count;
  if (topics.empty()) {
    ROSBAG2_COMPRESSION_LOG_WARN("No topics were listed in metadata.");
    return;
  }

  // Currently a bag file can only be played if all topics have the same serialization format.
  check_topics_serialization_formats(topics);
  check_converter_serialization_format(
    converter_options.output_serialization_format,
    topics[0].topic_metadata.serialization_format);
}

bool SequentialCompressionReader::has_next()
{
  if (storage_) {
    // If there's no new message, check if there's at least another file to read and update storage
    // to read from there. Otherwise, check if there's another message.
    if (!storage_->has_next() && has_next_file()) {
      load_next_file();
      storage_ = storage_factory_->open_read_only(
        *current_file_iterator_, metadata_.storage_identifier);
    }
    return storage_->has_next();
  }
  throw std::runtime_error{"Bag is not open. Call open() before reading."};
}

std::shared_ptr<rosbag2_storage::SerializedBagMessage> SequentialCompressionReader::read_next()
{
  if (storage_) {
    assert(decompressor_);
    auto message = storage_->read_next();
    if (compression_mode_ == rosbag2_compression::CompressionMode::MESSAGE) {
      decompressor_->decompress_serialized_bag_message(message.get());
    }
    return converter_ ? converter_->convert(message) : message;
  }
  throw std::runtime_error{"Bag is not open. Call open() before reading."};
}

std::vector<rosbag2_storage::TopicMetadata> SequentialCompressionReader::get_all_topics_and_types()
{
  if (storage_) {
    return storage_->get_all_topics_and_types();
  }
  throw std::runtime_error{"Bag is not open. Call open() before reading."};
}

bool SequentialCompressionReader::has_next_file() const
{
  // Handle case where bagfile is not split
  if (current_file_iterator_ == file_paths_.end()) {
    return false;
  }

  return current_file_iterator_ + 1 != file_paths_.end();
}

void SequentialCompressionReader::load_next_file()
{
  assert(current_file_iterator_ != file_paths_.end());
  assert(decompressor_);
  ++current_file_iterator_;
  if (compression_mode_ == rosbag2_compression::CompressionMode::FILE) {
    ROSBAG2_COMPRESSION_LOG_DEBUG_STREAM("Decompressing " << get_current_file().c_str());
    *current_file_iterator_ = decompressor_->decompress_uri(get_current_file());
  }
}

std::string SequentialCompressionReader::get_current_uri() const
{
  const auto current_file = get_current_file();
  const auto current_uri = rcpputils::fs::remove_extension(current_file);
  return current_uri.string();
}

std::string SequentialCompressionReader::get_current_file() const
{
  return *current_file_iterator_;
}

void SequentialCompressionReader::set_current_file(const std::string & file)
{
  *current_file_iterator_ = file;
}

void SequentialCompressionReader::check_topics_serialization_formats(
  const std::vector<rosbag2_storage::TopicInformation> & topics)
{
  const auto & storage_serialization_format =
    topics[0].topic_metadata.serialization_format;

  for (const auto & topic : topics) {
    if (topic.topic_metadata.serialization_format != storage_serialization_format) {
      throw std::runtime_error{
              "Topics with different rwm serialization format have been found. "
              "All topics must have the same serialization format."};
    }
  }
}

void SequentialCompressionReader::check_converter_serialization_format(
  const std::string & converter_serialization_format,
  const std::string & storage_serialization_format)
{
  if (converter_serialization_format != storage_serialization_format) {
    converter_ = std::make_unique<rosbag2_cpp::Converter>(
      storage_serialization_format,
      converter_serialization_format,
      converter_factory_);
    const auto topics = storage_->get_all_topics_and_types();
    for (const auto & topic_with_type : topics) {
      converter_->add_topic(topic_with_type.name, topic_with_type.type);
    }
  }
}
}  // namespace rosbag2_compression
