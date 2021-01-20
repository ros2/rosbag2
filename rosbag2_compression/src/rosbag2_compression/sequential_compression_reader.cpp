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
  std::unique_ptr<rosbag2_compression::CompressionFactory> compression_factory,
  std::unique_ptr<rosbag2_storage::StorageFactoryInterface> storage_factory,
  std::shared_ptr<rosbag2_cpp::SerializationFormatConverterFactoryInterface> converter_factory,
  std::unique_ptr<rosbag2_storage::MetadataIo> metadata_io)
: SequentialReader(std::move(storage_factory), converter_factory, std::move(metadata_io)),
  compression_factory_{std::move(compression_factory)}
{}

SequentialCompressionReader::~SequentialCompressionReader()
{}

void SequentialCompressionReader::preprocess_current_file()
{
  if (!decompressor_) {
    compression_mode_ = compression_mode_from_string(metadata_.compression_mode);
    if (compression_mode_ == rosbag2_compression::CompressionMode::NONE) {
      throw std::invalid_argument{
              "SequentialCompressionReader requires a CompressionMode that is not NONE!"};
    }
    decompressor_ = compression_factory_->create_decompressor(metadata_.compression_format);
    if (!decompressor_) {
      throw std::runtime_error{"Couldn't initialize decompressor."};
    }
  }
  if (compression_mode_ == CompressionMode::FILE) {
    ROSBAG2_COMPRESSION_LOG_INFO_STREAM("Decompressing " << get_current_file().c_str());
    *current_file_iterator_ = decompressor_->decompress_uri(get_current_file());
  }
}

void SequentialCompressionReader::open(
  const rosbag2_storage::StorageOptions & storage_options,
  const rosbag2_cpp::ConverterOptions & converter_options)
{
  if (!metadata_io_->metadata_file_exists(storage_options.uri)) {
    std::stringstream errmsg;
    errmsg << "Could not find metadata for bag: \"" << storage_options.uri <<
      "\". Legacy ROS 1 bag files aren't supported by rosbag2 decompression.";
    throw std::runtime_error{errmsg.str()};
  }
  SequentialReader::open(storage_options, converter_options);
}

std::shared_ptr<rosbag2_storage::SerializedBagMessage> SequentialCompressionReader::read_next()
{
  if (storage_ && decompressor_) {
    auto message = storage_->read_next();
    if (compression_mode_ == rosbag2_compression::CompressionMode::MESSAGE) {
      decompressor_->decompress_serialized_bag_message(message.get());
    }
    return converter_ ? converter_->convert(message) : message;
  }
  throw std::runtime_error{"Bag is not open. Call open() before reading."};
}

}  // namespace rosbag2_compression
