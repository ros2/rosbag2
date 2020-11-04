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

#include "rosbag2_compression/sequential_compression_writer.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "rcpputils/filesystem_helper.hpp"

#include "rcutils/filesystem.h"

#include "rosbag2_compression/zstd_compressor.hpp"

#include "rosbag2_cpp/info.hpp"

#include "rosbag2_storage/storage_options.hpp"
#include "rosbag2_storage/storage_interfaces/read_write_interface.hpp"

#include "logging.hpp"

namespace rosbag2_compression
{

namespace
{
std::string format_storage_uri(const std::string & base_folder, uint64_t storage_count)
{
  // Right now `base_folder_` is always just the folder name for where to install the bagfile.
  // The name of the folder needs to be queried in case
  // SequentialWriter is opened with a relative path.
  std::stringstream storage_file_name;
  storage_file_name << rcpputils::fs::path(base_folder).filename().string() << "_" << storage_count;

  return (rcpputils::fs::path(base_folder) / storage_file_name.str()).string();
}
}  // namespace

SequentialCompressionWriter::SequentialCompressionWriter(
  const rosbag2_compression::CompressionOptions & compression_options)
: SequentialWriter(),
  compression_factory_{std::make_unique<rosbag2_compression::CompressionFactory>()},
  compression_options_{compression_options}
{}

SequentialCompressionWriter::SequentialCompressionWriter(
  const rosbag2_compression::CompressionOptions & compression_options,
  std::unique_ptr<rosbag2_compression::CompressionFactory> compression_factory,
  std::unique_ptr<rosbag2_storage::StorageFactoryInterface> storage_factory,
  std::shared_ptr<rosbag2_cpp::SerializationFormatConverterFactoryInterface> converter_factory,
  std::unique_ptr<rosbag2_storage::MetadataIo> metadata_io)
: SequentialWriter(std::move(storage_factory), converter_factory, std::move(metadata_io)),
  compression_factory_{std::move(compression_factory)},
  compression_options_{compression_options}
{}

SequentialCompressionWriter::~SequentialCompressionWriter()
{
  reset();
}

void SequentialCompressionWriter::init_metadata()
{
  metadata_ = rosbag2_storage::BagMetadata{};
  metadata_.storage_identifier = storage_->get_storage_identifier();
  metadata_.starting_time = std::chrono::time_point<std::chrono::high_resolution_clock>{
    std::chrono::nanoseconds::max()};
  metadata_.relative_file_paths = {storage_->get_relative_file_path()};
  metadata_.compression_format = compression_options_.compression_format;
  metadata_.compression_mode =
    rosbag2_compression::compression_mode_to_string(compression_options_.compression_mode);
}

void SequentialCompressionWriter::setup_compression()
{
  if (compression_options_.compression_mode == rosbag2_compression::CompressionMode::NONE) {
    throw std::invalid_argument{
            "SequentialCompressionWriter requires a CompressionMode that is not NONE!"};
  }
  compressor_ = compression_factory_->create_compressor(compression_options_.compression_format);
}

void SequentialCompressionWriter::open(
  const rosbag2_storage::StorageOptions & storage_options,
  const rosbag2_cpp::ConverterOptions & converter_options)
{
  SequentialWriter::open(storage_options, converter_options);
  setup_compression();
}


void SequentialCompressionWriter::reset()
{
  if (!base_folder_.empty() && compressor_) {
    // Reset may be called before initializing the compressor (ex. bad options).
    // We compress the last file only if it hasn't been compressed earlier (ex. in split_bagfile()).
    if (compression_options_.compression_mode == rosbag2_compression::CompressionMode::FILE &&
      should_compress_last_file_)
    {
      try {
        storage_.reset();  // Storage must be closed before it can be compressed.
        compress_last_file();
      } catch (const std::runtime_error & e) {
        ROSBAG2_COMPRESSION_LOG_WARN_STREAM("Could not compress the last bag file.\n" << e.what());
      }
    }
    finalize_metadata();
    metadata_io_->write_metadata(base_folder_, metadata_);
  }

  buffer_layer_.reset();
  storage_.reset();  // Necessary to ensure that the storage is destroyed before the factory
  storage_factory_.reset();
}

void SequentialCompressionWriter::compress_last_file()
{
  if (!compressor_) {
    throw std::runtime_error{"compress_last_file: Compressor was not opened!"};
  }

  const auto to_compress = rcpputils::fs::path{metadata_.relative_file_paths.back()};

  if (to_compress.exists() && to_compress.file_size() > 0u) {
    const auto compressed_uri = compressor_->compress_uri(to_compress.string());

    metadata_.relative_file_paths.back() = compressed_uri;

    if (!rcpputils::fs::remove(to_compress)) {
      ROSBAG2_COMPRESSION_LOG_ERROR_STREAM(
        "Failed to remove uncompressed bag: \"" << to_compress.string() << "\"");
    }
  } else {
    ROSBAG2_COMPRESSION_LOG_DEBUG_STREAM(
      "Removing last file: \"" << to_compress.string() <<
        "\" because it either is empty or does not exist.");

    metadata_.relative_file_paths.pop_back();
  }
}

void SequentialCompressionWriter::split_bagfile()
{
  // Flush buffer layer
  buffer_layer_->close();

  storage_options_.uri = format_storage_uri(
    base_folder_,
    metadata_.relative_file_paths.size());

  storage_ = storage_factory_->open_read_write(storage_options_);

  // Update storage in buffer layer and restart consumer thread
  buffer_layer_->set_storage(storage_);
  buffer_layer_->start_consumer();

  if (compression_options_.compression_mode == rosbag2_compression::CompressionMode::FILE) {
    compress_last_file();
  }

  if (!storage_) {
    // Add a check to make sure reset() does not compress the file again if we couldn't load the
    // storage plugin.
    should_compress_last_file_ = false;

    std::stringstream errmsg;
    errmsg << "Failed to rollover bagfile to new file: \"" << storage_options_.uri << "\"!";
    throw std::runtime_error{errmsg.str()};
  }

  metadata_.relative_file_paths.push_back(storage_->get_relative_file_path());

  // Re-register all topics since we rolled-over to a new bagfile.
  for (const auto & topic : topics_names_to_info_) {
    storage_->create_topic(topic.second.topic_metadata);
  }
}

void SequentialCompressionWriter::compress_message(
  std::shared_ptr<rosbag2_storage::SerializedBagMessage> message)
{
  if (!compressor_) {
    throw std::runtime_error{"Cannot compress message; Writer is not open!"};
  }

  compressor_->compress_serialized_bag_message(message.get());
}

std::shared_ptr<rosbag2_storage::SerializedBagMessage>
SequentialCompressionWriter::get_writeable_message(
  std::shared_ptr<rosbag2_storage::SerializedBagMessage> message)
{
  auto writeable_msg = SequentialWriter::get_writeable_message(message);
  if (compression_options_.compression_mode == rosbag2_compression::CompressionMode::MESSAGE) {
    compress_message(writeable_msg);
  }
  return writeable_msg;
}

}  // namespace rosbag2_compression
