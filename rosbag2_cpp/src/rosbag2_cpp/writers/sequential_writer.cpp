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

#include "rosbag2_cpp/writers/sequential_writer.hpp"

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
#include "rosbag2_cpp/storage_options.hpp"

namespace rosbag2_cpp
{
namespace writers
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

// TODO(piraka9011) Initialize defaults in header file instead.
SequentialWriter::SequentialWriter(
  std::unique_ptr<rosbag2_storage::StorageFactoryInterface> storage_factory,
  std::shared_ptr<SerializationFormatConverterFactoryInterface> converter_factory,
  std::unique_ptr<rosbag2_storage::MetadataIo> metadata_io)
: storage_factory_(std::move(storage_factory)),
  converter_factory_(std::move(converter_factory)),
  storage_(nullptr),
  metadata_io_(std::move(metadata_io)),
  converter_(nullptr),
  compressor_{nullptr},
  max_bagfile_size_(rosbag2_storage::storage_interfaces::MAX_BAGFILE_SIZE_NO_SPLIT),
  topics_names_to_info_(),
  metadata_(),
  compression_mode_{CompressionMode::NONE},
  should_compress_last_file_{true} {}


SequentialWriter::~SequentialWriter()
{
  reset();
}

void SequentialWriter::check_bagfile_size(const StorageOptions & storage_options)
{
  if (storage_options.max_bagfile_size != 0 &&
    storage_options.max_bagfile_size < storage_->get_minimum_split_file_size())
  {
    throw std::invalid_argument(
            "Invalid bag splitting size given. Please provide a different value.");
  }
}

void SequentialWriter::init_compression(const CompressionOptions & compression_options)
{
  if (compression_options.compression_mode != rosbag2_cpp::CompressionMode::NONE) {
    if (compression_options.compression_format == "zstd") {
      compressor_ = std::make_unique<rosbag2_compression::ZstdCompressor>();
    } else {
      std::stringstream err;
      err << "Unsupported compression format " << compression_options.compression_format;
      throw std::runtime_error{err.str()};
    }
  }
}

void SequentialWriter::init_metadata(const CompressionOptions & compression_options)
{
  metadata_ = rosbag2_storage::BagMetadata{};
  metadata_.storage_identifier = storage_->get_storage_identifier();
  metadata_.starting_time = std::chrono::time_point<std::chrono::high_resolution_clock>(
    std::chrono::nanoseconds::max());
  metadata_.relative_file_paths = {storage_->get_relative_file_path()};
  if (compression_options.compression_mode != rosbag2_cpp::CompressionMode::NONE) {
    metadata_.compression_mode = rosbag2_cpp::compression_mode_to_string(
      compression_options.compression_mode);
    metadata_.compression_format = compression_options.compression_format;
  }
}

void SequentialWriter::open(
  const StorageOptions & storage_options,
  const ConverterOptions & converter_options,
  const CompressionOptions & compression_options)
{
  max_bagfile_size_ = storage_options.max_bagfile_size;
  base_folder_ = storage_options.uri;
  compression_mode_ = compression_options.compression_mode;

  if (converter_options.output_serialization_format !=
    converter_options.input_serialization_format)
  {
    converter_ = std::make_unique<Converter>(converter_options, converter_factory_);
  }

  const auto storage_uri = format_storage_uri(base_folder_, 0);
  storage_ = storage_factory_->open_read_write(storage_uri, storage_options.storage_id);
  if (!storage_) {
    throw std::runtime_error("No storage could be initialized. Abort");
  }

  check_bagfile_size(storage_options);
  init_compression(compression_options);
  init_metadata(compression_options);
}

void SequentialWriter::reset()
{
  if (!base_folder_.empty()) {
    if (max_bagfile_size_ == rosbag2_storage::storage_interfaces::MAX_BAGFILE_SIZE_NO_SPLIT) {
      compress_file_and_update_metadata();
    }
    finalize_metadata();
    metadata_io_->write_metadata(base_folder_, metadata_);
  }

  storage_.reset();  // Necessary to ensure that the storage is destroyed before the factory
  storage_factory_.reset();
}

void SequentialWriter::create_topic(const rosbag2_storage::TopicMetadata & topic_with_type)
{
  if (!storage_) {
    throw std::runtime_error("Bag is not open. Call open() before writing.");
  }

  if (converter_) {
    converter_->add_topic(topic_with_type.name, topic_with_type.type);
  }

  if (topics_names_to_info_.find(topic_with_type.name) ==
    topics_names_to_info_.end())
  {
    rosbag2_storage::TopicInformation info{};
    info.topic_metadata = topic_with_type;

    const auto insert_res = topics_names_to_info_.insert(
      std::make_pair(topic_with_type.name, info));

    if (!insert_res.second) {
      std::stringstream errmsg;
      errmsg << "Failed to insert topic \"" << topic_with_type.name << "\"!";

      throw std::runtime_error(errmsg.str());
    }

    storage_->create_topic(topic_with_type);
  }
}

void SequentialWriter::remove_topic(const rosbag2_storage::TopicMetadata & topic_with_type)
{
  if (!storage_) {
    throw std::runtime_error("Bag is not open. Call open() before removing.");
  }

  if (topics_names_to_info_.erase(topic_with_type.name) > 0) {
    storage_->remove_topic(topic_with_type);
  } else {
    std::stringstream errmsg;
    errmsg << "Failed to remove the non-existing topic \"" <<
      topic_with_type.name << "\"!";

    throw std::runtime_error(errmsg.str());
  }
}

bool SequentialWriter::compress_file_and_update_metadata()
{
  if (compressor_ && compression_mode_ == CompressionMode::FILE) {
    metadata_.relative_file_paths.back() =
      compressor_->compress_uri(metadata_.relative_file_paths.back());
    return true;
  }
  return false;
}

void SequentialWriter::split_bagfile()
{
  compress_file_and_update_metadata();

  const auto storage_uri = format_storage_uri(
    base_folder_,
    metadata_.relative_file_paths.size());
  storage_ = storage_factory_->open_read_write(storage_uri, metadata_.storage_identifier);
  if (!storage_) {
    std::stringstream errmsg;
    errmsg << "Failed to rollover bagfile to new file: \"" << storage_uri << "\"!";
    throw std::runtime_error(errmsg.str());
  }

  metadata_.relative_file_paths.push_back(storage_->get_relative_file_path());

  // Re-register all topics since we rolled-over to a new bagfile.
  for (const auto & topic : topics_names_to_info_) {
    storage_->create_topic(topic.second.topic_metadata);
  }
}

bool SequentialWriter::compress_message(
  std::shared_ptr<rosbag2_storage::SerializedBagMessage> message)
{
  if (compressor_ && compression_mode_ == CompressionMode::MESSAGE) {
    auto converted_message = converter_ ? converter_->convert(message) : message;
    compressor_->compress_serialized_bag_message(converted_message.get());
    return true;
  }
  return false;
}

void SequentialWriter::write(std::shared_ptr<rosbag2_storage::SerializedBagMessage> message)
{
  if (!storage_) {
    throw std::runtime_error("Bag is not open. Call open() before writing.");
  }

  // Update the message count for the Topic.
  ++topics_names_to_info_.at(message->topic_name).message_count;

  if (should_split_bagfile()) {
    split_bagfile();
  }

  const auto message_timestamp = std::chrono::time_point<std::chrono::high_resolution_clock>(
    std::chrono::nanoseconds(message->time_stamp));
  metadata_.starting_time = std::min(metadata_.starting_time, message_timestamp);

  const auto duration = message_timestamp - metadata_.starting_time;
  metadata_.duration = std::max(metadata_.duration, duration);

  compress_message(message);
  storage_->write(converter_ ? converter_->convert(message) : message);
}

bool SequentialWriter::should_split_bagfile() const
{
  if (max_bagfile_size_ == rosbag2_storage::storage_interfaces::MAX_BAGFILE_SIZE_NO_SPLIT) {
    return false;
  } else {
    return storage_->get_bagfile_size() > max_bagfile_size_;
  }
}

void SequentialWriter::finalize_metadata()
{
  metadata_.bag_size = 0;

  for (const auto & path : metadata_.relative_file_paths) {
    metadata_.bag_size += rcutils_get_file_size(path.c_str());
  }

  metadata_.topics_with_message_count.clear();
  metadata_.topics_with_message_count.reserve(topics_names_to_info_.size());
  metadata_.message_count = 0;

  for (const auto & topic : topics_names_to_info_) {
    metadata_.topics_with_message_count.push_back(topic.second);
    metadata_.message_count += topic.second.message_count;
  }
}

}  // namespace writers
}  // namespace rosbag2_cpp
