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

#include "rcpputils/asserts.hpp"
#include "rcpputils/filesystem_helper.hpp"
#include "rosbag2_cpp/logging.hpp"

#include "vtr_storage/random_access_reader.hpp"

namespace vtr {
namespace storage {

// copied from sequential_reader.cpp
namespace details {
std::vector<std::string> resolve_relative_paths(
    const std::string &base_folder, std::vector<std::string> relative_files,
    const int version = 4) {
  auto base_path = rcpputils::fs::path(base_folder);
  if (version < 4) {
    // In older rosbags (version <=3) relative files are prefixed with the
    // rosbag folder name
    base_path = rcpputils::fs::path(base_folder).parent_path();
  }

  rcpputils::require_true(base_path.exists(),
                          "base folder does not exist: " + base_folder);
  rcpputils::require_true(base_path.is_directory(),
                          "base folder has to be a directory: " + base_folder);

  for (auto &file : relative_files) {
    auto path = rcpputils::fs::path(file);
    if (path.is_absolute()) {
      continue;
    }
    file = (base_path / path).string();
  }

  return relative_files;
}
}  // namespace details

RandomAccessReader::RandomAccessReader(
    const std::string &stream_name,
    std::unique_ptr<rosbag2_storage::StorageFactoryInterface> storage_factory,
    std::shared_ptr<rosbag2_cpp::SerializationFormatConverterFactoryInterface>
        converter_factory,
    std::unique_ptr<rosbag2_storage::MetadataIo> metadata_io)
    : SequentialReader(std::move(storage_factory), std::move(converter_factory),
                       std::move(metadata_io)),
      stream_name_(stream_name) {}

void RandomAccessReader::open(
    const rosbag2_cpp::StorageOptions &storage_options,
    const rosbag2_cpp::ConverterOptions &converter_options) {
  // If there is a metadata.yaml file present, load it.
  // If not, ----ASSUME A FILE STRUCTURE FOR SQLITE: ----
  // This could happen if the bag has just been created and no metadata has been
  // written yet.
  std::vector<std::string> relative_file_paths;
  int metadata_version;
  bool metadata_file_exists =
      metadata_io_->metadata_file_exists(storage_options.uri);
  if (metadata_file_exists) {
    metadata_ = metadata_io_->read_metadata(storage_options.uri);
    if (metadata_.relative_file_paths.empty()) {
      ROSBAG2_CPP_LOG_WARN("No file paths were found in metadata.");
      return;
    }
    relative_file_paths = metadata_.relative_file_paths;
    metadata_version = metadata_.version;
  } else {
    relative_file_paths.push_back(storage_options.uri +
                                  ("/" + stream_name_ + "_0.db3"));
    metadata_version = 4;
  }
  file_paths_ = details::resolve_relative_paths(
      storage_options.uri, relative_file_paths, metadata_version);
  current_file_iterator_ = file_paths_.begin();

  storage_ = storage_factory_->open_read_only(get_current_file(),
                                              storage_options.storage_id);
  if (!storage_) {
    throw std::runtime_error{"No storage could be initialized. Abort"};
  }

  if (!metadata_file_exists) {
    metadata_ = storage_->get_metadata();
  }

  auto topics = metadata_.topics_with_message_count;
  if (topics.empty()) {
    ROSBAG2_CPP_LOG_WARN("No topics were listed in metadata.");
    return;
  }
  fill_topics_metadata();

  // Currently a bag file can only be played if all topics have the same
  // serialization format.
  check_topics_serialization_formats(topics);
  check_converter_serialization_format(
      converter_options.output_serialization_format,
      topics[0].topic_metadata.serialization_format);
}

std::shared_ptr<rosbag2_storage::SerializedBagMessage>
RandomAccessReader::read_at_timestamp(rcutils_time_point_value_t timestamp) {
  if (storage_) {
    auto message = storage_->read_at_timestamp(timestamp);
    return converter_ ? converter_->convert(message) : message;
  }
  throw std::runtime_error("Bag is not open. Call open() before reading.");
}

std::shared_ptr<rosbag2_storage::SerializedBagMessage>
RandomAccessReader::read_at_index(uint32_t index) {
  if (storage_) {
    auto message = storage_->read_at_index(index);
    return converter_ ? converter_->convert(message) : message;
  }
  throw std::runtime_error("Bag is not open. Call open() before reading.");
}

std::shared_ptr<
    std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>>>
RandomAccessReader::read_at_timestamp_range(
    rcutils_time_point_value_t timestamp_begin,
    rcutils_time_point_value_t timestamp_end) {
  if (storage_) {
    auto message_vector =
        storage_->read_at_timestamp_range(timestamp_begin, timestamp_end);
    if (converter_) {
      for (auto &message : *message_vector) {
        message = converter_->convert(message);
      }
    }
    return message_vector;
  } else {
    throw std::runtime_error("Bag is not open. Call open() before reading.");
  }
}
std::shared_ptr<
    std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>>>
RandomAccessReader::read_at_index_range(uint32_t index_begin,
                                        uint32_t index_end) {
  if (storage_) {
    auto message_vector = storage_->read_at_index_range(index_begin, index_end);
    if (converter_) {
      for (auto &message : *message_vector) {
        message = converter_->convert(message);
      }
    }
    return message_vector;
  } else {
    throw std::runtime_error("Bag is not open. Call open() before reading.");
  }
}

}  // namespace storage
}  // namespace vtr