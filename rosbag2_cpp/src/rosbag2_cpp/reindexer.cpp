// Copyright 2020 DCS Corporation, All Rights Reserved.
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
//
// DISTRIBUTION A. Approved for public release; distribution unlimited.
// OPSEC #4584.
//
// Delivered to the U.S. Government with Unlimited Rights, as defined in DFARS
// Part 252.227-7013 or 7014 (Feb 2014).
//
// This notice must appear in all copies of this file and its derivatives.

#include <algorithm>
#include <iostream>
#include <memory>
#include <regex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rcpputils/asserts.hpp"
#include "rcpputils/filesystem_helper.hpp"

#include "rcutils/filesystem.h"

#include "rosbag2_cpp/logging.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/reindexer.hpp"

#include "rosbag2_storage/storage_options.hpp"

#ifdef WIN32
// Import windows filesystem functionality
#include <windows.h>
#include <tchar.h>
#include <stdio.h>
#else
// We're on a UNIX system. Import their filesystem stuff instead
#include <dirent.h>
#endif


namespace rosbag2_cpp
{
namespace reindexers
{
namespace details
{
std::vector<rcpputils::fs::path> resolve_relative_paths(
  const rcpputils::fs::path & base_folder,
  std::vector<rcpputils::fs::path> relative_files,
  const int version = 4)
{
  auto base_path = rcpputils::fs::path(base_folder);  // Preserve folder
  if (version < 4) {
    // In older rosbags (version <=3) relative files are prefixed with the rosbag folder name
    base_path = rcpputils::fs::path(base_folder).parent_path();
  }

  rcpputils::require_true(
    base_path.exists(), "base folder does not exist: " + base_folder.string());
  rcpputils::require_true(
    base_path.is_directory(), "base folder has to be a directory: " + base_folder.string());

  for (auto & file : relative_files) {
    auto path = rcpputils::fs::path(file);
    if (path.is_absolute()) {
      continue;
    }
    file = (base_path / path).string();
  }

  return relative_files;
}
}  // namespace details

std::string strip_parent_path(const rcpputils::fs::path & relative_path)
{
  return relative_path.filename().string();
}

Reindexer::Reindexer(
  std::unique_ptr<rosbag2_storage::StorageFactoryInterface> storage_factory,
  std::shared_ptr<SerializationFormatConverterFactoryInterface> converter_factory,
  std::unique_ptr<rosbag2_storage::MetadataIo> metadata_io)
: storage_factory_(std::move(storage_factory)),
  converter_(nullptr),
  metadata_io_(std::move(metadata_io)),
  converter_factory_(std::move(converter_factory))
{
  // Make sure pointer isn't pointing to uninitialized memory
  // std::unique_ptr<rosbag2_cpp::readers::SequentialReader> bagfile_reader = nullptr;

  // // TODO: figure out how to make reader creation more generic
  // //       i.e: allow other reader types here, automagically
  // auto bagfile_reader_ = std::make_unique<rosbag2_cpp::Reader> (
  //   std::make_unique<rosbag2_cpp::readers::SequentialReader>(storage_factory_,
  //      converter_, metadata_io_)
  // );
}

Reindexer::~Reindexer()
{
  reset();
}

void Reindexer::reset()
{
  // if (storage_) {
  //   storage_.reset();
  // }
}

bool Reindexer::comp_rel_file(
  const rcpputils::fs::path & first_path, const rcpputils::fs::path & second_path)
{
  std::regex regex_rule(".*_(\\d+)\\.db3", std::regex_constants::ECMAScript);

  std::smatch first_match;
  std::smatch second_match;

  auto first_path_string = first_path.string();
  auto second_path_string = second_path.string();

  auto first_regex_good = std::regex_match(first_path_string, first_match, regex_rule);
  auto second_regex_good = std::regex_match(second_path_string, second_match, regex_rule);

  // Make sure the paths have regex matches
  if (!first_regex_good || !second_regex_good) {
    throw std::runtime_error("Malformed relative file name. Expected numerical identifier.");
  }

  // Convert database numbers to uint
  u_int32_t first_db_num = std::stoul(first_match.str(1), nullptr, 10);
  u_int32_t second_db_num = std::stoul(second_match.str(1), nullptr, 10);

  return first_db_num < second_db_num;
}

std::vector<rcpputils::fs::path> Reindexer::get_database_files(
  const rcpputils::fs::path & base_folder)
{
  // Look in the uri directory to see what database files are there
  std::vector<rcpputils::fs::path> output;

  auto allocator = rcutils_get_default_allocator();
  auto dir_iter = rcutils_dir_iter_start(base_folder.string().c_str(), allocator);

  // Make sure there are files in the directory
  if (dir_iter == nullptr) {
    throw std::runtime_error("Empty directory.");
  }

  // Get all file names in directory
  do {
    auto found_file = rcpputils::fs::path(dir_iter->entry_name);

    // We are ONLY interested in database files
    if (found_file.extension().string() == ".db3") {
      auto full_path = base_folder / found_file;
      output.emplace_back(full_path);
    }
  } while (rcutils_dir_iter_next(dir_iter));

  // Sort relative file path by database number
  std::sort(
    output.begin(), output.end(),
    [](rcpputils::fs::path a, rcpputils::fs::path b) {return comp_rel_file(a, b);});

  return output;
}

void Reindexer::fill_topics_metadata()
{
  // rcpputils::check_true(storage_ != nullptr, "Bag is not open. Call open() before reading.");
  topics_metadata_.clear();
  topics_metadata_.reserve(metadata_.topics_with_message_count.size());
  for (const auto & topic_information : metadata_.topics_with_message_count) {
    topics_metadata_.push_back(topic_information.topic_metadata);
  }
}

void Reindexer::init_metadata(
  const std::vector<rcpputils::fs::path> & files,
  const rosbag2_storage::StorageOptions & storage_options)
{
  metadata_ = rosbag2_storage::BagMetadata{};

  // This reindexer will only work on SQLite files, so this can't change
  metadata_.storage_identifier = storage_options.storage_id;
  metadata_.starting_time = std::chrono::time_point<std::chrono::high_resolution_clock>(
    std::chrono::nanoseconds::max());

  // Record the relative paths to the metadata
  for (const auto & path : files) {
    auto cleaned_path = strip_parent_path(path);
    metadata_.relative_file_paths.push_back(cleaned_path);
  }
}

void Reindexer::aggregate_metadata(
  const std::vector<rcpputils::fs::path> & files,
  const rosbag2_storage::StorageOptions & storage_options)
{
  // In order to most accurately reconstruct the metadata, we need to
  // visit each of the contained relative database files in the bag,
  // open them, slurp up the info, and stuff it into the master
  // metadata object.
  auto metadata_io_default = std::make_unique<rosbag2_storage::MetadataIo>();
  auto bag_reader = std::make_unique<rosbag2_cpp::readers::SequentialReader>(
    std::move(storage_factory_), converter_factory_, std::move(metadata_io_default));
  ROSBAG2_CPP_LOG_INFO_STREAM("Extracting metadata from database(s)");
  for (const auto & f_ : files) {
    ROSBAG2_CPP_LOG_INFO_STREAM("Extracting from file: " + f_.string());

    // Set up reader
    rosbag2_storage::StorageOptions temp_so = {
      f_.string(),  // uri
      storage_options.storage_id,  // storage_id
      storage_options.max_bagfile_size,  // max_bagfile_size
      storage_options.max_bagfile_duration,  // max_bagfile_duration
      storage_options.max_cache_size,  // max_cache_size
      storage_options.storage_config_uri  // storage_config_uri
    };

    // We aren't actually interested in reading messages, so use a blank converter option
    rosbag2_cpp::ConverterOptions blank_converter_options {};
    ROSBAG2_CPP_LOG_INFO_STREAM("Preparing to open...");
    bag_reader->open(temp_so, blank_converter_options);
    ROSBAG2_CPP_LOG_INFO_STREAM("Opened");
    // Get metadata from the bag file
    auto temp_metadata = bag_reader->get_metadata();

    if (temp_metadata.starting_time < metadata_.starting_time) {
      metadata_.starting_time = temp_metadata.starting_time;
    }
    // ROSBAG2_CPP_LOG_INFO_STREAM("Current duration: " + std::to_string(metadata_.duration.count()));
    // ROSBAG2_CPP_LOG_INFO_STREAM(
    //   "Incoming duration: " + std::to_string(temp_metadata.duration.count()));
    metadata_.duration += temp_metadata.duration;
    // ROSBAG2_CPP_LOG_INFO_STREAM("New duration: " + std::to_string(metadata_.duration.count()));
    metadata_.message_count += temp_metadata.message_count;

    // Add the topic metadata
    for (const auto & topic : temp_metadata.topics_with_message_count) {
      auto found_topic = std::find_if(
        metadata_.topics_with_message_count.begin(),
        metadata_.topics_with_message_count.end(),
        [&topic](const rosbag2_storage::TopicInformation & agg_topic)
        {return topic.topic_metadata.name == agg_topic.topic_metadata.name;});
      if (found_topic == metadata_.topics_with_message_count.end()) {
        // It's a new topic. Add it.
        metadata_.topics_with_message_count.emplace_back(topic);
      } else {
        // Merge in the new information
        found_topic->message_count += topic.message_count;
        if (topic.topic_metadata.offered_qos_profiles != "") {
          found_topic->topic_metadata.offered_qos_profiles =
            topic.topic_metadata.offered_qos_profiles;
        }
        if (topic.topic_metadata.serialization_format != "") {
          found_topic->topic_metadata.serialization_format =
            topic.topic_metadata.serialization_format;
        }
        if (topic.topic_metadata.type != "") {
          found_topic->topic_metadata.type = topic.topic_metadata.type;
        }
      }
    }

    ROSBAG2_CPP_LOG_INFO("Closing database");
    bag_reader->reset();   // Close the reader's storage
  }
  bag_reader.reset();   // Delete the reader
}

void Reindexer::reindex(const rosbag2_storage::StorageOptions & storage_options)
{
  ROSBAG2_CPP_LOG_INFO("Beginning Reindex Operation.");

  // Identify all database files
  base_folder_ = storage_options.uri;
  auto files = get_database_files(base_folder_);
  std::cout << "Finished getting database files\n";
  if (files.empty()) {
    ROSBAG2_CPP_LOG_ERROR("No database files found for reindexing. Abort");
    return;
  }

  // Create initial metadata
  init_metadata(files, storage_options);
  ROSBAG2_CPP_LOG_INFO_STREAM("Completed init_metadata");

  // Collect all metadata from database files
  aggregate_metadata(files, storage_options);
  ROSBAG2_CPP_LOG_INFO_STREAM("Completed aggregate_metadata");

  // Perform final touch-up
  finalize_metadata();
  ROSBAG2_CPP_LOG_INFO_STREAM("Completed finalize_metadata");

  ROSBAG2_CPP_LOG_INFO_STREAM("Base folder located at: " + base_folder_.string());
  metadata_io_->write_metadata(base_folder_.string(), metadata_);
  ROSBAG2_CPP_LOG_INFO("Reindexing operation completed.");
}

void Reindexer::finalize_metadata()
{
  metadata_.bag_size = 0;

  for (const auto & path : metadata_.relative_file_paths) {
    const auto bag_path = rcpputils::fs::path{path};

    if (bag_path.exists()) {
      metadata_.bag_size += bag_path.file_size();
    }
  }
}
}  // namespace reindexers
}  // namespace rosbag2_cpp
