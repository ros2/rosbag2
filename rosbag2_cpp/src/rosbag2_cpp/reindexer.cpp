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
#include <map>
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
Reindexer::Reindexer(
  std::unique_ptr<rosbag2_storage::StorageFactoryInterface> storage_factory,
  std::unique_ptr<rosbag2_storage::MetadataIo> metadata_io)
: storage_factory_(std::move(storage_factory)),
  metadata_io_(std::move(metadata_io))
  {
    regex_bag_extension_pattern_ = R"(._(\d+)\.([a-zA-Z0-9]))";
  }

Reindexer::~Reindexer() {}

// Determines which path should be placed first in a vector ordered by file number.
/**
 * Used to re-order discovered bag files, since the filesystem discovery functions
 * don't guarantee a preserved order
 */
bool Reindexer::compare_relative_file(
  const rcpputils::fs::path & first_path, const rcpputils::fs::path & second_path)
{
  std::regex regex_rule(regex_bag_extension_pattern_, std::regex_constants::ECMAScript);

  std::smatch first_match;
  std::smatch second_match;

  auto first_path_string = first_path.string();
  auto second_path_string = second_path.string();

  auto first_regex_good = std::regex_match(first_path_string, first_match, regex_rule);
  auto second_regex_good = std::regex_match(second_path_string, second_match, regex_rule);

  // Make sure the paths have regex matches
  if (!first_regex_good || !second_regex_good) {
    std::stringstream ss;
    ss << "Path " << first_path.string() <<
      "didn't meet expected naming convention: " << regex_bag_extension_pattern_;
    std::string error_text = ss.str();
    throw std::runtime_error(error_text.c_str());
  }

  u_int32_t first_db_num = std::stoul(first_match.str(1), nullptr, 10);
  u_int32_t second_db_num = std::stoul(second_match.str(1), nullptr, 10);

  return first_db_num < second_db_num;
}

// Retrieves bag storage files from the bag directory.
/**
 * @param output: A vector to save the discovered files inside of
 *    The files will be `emplace_back`-ed on the passed vector
 */
void Reindexer::get_bag_files(
  const rcpputils::fs::path & base_folder,
  const std::string & expected_extension,
  std::vector<rcpputils::fs::path> & output)
{
  // std::regex regex_rule(R"([a-zA-Z0-9])", std::regex_constants::ECMAScript);
  auto allocator = rcutils_get_default_allocator();
  auto dir_iter = rcutils_dir_iter_start(base_folder.string().c_str(), allocator);

  // Make sure there are files in the directory
  if (dir_iter == nullptr) {
    throw std::runtime_error("Empty directory.");
  }

  // Get all file names in directory
  do {
    auto found_file = rcpputils::fs::path(dir_iter->entry_name);

    // if (std::regex_match(found_file.extension().string(), regex_rule)) {
    if (found_file.extension().string() == expected_extension)
    {
      auto full_path = base_folder / found_file;
      output.emplace_back(full_path);
    }
  } while (rcutils_dir_iter_next(dir_iter));

  // Sort relative file path by database number
  std::sort(
    output.begin(), output.end(),
    [this](rcpputils::fs::path a, rcpputils::fs::path b) {return compare_relative_file(a, b);});
}

// Prepares a fresh BagMetadata object for reindexing.
/**
 * Creates a new `BagMetadata` object with the `storage_identifier` and `relative_file_paths` filled in
 * 
 * Also fills in `starting_time` with a dummy default value. Important for later functions
 */
void Reindexer::init_metadata(
  const std::vector<rcpputils::fs::path> & files,
  const rosbag2_storage::StorageOptions & storage_options)
{
  metadata_ = rosbag2_storage::BagMetadata{};

  metadata_.storage_identifier = storage_options.storage_id;
  metadata_.starting_time = std::chrono::time_point<std::chrono::high_resolution_clock>(
    std::chrono::nanoseconds::max());

  // Record the relative paths to the metadata
  for (const auto & path : files) {
    auto cleaned_path = path.filename().string();
    metadata_.relative_file_paths.push_back(cleaned_path);
  }
}

// Iterates through the bag files to collect various metadata parameters
/**
 * Collects the topic metadata, `starting_time`, and `duration` portions of the `BagMetadata`
 * being constructed
 * 
 * @param: files The list of bag files to reindex
 * 
 * @param: storage_options Used to construct the `Reader` needed to parse the bag files
 */
void Reindexer::aggregate_metadata(
  const std::vector<rcpputils::fs::path> & files,
  const std::unique_ptr<rosbag2_cpp::readers::SequentialReader> & bag_reader,
  const rosbag2_storage::StorageOptions & storage_options)
{
  std::map<std::string, rosbag2_storage::TopicInformation> temp_topic_info;

  // In order to most accurately reconstruct the metadata, we need to
  // visit each of the contained relative database files in the bag,
  // open them, slurp up the info, and stuff it into the master
  // metadata object.
  ROSBAG2_CPP_LOG_DEBUG_STREAM("Extracting metadata from database(s)");
  for (const auto & f_ : files) {
    ROSBAG2_CPP_LOG_DEBUG_STREAM("Extracting from file: " + f_.string());

    metadata_.bag_size += f_.file_size();

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
    bag_reader->open(temp_so, blank_converter_options);
    auto temp_metadata = bag_reader->get_metadata();

    if (temp_metadata.starting_time < metadata_.starting_time) {
      metadata_.starting_time = temp_metadata.starting_time;
    }
    metadata_.duration += temp_metadata.duration;
    ROSBAG2_CPP_LOG_DEBUG_STREAM("New duration: " + std::to_string(metadata_.duration.count()));
    metadata_.message_count += temp_metadata.message_count;

    // Add the topic metadata
    for (const auto & topic : temp_metadata.topics_with_message_count) {
      auto found_topic = temp_topic_info.find(topic.topic_metadata.name);
      if (found_topic == temp_topic_info.end()) {
        // It's a new topic. Add it.
        temp_topic_info[topic.topic_metadata.name] = topic;
      } else {
        ROSBAG2_CPP_LOG_DEBUG_STREAM("Found topic!");
        // Merge in the new information
        found_topic->second.message_count += topic.message_count;
        if (topic.topic_metadata.offered_qos_profiles != "") {
          found_topic->second.topic_metadata.offered_qos_profiles =
            topic.topic_metadata.offered_qos_profiles;
        }
        if (topic.topic_metadata.serialization_format != "") {
          found_topic->second.topic_metadata.serialization_format =
            topic.topic_metadata.serialization_format;
        }
        if (topic.topic_metadata.type != "") {
          found_topic->second.topic_metadata.type = topic.topic_metadata.type;
        }
      }
    }

    bag_reader->reset();
  }

  // Convert the topic map into topic metadata
  for (auto & topic : temp_topic_info) {
    metadata_.topics_with_message_count.emplace_back(topic.second);
  }
}

// Reconstructs a bag's `metadata.yaml` file from the enclosed bag files.
/**
 * The reindexer opens the files within the bag directory and uses the metadata of the files to
 * reconstruct the metadata file. Currently does not support compressed bags.
 * 
 * @param: storage_options The best-guess original storage options for the bag
 */
void Reindexer::reindex(const rosbag2_storage::StorageOptions & storage_options)
{
  ROSBAG2_CPP_LOG_INFO_STREAM("Beginning reindexing bag in directory: " << base_folder_);

  auto metadata_io_default = std::make_unique<rosbag2_storage::MetadataIo>();
  auto bag_reader = std::make_unique<rosbag2_cpp::readers::SequentialReader>(
    std::move(storage_factory_), converter_factory_, std::move(metadata_io_default));
  
  auto expected_extension = bag_reader->storage_->get_storage_extension();  // Does not currently work

  // Identify all bag files
  std::vector<rcpputils::fs::path> files;
  base_folder_ = storage_options.uri;
  get_bag_files(base_folder_, expected_extension, files);
  ROSBAG2_CPP_LOG_DEBUG("Finished getting database files");
  if (files.empty()) {
    throw std::runtime_error("No database files found for reindexing. Abort");
  }

  init_metadata(files, storage_options);
  ROSBAG2_CPP_LOG_DEBUG_STREAM("Completed init_metadata");

  // Collect all metadata from database files
  aggregate_metadata(files, bag_reader, storage_options);
  ROSBAG2_CPP_LOG_DEBUG_STREAM("Completed aggregate_metadata");

  metadata_io_->write_metadata(base_folder_.string(), metadata_);
  ROSBAG2_CPP_LOG_INFO("Reindexing complete.");
}
}  // namespace rosbag2_cpp
