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

#include <boost/filesystem.hpp>
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

#include "rosbag2_cpp/logging.hpp"
#include "rosbag2_cpp/reindexers/sequential_reindexer.hpp"


namespace rosbag2_cpp
{
namespace reindexers
{
namespace details
{
std::vector<std::string> resolve_relative_paths(
  const std::string & base_folder, std::vector<std::string> relative_files, const int version = 4)
{
  auto base_path = rcpputils::fs::path(base_folder);
  if (version < 4) {
    // In older rosbags (version <=3) relative files are prefixed with the rosbag folder name
    base_path = rcpputils::fs::path(base_folder).parent_path();
  }

  rcpputils::require_true(
    base_path.exists(), "base folder does not exist: " + base_folder);
  rcpputils::require_true(
    base_path.is_directory(), "base folder has to be a directory: " + base_folder);

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

std::string strip_parent_path(const std::string & relative_path)
{
  return rcpputils::fs::path(relative_path).filename().string();
}

SequentialReindexer::SequentialReindexer(
  std::unique_ptr<rosbag2_storage::StorageFactoryInterface> storage_factory,
  std::shared_ptr<SerializationFormatConverterFactoryInterface> converter_factory,
  std::unique_ptr<rosbag2_storage::MetadataIo> metadata_io)
: storage_factory_(std::move(storage_factory)),
  converter_(nullptr),
  metadata_io_(std::move(metadata_io)),
  converter_factory_(std::move(converter_factory))
{}

SequentialReindexer::~SequentialReindexer()
{
  reset();
}

void SequentialReindexer::reset()
{
  if (storage_) {
    storage_.reset();
  }
}


bool SequentialReindexer::comp_rel_file(
  const std::string & first_path, const std::string & second_path)
{
  std::regex regex_rule(".*_(\\d+)\\.db3", std::regex_constants::ECMAScript);

  std::smatch first_match;
  std::smatch second_match;

  auto first_regex_good = std::regex_match(first_path, first_match, regex_rule);
  auto second_regex_good = std::regex_match(second_path, second_match, regex_rule);

  // Make sure the paths have regex matches
  if (!first_regex_good || !second_regex_good) {
    throw std::runtime_error("Malformed relative file name. Expected numerical identifier.");
  }

  // Convert database numbers to uint
  u_int32_t first_db_num = std::stoul(first_match.str(1), nullptr, 10);
  u_int32_t second_db_num = std::stoul(second_match.str(1), nullptr, 10);

  return first_db_num < second_db_num;
}

std::vector<std::string> SequentialReindexer::get_database_files(const std::string & base_folder)
{
  // Look in the uri directory to see what database files are there
  std::vector<std::string> output;
  for (auto & p_ : boost::filesystem::directory_iterator(base_folder)) {
    // We are ONLY interested in database files
    if (p_.path().extension() != ".db3") {
      continue;
    }

    output.emplace_back(p_.path().c_str());
    std::cout << "Found path: " << p_.path().c_str() << "\n";
  }

  // Sort relative file path by database number
  std::sort(
    output.begin(), output.end(),
    [](std::string a, std::string b) {return comp_rel_file(a, b);});

  return output;
}

void SequentialReindexer::open(
  const std::string & database_file,
  const StorageOptions & storage_options)
{
  // Since this is a reindexing operation, assume that there is no metadata.yaml file.
  // As such, ask the storage with the given URI for its metadata.
  storage_ = storage_factory_->open_read_only(
    database_file, storage_options.storage_id);
  if (!storage_) {
    throw std::runtime_error{"No storage could be initialized. Abort"};
  }
}

void SequentialReindexer::fill_topics_metadata()
{
  rcpputils::check_true(storage_ != nullptr, "Bag is not open. Call open() before reading.");
  topics_metadata_.clear();
  topics_metadata_.reserve(metadata_.topics_with_message_count.size());
  for (const auto & topic_information : metadata_.topics_with_message_count) {
    topics_metadata_.push_back(topic_information.topic_metadata);
  }
}

void SequentialReindexer::init_metadata(const std::vector<std::string> & files)
{
  metadata_ = rosbag2_storage::BagMetadata{};

  // This reindexer will only work on SQLite files, so this can't change
  metadata_.storage_identifier = "sqlite3";
  metadata_.starting_time = std::chrono::time_point<std::chrono::high_resolution_clock>(
    std::chrono::nanoseconds::max());

  // Record the relative paths to the metadata
  for (const auto & path : files) {
    auto cleaned_path = strip_parent_path(path);
    metadata_.relative_file_paths.push_back(cleaned_path);
  }
}

void SequentialReindexer::aggregate_metadata(
  const std::vector<std::string> & files, const StorageOptions & storage_options)
{
  // In order to most accurately reconstruct the metadata, we need to
  // visit each of the contained relative database files in the bag,
  // open them, slurp up the info, and stuff it into the master
  // metadata object.
  ROSBAG2_CPP_LOG_INFO("Extracting metadata from database(s)");
  for (const auto & f_ : files) {
    open(f_, storage_options);  // Class storage_ is now full

    auto temp_metadata = storage_->get_metadata();

    // Last opened file will have our starting time
    metadata_.starting_time = temp_metadata.starting_time;
    metadata_.duration += temp_metadata.duration;
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
    storage_.reset();  // Class storage_ is now empty
  }
}

void SequentialReindexer::reindex(const StorageOptions & storage_options)
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
  init_metadata(files);

  // Collect all metadata from database files
  aggregate_metadata(files, storage_options);

  // Perform final touch-up
  finalize_metadata();

  metadata_io_->write_metadata(base_folder_, metadata_);
  ROSBAG2_CPP_LOG_INFO("Reindexing operation completed.");
}

void SequentialReindexer::finalize_metadata()
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
