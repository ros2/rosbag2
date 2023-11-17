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
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <sstream>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rcpputils/filesystem_helper.hpp"

#include "rosbag2_cpp/info.hpp"
#include "rosbag2_cpp/logging.hpp"
#include "rosbag2_cpp/reindexer.hpp"

#include "rosbag2_storage/storage_options.hpp"

namespace rosbag2_cpp
{
namespace writers
{

static constexpr char const * kDefaultStorageID = "sqlite3";

namespace
{
std::string strip_parent_path(const std::string & relative_path)
{
  return rcpputils::fs::path(relative_path).filename().string();
}

/// @brief Stringify the current system time for use in filenames
/// @return ISO 8601 string 'yyyymmddThhmmssZ'
std::string now_datetime_string()
{
  char buf[sizeof "yyyymmddThhmmssZ"];
  auto now = std::chrono::system_clock::now();
  std::time_t now_t = std::chrono::system_clock::to_time_t(now);
  if (!std::strftime(buf, sizeof buf, "%Y%m%dT%H%M%SZ", std::gmtime(&now_t))) {
    throw std::runtime_error("Failed to format time string");
  }
  return std::string{buf};
}


}  // namespace

SequentialWriter::SequentialWriter(
  std::unique_ptr<rosbag2_storage::StorageFactoryInterface> storage_factory,
  std::shared_ptr<SerializationFormatConverterFactoryInterface> converter_factory,
  std::unique_ptr<rosbag2_storage::MetadataIo> metadata_io)
: storage_factory_(std::move(storage_factory)),
  converter_factory_(std::move(converter_factory)),
  storage_(nullptr),
  metadata_io_(std::move(metadata_io)),
  converter_(nullptr),
  topics_names_to_info_(),
  metadata_()
{}

SequentialWriter::~SequentialWriter()
{
  close();
}

std::optional<rosbag2_storage::BagMetadata> SequentialWriter::preinit_metadata(
  const rosbag2_storage::StorageOptions & storage_options)
{
  std::filesystem::path storage_path(storage_options.uri);
  if (std::filesystem::exists(storage_path) && !std::filesystem::is_directory(storage_path)) {
    std::stringstream error;
    error << "Cannot append files to non-directory file (" << storage_path.string() << ").";
    throw std::runtime_error{error.str()};
  } else if (!std::filesystem::exists(storage_path)) {
    return std::nullopt;
  }
  // Exists and is a directory
  rosbag2_storage::MetadataIo metadata_io;
  if (!metadata_io.metadata_file_exists(storage_options.uri)) {
    rosbag2_cpp::Reindexer reindexer;
    reindexer.reindex(storage_options);
  }
  assert(metadata_io.metadata_file_exists(storage_options.uri));
  auto metadata = metadata_io.read_metadata(storage_options.uri);

  if (storage_options.suffix_style == rosbag2_storage::FileSuffixStyle::Index) {
    // Check files in order, keeping the last index found
    const auto & files = metadata.relative_file_paths;
    for (auto it = files.begin(); it != files.end(); it++) {
      std::string stem = std::filesystem::path(*it).stem().string();
      size_t suffix_idx = stem.find_last_of('_');
      std::string suffix = stem.substr(suffix_idx + 1);
      if (std::regex_match(suffix, std::regex("\\d+"))) {
        // It's only an index if it's just digits, otherwise it's ignored
        bag_idx_ = std::stoul(suffix);
        ROSBAG2_CPP_LOG_DEBUG("Discovered an index %zu", bag_idx_);
      }
    }
    if (bag_idx_ > 0) {
      ROSBAG2_CPP_LOG_INFO(
        "Discovered existing file index %zu, starting at %zu", bag_idx_, bag_idx_ + 1);
      bag_idx_++;
    }
  }

  return metadata;
}

void SequentialWriter::init_metadata(std::optional<rosbag2_storage::BagMetadata> initial_value)
{
  if (initial_value) {
    metadata_ = *initial_value;
  } else {
    metadata_ = rosbag2_storage::BagMetadata{};
    metadata_.starting_time = std::chrono::time_point<std::chrono::high_resolution_clock>(
      std::chrono::nanoseconds::max());
  }

  while (
    storage_options_.max_bagfile_splits > 0 &&
    metadata_.relative_file_paths.size() >= storage_options_.max_bagfile_splits)
  {
    remove_first_file();
  }

  metadata_.storage_identifier = storage_->get_storage_identifier();
  metadata_.relative_file_paths.push_back(strip_parent_path(storage_->get_relative_file_path()));

  rosbag2_storage::FileInformation file_info{};
  file_info.path = strip_parent_path(storage_->get_relative_file_path());
  file_info.starting_time = std::chrono::time_point<std::chrono::high_resolution_clock>(
    std::chrono::nanoseconds::max());
  file_info.message_count = 0;
  metadata_.files.push_back(file_info);
  ROSBAG2_CPP_LOG_INFO("Opened %s for writing", metadata_.relative_file_paths.back().c_str());
}

void SequentialWriter::open(
  const rosbag2_storage::StorageOptions & storage_options,
  const ConverterOptions & converter_options)
{
  base_folder_ = storage_options.uri;
  storage_options_ = storage_options;
  if (storage_options_.storage_id.empty()) {
    storage_options_.storage_id = kDefaultStorageID;
  }

  if (converter_options.output_serialization_format !=
    converter_options.input_serialization_format)
  {
    converter_ = std::make_unique<Converter>(converter_options, converter_factory_);
  }

  std::optional<rosbag2_storage::BagMetadata> initial_metadata;
  rcpputils::fs::path db_path(storage_options.uri);

  if (storage_options.append_files) {
    initial_metadata = preinit_metadata(storage_options);
  } else if (db_path.is_directory()) {
    std::stringstream error;
    error << "Database directory already exists (" << db_path.string() <<
      "), can't overwrite existing database";
    throw std::runtime_error{error.str()};
  }

  bool dir_created = rcpputils::fs::create_directories(db_path);
  if (!dir_created) {
    std::stringstream error;
    error << "Failed to create database directory (" << db_path.string() << ").";
    throw std::runtime_error{error.str()};
  }

  storage_options_.uri = format_storage_uri(base_folder_, bag_idx_);
  storage_ = storage_factory_->open_read_write(storage_options_);
  if (!storage_) {
    throw std::runtime_error("No storage could be initialized. Abort");
  }

  if (storage_options_.max_bagfile_size != 0 &&
    storage_options_.max_bagfile_size < storage_->get_minimum_split_file_size())
  {
    std::stringstream error;
    error << "Invalid bag splitting size given. Please provide a value greater than " <<
      storage_->get_minimum_split_file_size() << ". Specified value of " <<
      storage_options.max_bagfile_size;
    throw std::runtime_error{error.str()};
  }

  if (storage_options_.max_bagfile_splits > rosbag2_storage::max_allowed_file_splits) {
    std::stringstream error;
    error << "Invalid value for max_bagfile_splits given. The maximum allowed value of "
      "is: " << rosbag2_storage::max_allowed_file_splits <<
      ". Specified value: " << storage_options_.max_bagfile_splits;
    throw std::runtime_error{error.str()};
  }

  use_cache_ = storage_options.max_cache_size > 0u;
  if (storage_options.snapshot_mode && !use_cache_) {
    throw std::runtime_error(
            "Max cache size must be greater than 0 when snapshot mode is enabled");
  }

  if (use_cache_) {
    if (storage_options.snapshot_mode) {
      message_cache_ = std::make_shared<rosbag2_cpp::cache::CircularMessageCache>(
        storage_options.max_cache_size);
    } else {
      message_cache_ = std::make_shared<rosbag2_cpp::cache::MessageCache>(
        storage_options.max_cache_size);
    }
    cache_consumer_ = std::make_unique<rosbag2_cpp::cache::CacheConsumer>(
      message_cache_,
      std::bind(&SequentialWriter::write_messages, this, std::placeholders::_1));
  }

  init_metadata(initial_metadata);
}

void SequentialWriter::close()
{
  if (use_cache_) {
    // destructor will flush message cache
    cache_consumer_.reset();
    message_cache_.reset();
  }

  if (!base_folder_.empty()) {
    finalize_metadata();
    metadata_io_->write_metadata(base_folder_, metadata_);
  }

  storage_.reset();  // Necessary to ensure that the storage is destroyed before the factory
  storage_factory_.reset();
}

void SequentialWriter::create_topic(const rosbag2_storage::TopicMetadata & topic_with_type)
{
  if (topics_names_to_info_.find(topic_with_type.name) !=
    topics_names_to_info_.end())
  {
    // nothing to do, topic already created
    return;
  }

  if (!storage_) {
    throw std::runtime_error("Bag is not open. Call open() before writing.");
  }

  rosbag2_storage::TopicInformation info{};
  info.topic_metadata = topic_with_type;

  bool insert_succeded = false;
  {
    std::lock_guard<std::mutex> lock(topics_info_mutex_);
    const auto insert_res = topics_names_to_info_.insert(
      std::make_pair(topic_with_type.name, info));
    insert_succeded = insert_res.second;
  }

  if (!insert_succeded) {
    std::stringstream errmsg;
    errmsg << "Failed to insert topic \"" << topic_with_type.name << "\"!";

    throw std::runtime_error(errmsg.str());
  }

  storage_->create_topic(topic_with_type);

  if (converter_) {
    converter_->add_topic(topic_with_type.name, topic_with_type.type);
  }
}

void SequentialWriter::remove_topic(const rosbag2_storage::TopicMetadata & topic_with_type)
{
  if (!storage_) {
    throw std::runtime_error("Bag is not open. Call open() before removing.");
  }

  bool erased = false;
  {
    std::lock_guard<std::mutex> lock(topics_info_mutex_);
    erased = topics_names_to_info_.erase(topic_with_type.name) > 0;
  }

  if (erased) {
    storage_->remove_topic(topic_with_type);
  } else {
    std::stringstream errmsg;
    errmsg << "Failed to remove the non-existing topic \"" <<
      topic_with_type.name << "\"!";

    throw std::runtime_error(errmsg.str());
  }
}

std::string SequentialWriter::format_storage_uri(
  const std::string & base_folder, uint64_t storage_count)
{
  // Right now `base_folder_` is always just the folder name for where to install the bagfile.
  // The name of the folder needs to be queried in case
  // SequentialWriter is opened with a relative path.
  rcpputils::fs::path base_dir{base_folder};
  std::string file_prefix = base_dir.filename().string() + "_";
  std::string storage_file_name;

  switch (storage_options_.suffix_style) {
    case rosbag2_storage::FileSuffixStyle::Index:
      storage_file_name = file_prefix + std::to_string(storage_count);
      break;
    case rosbag2_storage::FileSuffixStyle::Datetime:
      storage_file_name = file_prefix + now_datetime_string();
      if (storage_) {
        // If there is a previous file, and it has the same timestamp to the second,
        // append an increasing index on it, for rare case of bags split in less than a second.
        std::filesystem::path prev_stem{storage_->get_relative_file_path()};
        prev_stem = prev_stem.filename().replace_extension();
        size_t index = 0;
        if (prev_stem.has_extension()) {
          // The index is the remaining extension (excluding first character '.')
          index = std::stoul(prev_stem.extension().string().substr(1));
          prev_stem = prev_stem.replace_extension();
        }
        if (prev_stem == storage_file_name) {
          storage_file_name += "." + std::to_string(index + 1);
        }
      }
      break;
    default:
      throw std::runtime_error("Unhandled file suffix style");
  }

  return (base_dir / storage_file_name).string();
}

void SequentialWriter::switch_to_next_storage()
{
  // consume remaining message cache
  if (use_cache_) {
    cache_consumer_->stop();
    message_cache_->log_dropped();
  }

  storage_options_.uri = format_storage_uri(
    base_folder_,
    ++bag_idx_);
  storage_ = storage_factory_->open_read_write(storage_options_);

  if (!storage_) {
    std::stringstream errmsg;
    errmsg << "Failed to rollover bagfile to new file: \"" << storage_options_.uri << "\"!";

    throw std::runtime_error(errmsg.str());
  }

  // Re-register all topics since we rolled-over to a new bagfile.
  for (const auto & topic : topics_names_to_info_) {
    storage_->create_topic(topic.second.topic_metadata);
  }

  if (use_cache_) {
    // restart consumer thread for cache
    cache_consumer_->start();
  }
}

void SequentialWriter::split_bagfile()
{
  auto info = std::make_shared<bag_events::BagSplitInfo>();
  info->closed_file = storage_->get_relative_file_path();
  switch_to_next_storage();
  info->opened_file = storage_->get_relative_file_path();

  if (
    storage_options_.max_bagfile_splits > 0 &&
    metadata_.relative_file_paths.size() >= storage_options_.max_bagfile_splits)
  {
    // We are about to create a new file when switching to next storage
    // so if max splits is set, delete a file to make room
    remove_first_file();
  }

  metadata_.relative_file_paths.push_back(strip_parent_path(storage_->get_relative_file_path()));

  rosbag2_storage::FileInformation file_info{};
  file_info.starting_time = std::chrono::time_point<std::chrono::high_resolution_clock>(
    std::chrono::nanoseconds::max());
  file_info.path = strip_parent_path(storage_->get_relative_file_path());
  metadata_.files.push_back(file_info);

  ROSBAG2_CPP_LOG_INFO("Opened %s for writing", metadata_.relative_file_paths.back().c_str());

  callback_manager_.execute_callbacks(bag_events::BagEvent::WRITE_SPLIT, info);
}

void SequentialWriter::write(std::shared_ptr<rosbag2_storage::SerializedBagMessage> message)
{
  if (!storage_) {
    throw std::runtime_error("Bag is not open. Call open() before writing.");
  }

  // Get TopicInformation handler for counting messages.
  rosbag2_storage::TopicInformation * topic_information {nullptr};
  try {
    topic_information = &topics_names_to_info_.at(message->topic_name);
  } catch (const std::out_of_range & /* oor */) {
    std::stringstream errmsg;
    errmsg << "Failed to write on topic '" << message->topic_name <<
      "'. Call create_topic() before first write.";
    throw std::runtime_error(errmsg.str());
  }

  const auto message_timestamp = std::chrono::time_point<std::chrono::high_resolution_clock>(
    std::chrono::nanoseconds(message->time_stamp));

  if (is_first_message_) {
    // Update bagfile starting time
    metadata_.starting_time = message_timestamp;
    is_first_message_ = false;
  }

  if (should_split_bagfile(message_timestamp)) {
    split_bagfile();
    metadata_.files.back().starting_time = message_timestamp;
  }

  metadata_.starting_time = std::min(metadata_.starting_time, message_timestamp);

  metadata_.files.back().starting_time =
    std::min(metadata_.files.back().starting_time, message_timestamp);

  const auto duration = message_timestamp - metadata_.starting_time;
  metadata_.duration = std::max(metadata_.duration, duration);

  const auto file_duration = message_timestamp - metadata_.files.back().starting_time;
  metadata_.files.back().duration =
    std::max(metadata_.files.back().duration, file_duration);

  auto converted_msg = get_writeable_message(message);

  metadata_.files.back().message_count++;
  if (storage_options_.max_cache_size == 0u) {
    // If cache size is set to zero, we write to storage directly
    storage_->write(converted_msg);
    ++topic_information->message_count;
  } else {
    // Otherwise, use cache buffer
    message_cache_->push(converted_msg);
  }
}

bool SequentialWriter::take_snapshot()
{
  if (!storage_options_.snapshot_mode) {
    ROSBAG2_CPP_LOG_WARN("SequentialWriter take_snaphot called when snapshot mode is disabled");
    return false;
  }
  message_cache_->notify_data_ready();
  return true;
}

std::shared_ptr<rosbag2_storage::SerializedBagMessage>
SequentialWriter::get_writeable_message(
  std::shared_ptr<rosbag2_storage::SerializedBagMessage> message)
{
  return converter_ ? converter_->convert(message) : message;
}

bool SequentialWriter::should_split_bagfile(
  const std::chrono::time_point<std::chrono::high_resolution_clock> & current_time) const
{
  // Assume we aren't splitting
  bool should_split = false;

  // Splitting by size
  if (storage_options_.max_bagfile_size !=
    rosbag2_storage::storage_interfaces::MAX_BAGFILE_SIZE_NO_SPLIT)
  {
    should_split = (storage_->get_bagfile_size() >= storage_options_.max_bagfile_size);
  }

  // Splitting by time
  if (storage_options_.max_bagfile_duration !=
    rosbag2_storage::storage_interfaces::MAX_BAGFILE_DURATION_NO_SPLIT)
  {
    auto max_duration_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::seconds(storage_options_.max_bagfile_duration));
    should_split = should_split ||
      ((current_time - metadata_.files.back().starting_time) > max_duration_ns);
  }

  return should_split;
}

void SequentialWriter::finalize_metadata()
{
  metadata_.bag_size = 0;

  for (const auto & path : metadata_.relative_file_paths) {
    const auto bag_path = rcpputils::fs::path{path};

    if (bag_path.exists()) {
      metadata_.bag_size += bag_path.file_size();
    }
  }

  metadata_.topics_with_message_count.clear();
  metadata_.topics_with_message_count.reserve(topics_names_to_info_.size());
  metadata_.message_count = 0;

  for (const auto & topic : topics_names_to_info_) {
    metadata_.topics_with_message_count.push_back(topic.second);
    metadata_.message_count += topic.second.message_count;
  }
}

void SequentialWriter::write_messages(
  const std::vector<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>> & messages)
{
  if (messages.empty()) {
    return;
  }
  storage_->write(messages);
  std::lock_guard<std::mutex> lock(topics_info_mutex_);
  for (const auto & msg : messages) {
    if (topics_names_to_info_.find(msg->topic_name) != topics_names_to_info_.end()) {
      topics_names_to_info_[msg->topic_name].message_count++;
    }
  }
}

void SequentialWriter::add_event_callbacks(const bag_events::WriterEventCallbacks & callbacks)
{
  if (callbacks.write_split_callback) {
    callback_manager_.add_event_callback(
      callbacks.write_split_callback,
      bag_events::BagEvent::WRITE_SPLIT);
  }
}

void SequentialWriter::remove_first_file()
{
  if (metadata_.relative_file_paths.empty()) {
    return;
  }
  auto last_file_end_time = metadata_.files.back().starting_time + metadata_.files.back().duration;

  const auto expired_db_path = (
    std::filesystem::path(base_folder_) / metadata_.relative_file_paths.front());
  if (!std::filesystem::exists(expired_db_path)) {
    ROSBAG2_CPP_LOG_WARN_STREAM(
      "Couldn't remove fie " << expired_db_path <<
        " which doesn't exist any more. Ignoring.");
  } else if (std::filesystem::remove(expired_db_path)) {
    ROSBAG2_CPP_LOG_INFO_STREAM("Removed file " << expired_db_path);
  } else {
    ROSBAG2_CPP_LOG_ERROR_STREAM(
      "Failed to remove file " << expired_db_path <<
        " - removing from metadata and ignoring. May affect ability to enforce max_bagfile_splits");
  }

  metadata_.message_count -= metadata_.files.begin()->message_count;
  // TODO(hrfuller) Should update topics_names_to_info as well but the
  // per topic metadata isn't recorded per file.
  metadata_.relative_file_paths.erase(metadata_.relative_file_paths.begin());
  metadata_.files.erase(metadata_.files.begin());
  // Update the global bag metadata with new times excluding the bag that was removed.
  metadata_.starting_time = metadata_.files.front().starting_time;
  metadata_.duration = last_file_end_time - metadata_.starting_time;
}

}  // namespace writers
}  // namespace rosbag2_cpp
