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
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <memory>
#include <regex>
#include <stdexcept>
#include <string>
#include <sstream>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rcpputils/env.hpp"

#include "rosbag2_cpp/info.hpp"
#include "rosbag2_cpp/logging.hpp"
#include "rosbag2_storage/default_storage_id.hpp"
#include "rosbag2_storage/storage_options.hpp"

namespace fs = std::filesystem;

namespace rosbag2_cpp
{
namespace writers
{

namespace
{
std::string strip_parent_path(const std::string & relative_path)
{
  return fs::path(relative_path).filename().generic_string();
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
  transient_local_cache_(std::make_shared<rosbag2_cpp::cache::TransientLocalMessagesCache>()),
  metadata_()
{}

SequentialWriter::~SequentialWriter()
{
  // Deleting all callbacks before calling close(). Calling callbacks from destructor is not safe.
  // Callbacks likely was created after SequentialWriter object and may point to the already
  // destructed objects.
  callback_manager_.delete_all_callbacks();
  SequentialWriter::close();
}

void SequentialWriter::init_metadata()
{
  metadata_ = rosbag2_storage::BagMetadata{};
  metadata_.storage_identifier = storage_->get_storage_identifier();
  metadata_.starting_time = std::chrono::time_point<std::chrono::high_resolution_clock>(
    std::chrono::nanoseconds::max());
  metadata_.duration = std::chrono::nanoseconds(0);
  metadata_.relative_file_paths = {strip_parent_path(storage_->get_relative_file_path())};
  rosbag2_storage::FileInformation file_info{};
  file_info.path = strip_parent_path(storage_->get_relative_file_path());
  file_info.starting_time = std::chrono::time_point<std::chrono::high_resolution_clock>(
    std::chrono::nanoseconds::max());
  file_info.duration = std::chrono::nanoseconds(0);
  file_info.message_count = 0;
  metadata_.custom_data = storage_options_.custom_data;
  metadata_.files = {file_info};
  per_file_topic_message_counts_.clear();
  per_file_topic_message_counts_.emplace_back();  // Initialize tracking for first file
  metadata_.ros_distro = rcpputils::get_env_var("ROS_DISTRO");
  if (metadata_.ros_distro.empty()) {
    ROSBAG2_CPP_LOG_WARN(
      "Environment variable ROS_DISTRO not set, can't store value in bag metadata.");
  }
}

void SequentialWriter::open(
  const rosbag2_storage::StorageOptions & storage_options,
  const ConverterOptions & converter_options)
{
  // Note. close and open methods protected with mutex on upper rosbag2_cpp::writer level.
  if (is_open_) {
    return;  // The writer already opened
  }
  if (storage_options.uri.empty()) {
    throw std::runtime_error("Can't open rosbag2_cpp::SequentialWriter. The input URI is empty");
  }

  base_folder_ = storage_options.uri;
  storage_options_ = storage_options;

  if (storage_options_.storage_id.empty()) {
    storage_options_.storage_id = rosbag2_storage::get_default_storage_id();
  }

  if (converter_options.output_serialization_format !=
    converter_options.input_serialization_format)
  {
    converter_ = std::make_unique<Converter>(converter_options, converter_factory_);
  }

  fs::path storage_path(storage_options.uri);
  if (fs::is_directory(storage_path)) {
    std::stringstream error;
    error << "Bag directory already exists (" << storage_path.string() <<
      "), can't overwrite existing bag";
    throw std::runtime_error{error.str()};
  }

  bool dir_created = fs::create_directories(storage_path);
  if (!dir_created) {
    std::stringstream error;
    error << "Failed to create bag directory (" << storage_path.string() << ").";
    throw std::runtime_error{error.str()};
  }

  storage_options_.uri = format_storage_uri(base_folder_, 0);
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

  use_cache_ =
    storage_options.max_cache_size > 0u || storage_options.max_cache_duration > 0u;

  if (storage_options.snapshot_mode && !use_cache_) {
    throw std::runtime_error(
            "Either the max cache size or the maximum cache duration must be greater than 0"
            " when snapshot mode is enabled");
  }

  if (use_cache_) {
    if (storage_options.snapshot_mode) {
      message_cache_ = std::make_shared<rosbag2_cpp::cache::CircularMessageCache>(
        storage_options.max_cache_size, storage_options.max_cache_duration);
    } else {
      message_cache_ = std::make_shared<rosbag2_cpp::cache::MessageCache>(
        storage_options.max_cache_size, storage_options.max_cache_duration);
    }
    cache_consumer_ = std::make_unique<rosbag2_cpp::cache::CacheConsumer>(
      message_cache_,
      std::bind(&SequentialWriter::write_messages, this, std::placeholders::_1));
  }

  init_metadata();
  // Register topics in storage if they already exists
  metadata_.topics_with_message_count.clear();
  metadata_.topics_with_message_count.reserve(topics_names_to_info_.size());
  for (auto & [topic_name, topic_info] : topics_names_to_info_) {
    topic_info.message_count = 0U;

    metadata_.topics_with_message_count.push_back(topic_info);
    // Adjust serialization format if converter is used. Note: Do not modify the serialization
    // format in the original topics_names_to_info_ map, since it is persist
    // across close()->open() calls.
    auto & topic_metadata = metadata_.topics_with_message_count.back().topic_metadata;
    if (converter_) {
      topic_metadata.serialization_format = converter_->get_output_serialization_format();
      converter_->add_topic(topic_name, topic_info.topic_metadata.type);
    }
    auto const & md = topic_names_to_message_definitions_[topic_name];
    storage_->create_topic(topic_metadata, md);
  }
  storage_->update_metadata(metadata_);
  next_file_index_ = 1;  // First file is 0, next will be 1
  is_open_ = true;
}

void SequentialWriter::flush_cache_update_metadata_and_close_storage()
{
  if (use_cache_) {
    // destructor will flush message cache
    cache_consumer_.reset();
    message_cache_.reset();
  }
  finalize_metadata();
  if (storage_) {
    storage_->update_metadata(metadata_);
    storage_.reset();  // Destroy storage before calling WRITE_SPLIT callback to make sure that
    // bag file was closed before callback call.
  }
}

void SequentialWriter::close()
{
  // Note. close and open methods protected with mutex on upper rosbag2_cpp::writer level.
  if (!is_open_.exchange(false)) {
    return;  // The writer is not open
  }

  flush_cache_update_metadata_and_close_storage();

  if (!metadata_.relative_file_paths.empty()) {
    // Take the latest file name from metadata in case if it was updated after compression in
    // derived class
    auto closed_file =
      (fs::path(base_folder_) / metadata_.relative_file_paths.back()).generic_string();
    execute_bag_split_callbacks(closed_file, "");
  }

  if (!base_folder_.empty()) {
    metadata_io_->write_metadata(base_folder_, metadata_);
  }

  // Zero message counts for all topics
  std::lock_guard<std::mutex> lock(topics_info_mutex_);
  for (auto & [_, topic_info] : topics_names_to_info_) {
    topic_info.message_count = 0U;
  }

  converter_.reset();
}

void SequentialWriter::create_topic(const rosbag2_storage::TopicMetadata & topic_with_type)
{
  // Don't need to lock topics_info_mutex_ since we are not modifying topics_names_to_info_ here
  if (topics_names_to_info_.find(topic_with_type.name) != topics_names_to_info_.end()) {
    return;  // nothing to do, topic already created
  }
  rosbag2_storage::MessageDefinition definition =
    message_definitions_.get_full_text_ext(topic_with_type.type, topic_with_type.name);

  if (definition.encoded_message_definition.empty() ||
    definition.encoding.empty() || definition.encoding == "unknown")
  {
    ROSBAG2_CPP_LOG_WARN("Message definition for topic '%s' with type '%s' not found. "
      "Message definition will be left empty in bag.",
      topic_with_type.name.c_str(), topic_with_type.type.c_str());
    definition =
      rosbag2_storage::MessageDefinition::empty_message_definition_for(topic_with_type.type);
  }

  // Copy hash from topic_with_type to message definition
  definition.type_hash = topic_with_type.type_description_hash;
  create_topic(topic_with_type, definition);
}

void SequentialWriter::create_topic(
  const rosbag2_storage::TopicMetadata & topic_with_type,
  const rosbag2_storage::MessageDefinition & message_definition)
{
  rosbag2_storage::TopicInformation info{};
  {
    std::lock_guard<std::mutex> lock(topics_info_mutex_);
    if (topics_names_to_info_.find(topic_with_type.name) != topics_names_to_info_.end()) {
      return;  // nothing to do, topic already created
    }
    info.topic_metadata = topic_with_type;
    (void)topics_names_to_info_.insert({topic_with_type.name, info});
    (void)topic_names_to_message_definitions_.insert({topic_with_type.name, message_definition});
  }

  if (is_open_.load()) {
    // Adjust serialization format if converter is used
    if (converter_) {
      info.topic_metadata.serialization_format = converter_->get_output_serialization_format();
      converter_->add_topic(info.topic_metadata.name, info.topic_metadata.type);
    }
    storage_->create_topic(info.topic_metadata, message_definition);
    metadata_.topics_with_message_count.push_back(info);
  }
}

void SequentialWriter::create_transient_local_topic(
  const rosbag2_storage::TopicMetadata & topic_with_type,
  size_t num_last_messages)
{
  transient_local_cache_->add_topic(topic_with_type.name, num_last_messages);
  create_topic(topic_with_type);
}

void SequentialWriter::create_transient_local_topic(
  const rosbag2_storage::TopicMetadata & topic_with_type,
  size_t num_last_messages,
  const rosbag2_storage::MessageDefinition & message_definition)
{
  transient_local_cache_->add_topic(topic_with_type.name, num_last_messages);
  create_topic(topic_with_type, message_definition);
}

void SequentialWriter::remove_topic(const rosbag2_storage::TopicMetadata & topic_with_type)
{
  if (transient_local_cache_->has_topic(topic_with_type.name)) {
    transient_local_cache_->remove_topic(topic_with_type.name);
  }
  std::lock_guard<std::mutex> lock(topics_info_mutex_);
  bool erased = topics_names_to_info_.erase(topic_with_type.name) > 0;
  erased = erased && (topic_names_to_message_definitions_.erase(topic_with_type.name) > 0);

  if (erased) {
    if (is_open_.load()) {
      storage_->remove_topic(topic_with_type);
    }
  } else {
    std::stringstream errmsg;
    errmsg << "Failed to remove the non-existing topic \"" << topic_with_type.name << "\"!";
    throw std::runtime_error(errmsg.str());
  }
}

std::string SequentialWriter::format_storage_uri(
  const std::string & base_folder, uint64_t storage_count)
{
  // Extract prefix from directory name by removing timestamp pattern if present
  // This handles the case when --output is not specified and default timestamped directory is used
  // Currently, the default timestamp format is `YYYY_MM_DD-HH_MM_SS`
  std::string dir_name = fs::path(base_folder).filename().generic_string();
  // Handle edge case where filename() returns empty (e.g., base_folder is "/" or ".")
  // This should not happen in practice since base_folder is validated in open(), but
  // we add this check for defensive programming.
  if (dir_name.empty()) {
    dir_name = "rosbag2";  // Use default prefix
  }
  static std::regex timestamp_pattern("_" + std::string(TIMESTAMP_PATTERN) + "$");
  std::string prefix = std::regex_replace(dir_name, timestamp_pattern, "");

  // Generate timestamp in local time.
  // Note: During DST switches the same string may occur twice. However, we're also adding the
  // sequence counter as part of the filename, so duplicates still remain distinguishable.
  auto time_t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::tm timestamp{};
#ifdef _WIN32
  localtime_s(&timestamp, &time_t);
#else
  localtime_r(&time_t, &timestamp);
#endif

  // Generate filename in format {storage_count}_{prefix}_{timestamp}
  // Note: Underscores are used as separators. If the prefix contains underscores, this creates
  // theoretical ambiguity when parsing filenames. However, parsing is typically done by matching
  // the timestamp pattern from the end, which avoids ambiguity in practice.
  std::stringstream storage_file_name;
  storage_file_name << storage_count << "_" << prefix << "_" <<
    std::put_time(&timestamp, "%Y_%m_%d-%H_%M_%S");

  return (fs::path(base_folder) / storage_file_name.str()).generic_string();
}

void SequentialWriter::switch_to_next_storage()
{
  // consume remaining message cache
  if (use_cache_) {
    cache_consumer_->stop();
    message_cache_->log_dropped();
  }

  finalize_metadata();
  storage_->update_metadata(metadata_);

  // Check for overflow: if next_file_index_ is 0, we've wrapped around (very unlikely but possible)
  if (next_file_index_ == 0) {
    ROSBAG2_CPP_LOG_WARN_STREAM(
      "File index counter has overflowed (wrapped to 0). "
      "This should not happen in practice, but continuing with index 0. "
      "If circular logging is enabled, ensure old files are deleted to avoid conflicts.");
  }

  storage_options_.uri = format_storage_uri(base_folder_, next_file_index_);
  next_file_index_++;
  // TODO(morlov): If we would ever remove the upper level writer mutex lock, consider protecting
  //  storage_ with mutex to avoid race conditions with write(msg) call when we are switching to
  //  next storage and not using cache.
  storage_ = storage_factory_->open_read_write(storage_options_);
  if (!storage_) {
    std::stringstream errmsg;
    errmsg << "Failed to rollover bagfile to new file: \"" << storage_options_.uri << "\"!";

    throw std::runtime_error(errmsg.str());
  }

  rosbag2_storage::FileInformation file_info{};
  file_info.starting_time =
    std::chrono::time_point<std::chrono::high_resolution_clock>(std::chrono::nanoseconds::max());
  file_info.path = strip_parent_path(storage_->get_relative_file_path());
  metadata_.files.push_back(file_info);
  metadata_.relative_file_paths.push_back(file_info.path);
  per_file_topic_message_counts_.emplace_back();  // Initialize tracking for new file

  // Delete oldest files if circular buffer limit exceeded (after new file is added)
  delete_oldest_files_if_needed();

  finalize_metadata();
  storage_->update_metadata(metadata_);
  {
    // Re-register all topics since we rolled-over to a new bagfile.
    std::lock_guard<std::mutex> lock(topics_info_mutex_);
    for (const auto & topic : topics_names_to_info_) {
      auto const & md = topic_names_to_message_definitions_[topic.first];
      storage_->create_topic(topic.second.topic_metadata, md);
    }
  }

  if (use_cache_) {
    // restart consumer thread for cache
    cache_consumer_->start();
  }
}

std::string SequentialWriter::split_bagfile_local(bool execute_callbacks)
{
  auto closed_file = storage_->get_relative_file_path();
  switch_to_next_storage();
  // In non-snapshot mode, write cached transient-local messages to the new bag file so that
  // transient-local topics appear in every split. In snapshot mode the merge happens
  // later inside write_messages() where the circular buffer is flushed together with the snapshot.
  if (!storage_options_.snapshot_mode) {
    write_transient_local_messages(last_recv_timestamp_, last_sent_timestamp_);
  }
  auto opened_file = storage_->get_relative_file_path();

  if (execute_callbacks) {
    execute_bag_split_callbacks(closed_file, opened_file);
  }
  return opened_file;
}

void SequentialWriter::execute_bag_split_callbacks(
  const std::string & closed_file, const std::string & opened_file)
{
  auto info = std::make_shared<bag_events::BagSplitInfo>();
  info->closed_file = closed_file;
  info->opened_file = opened_file;
  callback_manager_.execute_callbacks(bag_events::BagEvent::WRITE_SPLIT, info);
}

void SequentialWriter::split_bagfile()
{
  (void)split_bagfile_local();
}

void SequentialWriter::write(std::shared_ptr<const rosbag2_storage::SerializedBagMessage> message)
{
  if (!is_open_) {
    throw std::runtime_error("Bag is not open. Call open() before writing.");
  }

  if (!message || !message_within_accepted_time_range(message->recv_timestamp)) {
    return;
  }

  // Get TopicInformation handler for counting messages.
  rosbag2_storage::TopicInformation * topic_information_ptr{nullptr};
  const auto & topic_name = message->topic_name;
  if (const auto it = topics_names_to_info_.find(topic_name); it != topics_names_to_info_.end()) {
    topic_information_ptr = &(it->second);
  } else {
    std::stringstream errmsg;
    errmsg << "Failed to write on topic '" << topic_name <<
      "'. Call create_topic() before first write.";
    throw std::runtime_error(errmsg.str());
  }

  const auto message_timestamp = std::chrono::time_point<std::chrono::high_resolution_clock>(
    std::chrono::nanoseconds(message->recv_timestamp));

  if (is_first_message_) {
    // Update bagfile starting time
    metadata_.starting_time = message_timestamp;
    is_first_message_ = false;
  }

  if (!storage_options_.snapshot_mode && should_split_bagfile(message_timestamp)) {
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
  if (transient_local_cache_->has_topic(topic_name)) {
    transient_local_cache_->push(topic_name, converted_msg);
  }

  bool message_lost = false;
  if (storage_options_.max_cache_size == 0u && storage_options_.max_cache_duration == 0u) {
    // If cache size is set to zero, we write to storage directly
    if (storage_->write_message(converted_msg)) {
      metadata_.files.back().message_count++;
      topic_information_ptr->message_count++;
      per_file_topic_message_counts_.back()[message->topic_name]++;
    } else {
      message_lost = true;
    }
  } else {
    // Otherwise, use cache buffer
    message_lost = !message_cache_->push(converted_msg);
  }

  if (message_lost) {
    // Process message lost event
    auto msgs_lost_info = std::make_shared<std::vector<bag_events::MessagesLostInfo>>();
    msgs_lost_info->emplace_back(bag_events::MessagesLostInfo{message->topic_name, 1});
    on_messages_lost(std::move(msgs_lost_info));
  }

  last_recv_timestamp_ = message->recv_timestamp;
  last_sent_timestamp_ = message->send_timestamp;
}

bool SequentialWriter::take_snapshot()
{
  if (!storage_options_.snapshot_mode) {
    ROSBAG2_CPP_LOG_WARN("SequentialWriter take_snapshot called when snapshot mode is disabled");
    return false;
  }
  // Note: Information about start, duration and num messages for the current file in metadata_
  // will be updated in the write_messages(..), when cache_consumer call it as a callback.
  message_cache_->notify_data_ready();
  split_bagfile();
  return true;
}

std::shared_ptr<const rosbag2_storage::SerializedBagMessage>
SequentialWriter::get_writeable_message(
  std::shared_ptr<const rosbag2_storage::SerializedBagMessage> message)
{
  return converter_ ? converter_->convert(message) : message;
}

void SequentialWriter::write_transient_local_messages(
  rcutils_time_point_value_t recv_timestamp,
  rcutils_time_point_value_t send_timestamp)
{
  auto transient_messages = transient_local_cache_->get_messages_sorted_by_timestamp();
  if (transient_messages.empty()) {
    return;
  }

  // Adjust timestamps directly on the mutable messages returned by the cache.
  for (auto & msg : transient_messages) {
    msg->recv_timestamp = recv_timestamp;
    msg->send_timestamp = send_timestamp;
  }

  rosbag2_storage::SerializedBagMessages const_messages(
    transient_messages.begin(), transient_messages.end());
  auto lost_messages_idx = storage_->write_messages(const_messages);
  auto written_messages_count = const_messages.size() - lost_messages_idx.size();

  metadata_.message_count += written_messages_count;
  metadata_.files.back().message_count += written_messages_count;
  const auto prepend_time = std::chrono::time_point<std::chrono::high_resolution_clock>(
    std::chrono::nanoseconds(recv_timestamp));
  metadata_.starting_time = std::min(metadata_.starting_time, prepend_time);
  metadata_.files.back().starting_time =
    std::min(metadata_.files.back().starting_time, prepend_time);

  {
    std::lock_guard<std::mutex> lock(topics_info_mutex_);
    for (size_t i = 0; i < const_messages.size(); i++) {
      if (!lost_messages_idx.empty()) {
        auto is_lost = std::binary_search(lost_messages_idx.begin(), lost_messages_idx.end(), i);
        if (is_lost) {
          continue;
        }
      }

      const auto & message = const_messages[i];
      auto topic_info_it = topics_names_to_info_.find(message->topic_name);
      if (topic_info_it != topics_names_to_info_.end()) {
        topic_info_it->second.message_count++;
        per_file_topic_message_counts_.back()[message->topic_name]++;
      }
    }
  }

  // Notify about lost messages via callback
  if (!lost_messages_idx.empty()) {
    std::unordered_map<std::string, size_t> lost_messages_count;
    for (const auto & lost_message_index : lost_messages_idx) {
      const auto & topic_name = const_messages[lost_message_index]->topic_name;
      lost_messages_count[topic_name]++;
    }
    auto msgs_lost_info = std::make_shared<std::vector<bag_events::MessagesLostInfo>>();
    msgs_lost_info->reserve(lost_messages_count.size());
    for (const auto & [topic_name, count] : lost_messages_count) {
      msgs_lost_info->emplace_back(bag_events::MessagesLostInfo{topic_name, count});
    }
    on_messages_lost(std::move(msgs_lost_info));
  }
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
    // TODO(morlov): consider cached messages size in splitting decision. Right now we only consider
    //  the size of already written messages in storage. Add message_cache_->get_current_size() API.
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

void SequentialWriter::delete_oldest_files_if_needed()
{
  if (storage_options_.max_bag_files == 0) {
    return;
  }

  // Remove oldest file from tracking: adjust per-topic message counts and metadata
  auto remove_oldest_file_from_tracking = [&]()
    {
      {
        std::lock_guard<std::mutex> lock(topics_info_mutex_);
        const auto & oldest_topic_counts = per_file_topic_message_counts_.front();
        for (const auto & [topic_name, count] : oldest_topic_counts) {
          auto it = topics_names_to_info_.find(topic_name);
          if (it != topics_names_to_info_.end()) {
            it->second.message_count -= count;
          }
        }
      }
      per_file_topic_message_counts_.pop_front();
      metadata_.relative_file_paths.erase(metadata_.relative_file_paths.begin());
      metadata_.files.erase(metadata_.files.begin());
      if (!metadata_.files.empty()) {
        metadata_.starting_time = metadata_.files.front().starting_time;
      }
    };

  // Delete the oldest files until we're under the bag file count limit
  while (metadata_.files.size() > storage_options_.max_bag_files) {
    const auto & oldest_file = metadata_.files.front();
    const auto file_path = fs::path(base_folder_) / oldest_file.path;

    // Delete file from filesystem
    if (fs::exists(file_path)) {
      const auto file_size = fs::file_size(file_path);
      const auto file_duration_ns = oldest_file.duration.count();
      std::error_code ec;
      bool file_removed = fs::remove(file_path, ec);
      if (!file_removed || ec) {
        ROSBAG2_CPP_LOG_ERROR(
          "Failed to delete oldest bagfile: %s. Error: %s",
                              file_path.generic_string().c_str(), ec.message().c_str());
        break;  // Keep file in tracking; retry on next split to avoid tight loop
      }
      ROSBAG2_CPP_LOG_INFO(
        "Deleted oldest bagfile: %s (%lu bytes, %lu ns)",
                           oldest_file.path.c_str(), file_size, file_duration_ns);
      remove_oldest_file_from_tracking();
    } else {
      ROSBAG2_CPP_LOG_ERROR("Oldest bagfile to delete not found: %s",
                            file_path.generic_string().c_str());
      remove_oldest_file_from_tracking();
    }
  }
}

bool SequentialWriter::message_within_accepted_time_range(
  const rcutils_time_point_value_t current_time) const
{
  if (storage_options_.start_time_ns >= 0 &&
    static_cast<int64_t>(current_time) < storage_options_.start_time_ns)
  {
    return false;
  }

  if (storage_options_.end_time_ns >= 0 &&
    static_cast<int64_t>(current_time) > storage_options_.end_time_ns)
  {
    return false;
  }

  return true;
}

void SequentialWriter::finalize_metadata()
{
  metadata_.bag_size = 0;

  for (const auto & path : metadata_.relative_file_paths) {
    const auto bag_path = fs::path{path};

    if (fs::exists(bag_path)) {
      metadata_.bag_size += fs::file_size(bag_path);
    }
  }

  metadata_.topics_with_message_count.clear();
  metadata_.topics_with_message_count.reserve(topics_names_to_info_.size());
  metadata_.message_count = 0;

  for (const auto & [_, topic_info] : topics_names_to_info_) {
    metadata_.topics_with_message_count.push_back(topic_info);
    metadata_.message_count += topic_info.message_count;
    if (converter_) {
      // Adjust serialization format if converter is used. Note: Do not modify the serialization
      // format in the original topics_names_to_info_ map, since it is persist
      // across close()->open() calls.
      metadata_.topics_with_message_count.back().topic_metadata.serialization_format =
        converter_->get_output_serialization_format();
    }
  }
}

void SequentialWriter::write_messages(
  const std::vector<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>> & messages)
{
  if (messages.empty()) {
    return;
  }
  std::vector<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>> merged_messages;
  const auto * messages_to_write = &messages;

  if (storage_options_.snapshot_mode && transient_local_cache_) {
    const auto snapshot_earliest_recv_timestamp = messages.front()->recv_timestamp;
    const auto snapshot_earliest_send_timestamp = messages.front()->send_timestamp;
    auto transient_messages = transient_local_cache_->get_messages_sorted_by_timestamp();
    merged_messages.reserve(messages.size() + transient_messages.size());

    // Only prepend transient messages whose timestamps predate the snapshot window.
    // Messages inside the window are already present in the snapshot buffer.
    // Adjusted transient messages all receive the snapshot-earliest timestamp, so they
    // naturally sort before (or equal to) the first snapshot message — no extra sort needed.
    for (auto & transient_message : transient_messages) {
      if (transient_message->recv_timestamp < snapshot_earliest_recv_timestamp) {
        transient_message->recv_timestamp = snapshot_earliest_recv_timestamp;
        transient_message->send_timestamp = snapshot_earliest_send_timestamp;
        merged_messages.emplace_back(std::move(transient_message));
      }
    }

    std::copy(messages.begin(), messages.end(), std::back_inserter(merged_messages));
    messages_to_write = &merged_messages;
  }

  auto lost_messages_idx = storage_->write_messages(*messages_to_write);
  auto written_messages_count = messages_to_write->size() - lost_messages_idx.size();

  if (storage_options_.snapshot_mode && written_messages_count > 0) {
    // Update FileInformation about the last file in metadata in case of snapshot mode
    size_t first_msg_index = 0;
    // If some messages were lost, we need to find the first message that was written
    if (!lost_messages_idx.empty()) {
      for (size_t i = 0; i < messages_to_write->size(); ++i) {
        auto is_lost = std::binary_search(lost_messages_idx.begin(), lost_messages_idx.end(), i);
        if (!is_lost) {
          first_msg_index = i;
          break;
        }
      }
    }
    size_t last_msg_index = messages_to_write->size() - 1;
    // If some messages were lost, we need to find the last message that was written
    if (!lost_messages_idx.empty()) {
      for (size_t i = messages_to_write->size(); i-- > first_msg_index; ) {
        auto is_lost = std::binary_search(lost_messages_idx.begin(), lost_messages_idx.end(), i);
        if (!is_lost) {
          last_msg_index = i;
          break;
        }
      }
    }

    const auto first_msg_timestamp = std::chrono::time_point<std::chrono::high_resolution_clock>(
      std::chrono::nanoseconds((*messages_to_write)[first_msg_index]->recv_timestamp));
    const auto last_msg_timestamp = std::chrono::time_point<std::chrono::high_resolution_clock>(
      std::chrono::nanoseconds((*messages_to_write)[last_msg_index]->recv_timestamp));
    metadata_.files.back().starting_time = first_msg_timestamp;
    metadata_.files.back().duration = last_msg_timestamp - first_msg_timestamp;
  }

  metadata_.files.back().message_count += written_messages_count;
  metadata_.message_count += written_messages_count;

  {  // Update message count for each topic in metadata
    std::lock_guard<std::mutex> lock(topics_info_mutex_);
    for (size_t i = 0; i < messages_to_write->size(); i++) {
      if (!lost_messages_idx.empty()) {
        auto is_lost = std::binary_search(lost_messages_idx.begin(), lost_messages_idx.end(), i);
        if (is_lost) {
          continue;
        }
      }

      const auto & msg = (*messages_to_write)[i];
      auto topic_info_it = topics_names_to_info_.find(msg->topic_name);
      if (topic_info_it != topics_names_to_info_.end()) {
        topic_info_it->second.message_count++;
        per_file_topic_message_counts_.back()[msg->topic_name]++;
      }
    }
  }

  // Notify about lost messages via callback
  if (!lost_messages_idx.empty()) {
    std::unordered_map<std::string, size_t> lost_messages_count;
    for (const auto & lost_message_index : lost_messages_idx) {
      const auto & topic_name = (*messages_to_write)[lost_message_index]->topic_name;
      lost_messages_count[topic_name]++;
    }
    auto msgs_lost_info = std::make_shared<std::vector<bag_events::MessagesLostInfo>>();
    msgs_lost_info->reserve(lost_messages_count.size());
    for (const auto & [topic_name, count] : lost_messages_count) {
      msgs_lost_info->emplace_back(bag_events::MessagesLostInfo{topic_name, count});
    }
    on_messages_lost(std::move(msgs_lost_info));
  }
}

void SequentialWriter::on_messages_lost(
  std::shared_ptr<std::vector<bag_events::MessagesLostInfo>> msgs_lost_info)
{
  if (!msgs_lost_info->empty()) {
    std::lock_guard<std::mutex> lock(lost_messages_callbacks_mutex_);
    callback_manager_.execute_callbacks(
      bag_events::BagEvent::MESSAGES_LOST,
      std::move(msgs_lost_info));
  }
}

void SequentialWriter::add_event_callbacks(const bag_events::WriterEventCallbacks & callbacks)
{
  if (callbacks.write_split_callback) {
    callback_manager_.add_event_callback(
      callbacks.write_split_callback,
      bag_events::BagEvent::WRITE_SPLIT);
  }

  if (callbacks.messages_lost_callback) {
    callback_manager_.add_event_callback(
      callbacks.messages_lost_callback,
      bag_events::BagEvent::MESSAGES_LOST);
  }

  if (!callbacks.messages_lost_callback && !callbacks.write_split_callback) {
    throw std::runtime_error("No valid callback provided");
  }
}

bool SequentialWriter::has_callback_for_event(bag_events::BagEvent event) const
{
  return callback_manager_.has_callback_for_event(event);
}

}  // namespace writers
}  // namespace rosbag2_cpp
