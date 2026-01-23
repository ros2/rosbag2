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
    auto const & md = topic_names_to_message_definitions_[topic_name];
    storage_->create_topic(topic_info.topic_metadata, md);
    metadata_.topics_with_message_count.push_back(topic_info);
    if (converter_) {
      converter_->add_topic(topic_name, topic_info.topic_metadata.type);
    }
  }
  storage_->update_metadata(metadata_);
  is_open_ = true;
}

void SequentialWriter::close()
{
  // Note. close and open methods protected with mutex on upper rosbag2_cpp::writer level.
  if (!is_open_.exchange(false)) {
    return;  // The writer is not open
  }
  if (use_cache_) {
    // destructor will flush message cache
    cache_consumer_.reset();
    message_cache_.reset();
  }

  if (!base_folder_.empty()) {
    finalize_metadata();
    if (storage_) {
      storage_->update_metadata(metadata_);
    }
    metadata_io_->write_metadata(base_folder_, metadata_);
  }

  if (storage_) {
    storage_.reset();  // Destroy storage before calling WRITE_SPLIT callback to make sure that
    // bag file was closed before callback call.
  }
  if (!metadata_.relative_file_paths.empty()) {
    // Take the latest file name from metadata in case if it was updated after compression in
    // derived class
    auto closed_file =
      (fs::path(base_folder_) / metadata_.relative_file_paths.back()).generic_string();
    execute_bag_split_callbacks(closed_file, "");
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
    storage_->create_topic(topic_with_type, message_definition);
    metadata_.topics_with_message_count.push_back(info);
    if (converter_) {
      converter_->add_topic(topic_with_type.name, topic_with_type.type);
    }
  }
}

void SequentialWriter::remove_topic(const rosbag2_storage::TopicMetadata & topic_with_type)
{
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
  // Right now `base_folder_` is always just the folder name for where to install the bagfile.
  // The name of the folder needs to be queried in case
  // SequentialWriter is opened with a relative path.
  std::stringstream storage_file_name;
  storage_file_name << fs::path(base_folder).filename().generic_string() << "_" <<
    storage_count;

  return (fs::path(base_folder) / storage_file_name.str()).generic_string();
}

void SequentialWriter::switch_to_next_storage()
{
  // consume remaining message cache
  if (use_cache_) {
    cache_consumer_->stop();
    message_cache_->log_dropped();
  }

  storage_->update_metadata(metadata_);
  storage_options_.uri = format_storage_uri(
    base_folder_,
    metadata_.relative_file_paths.size());
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

  bool message_lost = false;
  if (storage_options_.max_cache_size == 0u) {
    // If cache size is set to zero, we write to storage directly
    if (storage_->write_message(converted_msg)) {
      metadata_.files.back().message_count++;
      topic_information_ptr->message_count++;
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
  auto lost_messages_idx = storage_->write_messages(messages);
  auto written_messages_count = messages.size() - lost_messages_idx.size();
  if (storage_options_.snapshot_mode && written_messages_count > 0) {
    // Update FileInformation about the last file in metadata in case of snapshot mode
    size_t first_msg_index = 0;
    // If some messages were lost, we need to find the first message that was written
    if (!lost_messages_idx.empty()) {
      for (size_t i = 0; i < messages.size(); ++i) {
        auto is_lost = std::binary_search(lost_messages_idx.begin(), lost_messages_idx.end(), i);
        if (!is_lost) {
          first_msg_index = i;
          break;
        }
      }
    }
    size_t last_msg_index = messages.size() - 1;
    // If some messages were lost, we need to find the last message that was written
    if (!lost_messages_idx.empty()) {
      for (size_t i = messages.size() - 1; i >= first_msg_index; i--) {
        auto is_lost = std::binary_search(lost_messages_idx.begin(), lost_messages_idx.end(), i);
        if (!is_lost) {
          last_msg_index = i;
          break;
        }
      }
    }

    const auto first_msg_timestamp = std::chrono::time_point<std::chrono::high_resolution_clock>(
      std::chrono::nanoseconds(messages[first_msg_index]->recv_timestamp));
    const auto last_msg_timestamp = std::chrono::time_point<std::chrono::high_resolution_clock>(
      std::chrono::nanoseconds(messages[last_msg_index]->recv_timestamp));
    metadata_.files.back().starting_time = first_msg_timestamp;
    metadata_.files.back().duration = last_msg_timestamp - first_msg_timestamp;
  }
  metadata_.files.back().message_count += written_messages_count;
  metadata_.message_count += written_messages_count;

  {  // Update message count for each topic in metadata
    std::lock_guard<std::mutex> lock(topics_info_mutex_);
    for (size_t i = 0; i < messages.size(); i++) {
      // If some messages were lost, we need to skip them
      if (!lost_messages_idx.empty()) {
        auto is_lost = std::binary_search(lost_messages_idx.begin(), lost_messages_idx.end(), i);
        if (is_lost) {
          continue;  // Skip lost messages
        }
      }
      auto topic_info_it = topics_names_to_info_.find(messages[i]->topic_name);
      if (topic_info_it != topics_names_to_info_.end()) {
        topic_info_it->second.message_count++;
      }
    }
  }

  // Notify about lost messages via callback
  if (!lost_messages_idx.empty()) {
    std::unordered_map<std::string, size_t> lost_messages_count;
    for (const auto & lost_message_index : lost_messages_idx) {
      const auto & topic_name = messages[lost_message_index]->topic_name;
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
