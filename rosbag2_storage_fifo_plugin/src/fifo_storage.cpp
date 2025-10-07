// Copyright 2024 Virginia Tech Transportation Institute - Jean Paul Talledo Vilela
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

#include "rosbag2_storage_fifo_plugin/fifo_storage.hpp"

#include <algorithm>
#include <cstring>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

namespace rosbag2_storage_fifo_plugin
{

// --- CONFIGURATION ---
// Define the hardcoded base path for the FIFOs here for easy modification.
const char * kFifoBasePath = "/Data/fifo";
// --- END CONFIGURATION ---

// FdWriter implementation from .hpp moved here for brevity in .hpp
void FdWriter::handleWrite(const std::byte * data, uint64_t size)
{
  if (size == 0) {
    return;
  }
  ssize_t bytes_written = ::write(fd_, data, size);
  if (bytes_written < 0) {
    throw std::runtime_error(std::string("Failed to write to FIFO: ") + strerror(errno));
  }
  if (static_cast<uint64_t>(bytes_written) != size) {
    throw std::runtime_error("Partial write to FIFO, this should not happen.");
  }
  total_bytes_written_ += bytes_written;
}

uint64_t FdWriter::size() const
{
  return total_bytes_written_;
}

FifoStorage::FifoStorage()
: logger_(rclcpp::get_logger("rosbag2_storage_fifo_plugin")),
  mcap_writer_(nullptr)
{
  metadata_.storage_identifier = storage_identifier_;
  definition_cache_ = std::make_unique<rosbag2_storage_mcap::internal::MessageDefinitionCache>();
}

FifoStorage::~FifoStorage()
{
  if (mcap_writer_) {
    write_metadata_to_yaml_fifo();
    mcap_writer_->close();
  }
  if (mcap_fifo_fd_ != -1) {
    ::close(mcap_fifo_fd_);
  }
}

void FifoStorage::create_fifo(const std::string & path)
{
  struct stat st;
  if (stat(path.c_str(), &st) == 0) {
    if (!S_ISFIFO(st.st_mode)) {
      throw std::runtime_error("File exists at FIFO path but is not a FIFO: " + path);
    }
    return;
  }

  if (mkfifo(path.c_str(), 0666) == -1) {
    if (errno != EEXIST) {
      throw std::runtime_error(
              "Failed to create FIFO: " + std::string(strerror(errno)) + " at " + path);
    }
  }
}

void FifoStorage::open(
  const rosbag2_storage::StorageOptions & storage_options,
  rosbag2_storage::storage_interfaces::IOFlag io_flag)
{
  (void)storage_options;  // Explicitly ignore the path from the -o argument

  if (io_flag == rosbag2_storage::storage_interfaces::IOFlag::READ_ONLY) {
    throw std::runtime_error("FifoStorage is a write-only storage implementation.");
  }

  // Create the hardcoded base directory
  std::filesystem::create_directories(kFifoBasePath);

  // Construct the FIFO paths using the hardcoded base path
  mcap_fifo_path_ = std::string(kFifoBasePath) + "/ros2_mcap.fifo";
  yaml_fifo_path_ = std::string(kFifoBasePath) + "/ros2_yaml.fifo";

  create_fifo(mcap_fifo_path_);
  create_fifo(yaml_fifo_path_);

  RCLCPP_INFO(logger_, "MCAP FIFO is at: %s", mcap_fifo_path_.c_str());
  RCLCPP_INFO(logger_, "Waiting for a reader to connect...");

  mcap_fifo_fd_ = ::open(mcap_fifo_path_.c_str(), O_WRONLY);
  if (mcap_fifo_fd_ == -1) {
    throw std::runtime_error(
            "Failed to open MCAP FIFO for writing: " + std::string(strerror(errno)));
  }
  RCLCPP_INFO(logger_, "Reader connected to MCAP FIFO.");

  mcap_fd_writer_ = std::make_unique<FdWriter>(mcap_fifo_fd_);
  mcap::McapWriterOptions options("ros2");
  mcap_writer_ = std::make_unique<mcap::McapWriter>();
  mcap_writer_->open(*mcap_fd_writer_, options);
}

void FifoStorage::write(std::shared_ptr<const rosbag2_storage::SerializedBagMessage> message)
{
  if (!mcap_writer_) {
    throw std::runtime_error("MCAP writer is not open. Call open() first.");
  }

  if (starting_time_ == 0) {
    starting_time_ = message->time_stamp;
  }
  last_timestamp_ = message->time_stamp;
  message_counts_[message->topic_name]++;

  mcap::Message msg;
  msg.channelId = topic_to_channel_id_[message->topic_name];
  msg.sequence = 0;
  msg.logTime = mcap::Timestamp(message->time_stamp);
  msg.publishTime = msg.logTime;
  msg.data = reinterpret_cast<const std::byte *>(message->serialized_data->buffer);
  msg.dataSize = message->serialized_data->buffer_length;

  auto status = mcap_writer_->write(msg);
  if (!status.ok()) {
    throw std::runtime_error("Failed to write message to MCAP FIFO: " + status.message);
  }
}

void FifoStorage::write(
  const std::vector<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>> & messages)
{
  for (const auto & message : messages) {
    write(message);
  }
}

void FifoStorage::create_topic(const rosbag2_storage::TopicMetadata & topic)
{
  if (topic_to_channel_id_.count(topic.name)) {
    return;
  }

  if (!mcap_writer_) {
    throw std::runtime_error("MCAP writer is not open. Call open() first.");
  }

  topics_.push_back(topic);
  message_counts_[topic.name] = 0;

  const auto definition_pair = definition_cache_->get_full_text(topic.type);
  const std::string & schema_text = definition_pair.second;

  mcap::Schema schema(topic.type, "ros2msg", schema_text);
  mcap_writer_->addSchema(schema);

  mcap::Channel channel(topic.name, "cdr", schema.id);
  mcap_writer_->addChannel(channel);
  topic_to_channel_id_[topic.name] = channel.id;
}

void FifoStorage::remove_topic(const rosbag2_storage::TopicMetadata & topic)
{
  (void)topic;
}

std::vector<rosbag2_storage::TopicMetadata> FifoStorage::get_all_topics_and_types()
{
  return topics_;
}

std::string FifoStorage::get_storage_identifier() const
{
  return storage_identifier_;
}

std::string FifoStorage::get_relative_file_path() const
{
  return mcap_fifo_path_;
}

uint64_t FifoStorage::get_bagfile_size() const
{
  if (mcap_fd_writer_) {
    return mcap_fd_writer_->size();
  }
  return 0;
}

rosbag2_storage::BagMetadata FifoStorage::get_metadata()
{
  metadata_.relative_file_paths = {get_relative_file_path()};
  metadata_.topics_with_message_count.clear();
  uint64_t total_messages = 0;

  for (const auto & topic : topics_) {
    rosbag2_storage::TopicInformation info{};
    info.topic_metadata = topic;
    info.message_count = message_counts_[topic.name];
    metadata_.topics_with_message_count.push_back(info);
    total_messages += info.message_count;
  }

  metadata_.message_count = total_messages;
  metadata_.starting_time = std::chrono::time_point<std::chrono::high_resolution_clock>(
    std::chrono::nanoseconds(starting_time_));
  if (last_timestamp_ > starting_time_) {
    metadata_.duration = std::chrono::nanoseconds(last_timestamp_ - starting_time_);
  } else {
    metadata_.duration = std::chrono::nanoseconds(0);
  }

  return metadata_;
}

void FifoStorage::set_filter(const rosbag2_storage::StorageFilter & storage_filter)
{
  (void)storage_filter;
}

void FifoStorage::reset_filter()
{
}

uint64_t FifoStorage::get_minimum_split_file_size() const
{
  return 0;
}

void FifoStorage::write_metadata_to_yaml_fifo()
{
  auto current_metadata = get_metadata();
  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "rosbag2_bagfile_information";
  out << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "version" << YAML::Value << 5;
  out << YAML::Key << "storage_identifier" << YAML::Value << current_metadata.storage_identifier;
  out << YAML::Key << "relative_file_paths" << YAML::Value << YAML::Flow <<
    current_metadata.relative_file_paths;
  out << YAML::Key << "duration" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "nanoseconds" << YAML::Value << current_metadata.duration.count();
  out << YAML::EndMap;
  out << YAML::Key << "starting_time" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "nanoseconds_since_epoch" << YAML::Value <<
    std::chrono::duration_cast<std::chrono::nanoseconds>(
    current_metadata.starting_time.time_since_epoch()).count();
  out << YAML::EndMap;
  out << YAML::Key << "message_count" << YAML::Value << current_metadata.message_count;
  out << YAML::Key << "topics_with_message_count" << YAML::Value << YAML::BeginSeq;
  for (const auto & topic_info : current_metadata.topics_with_message_count) {
    out << YAML::BeginMap;
    out << YAML::Key << "topic_metadata" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "name" << YAML::Value << topic_info.topic_metadata.name;
    out << YAML::Key << "type" << YAML::Value << topic_info.topic_metadata.type;
    out << YAML::Key << "serialization_format" << YAML::Value <<
      topic_info.topic_metadata.serialization_format;
    out << YAML::Key << "offered_qos_profiles" << YAML::Value <<
      topic_info.topic_metadata.offered_qos_profiles;
    out << YAML::EndMap;
    out << YAML::Key << "message_count" << YAML::Value << topic_info.message_count;
    out << YAML::EndMap;
  }
  out << YAML::EndSeq;
  out << YAML::EndMap;
  out << YAML::EndMap;

  int yaml_fd = ::open(yaml_fifo_path_.c_str(), O_WRONLY | O_NONBLOCK);
  if (yaml_fd != -1) {
    RCLCPP_INFO(logger_, "Writing metadata to YAML FIFO.");
    ssize_t written = ::write(yaml_fd, out.c_str(), out.size());
    if (written < 0) {
      RCLCPP_WARN(logger_, "Could not write metadata to YAML FIFO: %s", strerror(errno));
    }
    ::close(yaml_fd);
  } else {
    RCLCPP_WARN(
      logger_,
      "Could not open YAML FIFO to write metadata. A reader may not be present.");
  }
}

}  // namespace rosbag2_storage_fifo_plugin

PLUGINLIB_EXPORT_CLASS(
  rosbag2_storage_fifo_plugin::FifoStorage,
  rosbag2_storage::storage_interfaces::ReadWriteInterface)
