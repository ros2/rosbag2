// Copyright 2022, Amazon.com Inc or its Affiliates. All rights reserved.
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

#include "rcutils/logging_macros.h"
#include "rosbag2_storage/metadata_io.hpp"
#include "rosbag2_storage/ros_helper.hpp"
#include "rosbag2_storage/storage_interfaces/read_write_interface.hpp"
#include "rosbag2_storage_mcap/visibility_control.hpp"

#ifdef ROSBAG2_STORAGE_MCAP_HAS_YAML_HPP
  #include "rosbag2_storage/yaml.hpp"
#else
  // COMPATIBILITY(foxy, galactic) - this block is available in rosbag2_storage/yaml.hpp in H
  #ifdef _WIN32
    // This is necessary because of a bug in yaml-cpp's cmake
    #define YAML_CPP_DLL
    // This is necessary because yaml-cpp does not always use dllimport/dllexport consistently
    #pragma warning(push)
    #pragma warning(disable : 4251)
    #pragma warning(disable : 4275)
  #endif
  #include "yaml-cpp/yaml.h"
  #ifdef _WIN32
    #pragma warning(pop)
  #endif
#endif

#include <mcap/mcap.hpp>

#include <algorithm>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#ifdef ROSBAG2_STORAGE_MCAP_HAS_STORAGE_FILTER_TOPIC_REGEX
  #include <regex>
#endif

// This is necessary because of using stl types here. It is completely safe, because
// a) the member is not accessible from the outside
// b) there are no inline functions.
#ifdef _WIN32
  #pragma warning(push)
  #pragma warning(disable : 4251)
#endif

#define DECLARE_YAML_VALUE_MAP(KEY_TYPE, VALUE_TYPE, ...)                   \
  template <>                                                               \
  struct convert<KEY_TYPE>                                                  \
  {                                                                         \
    static Node encode(const KEY_TYPE & e)                                  \
    {                                                                       \
      static const std::pair<KEY_TYPE, VALUE_TYPE> mapping[] = __VA_ARGS__; \
      for (const auto & m : mapping) {                                      \
        if (m.first == e) {                                                 \
          return Node(m.second);                                            \
        }                                                                   \
      }                                                                     \
      return Node("");                                                      \
    }                                                                       \
                                                                            \
    static bool decode(const Node & node, KEY_TYPE & e)                     \
    {                                                                       \
      static const std::pair<KEY_TYPE, VALUE_TYPE> mapping[] = __VA_ARGS__; \
      const auto val = node.as<VALUE_TYPE>();                               \
      for (const auto & m : mapping) {                                      \
        if (m.second == val) {                                              \
          e = m.first;                                                      \
          return true;                                                      \
        }                                                                   \
      }                                                                     \
      return false;                                                         \
    }                                                                       \
  }

namespace
{
// Simple wrapper with default constructor for use by YAML
struct McapWriterOptions : mcap::McapWriterOptions
{
  McapWriterOptions()
      : mcap::McapWriterOptions("ros2")
  {
  }
};
}  // namespace

namespace YAML
{
#ifndef ROSBAG2_STORAGE_MCAP_HAS_YAML_HPP
template <typename T>
void optional_assign(const Node & node, std::string field, T & assign_to)
{
  if (node[field]) {
    assign_to = node[field].as<T>();
  }
}
#endif

DECLARE_YAML_VALUE_MAP(mcap::Compression, std::string,
                       {{mcap::Compression::None, "None"},
                        {mcap::Compression::Lz4, "Lz4"},
                        {mcap::Compression::Zstd, "Zstd"}});

DECLARE_YAML_VALUE_MAP(mcap::CompressionLevel, std::string,
                       {{mcap::CompressionLevel::Fastest, "Fastest"},
                        {mcap::CompressionLevel::Fast, "Fast"},
                        {mcap::CompressionLevel::Default, "Default"},
                        {mcap::CompressionLevel::Slow, "Slow"},
                        {mcap::CompressionLevel::Slowest, "Slowest"}});

template <>
struct convert<McapWriterOptions>
{
  // NOTE: when updating this struct, also update documentation in README.md
  static bool decode(const Node & node, McapWriterOptions & o)
  {
    optional_assign<bool>(node, "noChunkCRC", o.noChunkCRC);
    optional_assign<bool>(node, "noAttachmentCRC", o.noAttachmentCRC);
    optional_assign<bool>(node, "enableDataCRC", o.enableDataCRC);
    optional_assign<bool>(node, "noSummaryCRC", o.noSummaryCRC);
    optional_assign<bool>(node, "noChunking", o.noChunking);
    optional_assign<bool>(node, "noMessageIndex", o.noMessageIndex);
    optional_assign<bool>(node, "noSummary", o.noSummary);
    optional_assign<uint64_t>(node, "chunkSize", o.chunkSize);
    optional_assign<mcap::Compression>(node, "compression", o.compression);
    optional_assign<mcap::CompressionLevel>(node, "compressionLevel", o.compressionLevel);
    optional_assign<bool>(node, "forceCompression", o.forceCompression);
    // Intentionally omitting "profile" and "library"
    optional_assign<bool>(node, "noRepeatedSchemas", o.noRepeatedSchemas);
    optional_assign<bool>(node, "noRepeatedChannels", o.noRepeatedChannels);
    optional_assign<bool>(node, "noAttachmentIndex", o.noAttachmentIndex);
    optional_assign<bool>(node, "noMetadataIndex", o.noMetadataIndex);
    optional_assign<bool>(node, "noChunkIndex", o.noChunkIndex);
    optional_assign<bool>(node, "noStatistics", o.noStatistics);
    optional_assign<bool>(node, "noSummaryOffsets", o.noSummaryOffsets);
    return true;
  }
};
}  // namespace YAML

namespace rosbag2_storage_plugins
{
using mcap::ByteOffset;
using time_point = std::chrono::time_point<std::chrono::high_resolution_clock>;
static const char FILE_EXTENSION[] = ".mcap";
static const char LOG_NAME[] = "rosbag2_storage_mcap";

static void OnProblem(const mcap::Status & status)
{
  RCUTILS_LOG_ERROR_NAMED(LOG_NAME, "%s", status.message.c_str());
}

/**
 * A storage implementation for the MCAP file format.
 */
class ROSBAG2_STORAGE_MCAP_PUBLIC MCAPStorage
    : public rosbag2_storage::storage_interfaces::ReadWriteInterface
{
public:
  MCAPStorage();
  ~MCAPStorage() override;

  /** BaseIOInterface **/
#ifdef ROSBAG2_STORAGE_MCAP_HAS_STORAGE_OPTIONS
  void open(const rosbag2_storage::StorageOptions & storage_options,
            rosbag2_storage::storage_interfaces::IOFlag io_flag =
              rosbag2_storage::storage_interfaces::IOFlag::READ_WRITE) override;
  void open(const std::string & uri, rosbag2_storage::storage_interfaces::IOFlag io_flag =
                                       rosbag2_storage::storage_interfaces::IOFlag::READ_WRITE);
#else
  void open(const std::string & uri,
            rosbag2_storage::storage_interfaces::IOFlag io_flag =
              rosbag2_storage::storage_interfaces::IOFlag::READ_WRITE) override;
#endif

  /** BaseInfoInterface **/
  rosbag2_storage::BagMetadata get_metadata() override;
  std::string get_relative_file_path() const override;
  uint64_t get_bagfile_size() const override;
  std::string get_storage_identifier() const override;

  /** BaseReadInterface **/
#ifdef ROSBAG2_STORAGE_MCAP_HAS_SET_READ_ORDER
  bool set_read_order(const rosbag2_storage::ReadOrder &) override;
#endif
  bool has_next() override;
  std::shared_ptr<rosbag2_storage::SerializedBagMessage> read_next() override;
  std::vector<rosbag2_storage::TopicMetadata> get_all_topics_and_types() override;

  /** ReadOnlyInterface **/
  void set_filter(const rosbag2_storage::StorageFilter & storage_filter) override;
  void reset_filter() override;
#ifdef ROSBAG2_STORAGE_MCAP_OVERRIDE_SEEK_METHOD
  void seek(const rcutils_time_point_value_t & time_stamp) override;
#else
  void seek(const rcutils_time_point_value_t & timestamp);
#endif

  /** ReadWriteInterface **/
  uint64_t get_minimum_split_file_size() const override;

  /** BaseWriteInterface **/
  void write(std::shared_ptr<const rosbag2_storage::SerializedBagMessage> msg) override;
  void write(
    const std::vector<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>> & msg) override;
  void create_topic(const rosbag2_storage::TopicMetadata & topic,
                    const rosbag2_storage::MessageDefinition & message_definition) override;
  void remove_topic(const rosbag2_storage::TopicMetadata & topic) override;
#ifdef ROSBAG2_STORAGE_MCAP_HAS_UPDATE_METADATA
  void update_metadata(const rosbag2_storage::BagMetadata &) override;
#endif

private:
  void open_impl(const std::string & uri, const std::string & preset_profile,
                 rosbag2_storage::storage_interfaces::IOFlag io_flag,
                 const std::string & storage_config_uri);

  void reset_iterator();
  bool read_and_enqueue_message();
  bool enqueued_message_is_already_read();
  bool message_indexes_present();
  void ensure_summary_read();

  std::optional<rosbag2_storage::storage_interfaces::IOFlag> opened_as_;
  std::string relative_path_;

  std::shared_ptr<rosbag2_storage::SerializedBagMessage> next_;

  rosbag2_storage::BagMetadata metadata_{};
  std::unordered_map<std::string, rosbag2_storage::TopicInformation> topics_;
  std::unordered_map<std::string, mcap::SchemaId> schema_ids_;    // datatype -> schema_id
  std::unordered_map<std::string, mcap::ChannelId> channel_ids_;  // topic -> channel_id
  rosbag2_storage::StorageFilter storage_filter_{};
  mcap::ReadMessageOptions::ReadOrder read_order_ = mcap::ReadMessageOptions::ReadOrder::FileOrder;

  std::unique_ptr<std::ifstream> input_;
  std::unique_ptr<mcap::FileStreamReader> data_source_;
  std::unique_ptr<mcap::McapReader> mcap_reader_;
  std::unique_ptr<mcap::LinearMessageView> linear_view_;
  std::unique_ptr<mcap::LinearMessageView::Iterator> linear_iterator_;

  std::unique_ptr<mcap::McapWriter> mcap_writer_;

  bool has_read_summary_ = false;
  rcutils_time_point_value_t last_read_time_point_ = 0;
  std::optional<mcap::RecordOffset> last_read_message_offset_;
  std::optional<mcap::RecordOffset> last_enqueued_message_offset_;
};

MCAPStorage::MCAPStorage()
{
  metadata_.storage_identifier = get_storage_identifier();
  metadata_.message_count = 0;
}

MCAPStorage::~MCAPStorage()
{
  if (mcap_reader_) {
    mcap_reader_->close();
  }
  if (input_) {
    input_->close();
  }
  if (mcap_writer_) {
    mcap_writer_->close();
  }
}

/** BaseIOInterface **/
#ifdef ROSBAG2_STORAGE_MCAP_HAS_STORAGE_OPTIONS
void MCAPStorage::open(const rosbag2_storage::StorageOptions & storage_options,
                       rosbag2_storage::storage_interfaces::IOFlag io_flag)
{
  open_impl(storage_options.uri, storage_options.storage_preset_profile, io_flag,
            storage_options.storage_config_uri);
}
#endif

void MCAPStorage::open(const std::string & uri, rosbag2_storage::storage_interfaces::IOFlag io_flag)
{
  open_impl(uri, "", io_flag, "");
}

static void SetOptionsForPreset(const std::string & preset_profile, McapWriterOptions & options)
{
  if (preset_profile == "fastwrite") {
    options.noChunking = true;
    options.noSummaryCRC = true;
  } else if (preset_profile == "zstd_fast") {
    options.compression = mcap::Compression::Zstd;
    options.compressionLevel = mcap::CompressionLevel::Fastest;
    options.noChunkCRC = true;
  } else if (preset_profile == "zstd_small") {
    options.compression = mcap::Compression::Zstd;
    options.compressionLevel = mcap::CompressionLevel::Slowest;
    options.chunkSize = 4 * 1024 * 1024;
  } else if (preset_profile != "none") {
    throw std::runtime_error(
      "unknown MCAP storage preset profile "
      "(valid options are 'none', 'fastwrite', 'zstd_fast', 'zstd_small'): " +
      preset_profile);
  }
}

void MCAPStorage::open_impl(const std::string & uri, const std::string & preset_profile,
                            rosbag2_storage::storage_interfaces::IOFlag io_flag,
                            const std::string & storage_config_uri)
{
  switch (io_flag) {
    case rosbag2_storage::storage_interfaces::IOFlag::READ_ONLY: {
      relative_path_ = uri;
      input_ = std::make_unique<std::ifstream>(relative_path_, std::ios::binary);
      data_source_ = std::make_unique<mcap::FileStreamReader>(*input_);
      mcap_reader_ = std::make_unique<mcap::McapReader>();
      auto status = mcap_reader_->open(*data_source_);
      if (!status.ok()) {
        throw std::runtime_error(status.message);
      }
      last_read_time_point_ = 0;
      reset_iterator();
      break;
    }
    case rosbag2_storage::storage_interfaces::IOFlag::READ_WRITE:
    case rosbag2_storage::storage_interfaces::IOFlag::APPEND: {
      // APPEND does not seem to be used; treat it the same as READ_WRITE
      io_flag = rosbag2_storage::storage_interfaces::IOFlag::READ_WRITE;
      relative_path_ = uri + FILE_EXTENSION;

      mcap_writer_ = std::make_unique<mcap::McapWriter>();
      McapWriterOptions options;
      // Set defaults for the rosbag2 storage plugin specifically.
      options.noChunkCRC = true;
      options.compression = mcap::Compression::None;
      // Set options from preset profile first
      if (!preset_profile.empty()) {
        SetOptionsForPreset(preset_profile, options);
      }
      // If both preset profile and storage config are specified,
      // options from the storage config are overlaid on the options from the preset profile.
      if (!storage_config_uri.empty()) {
        YAML::Node yaml_node = YAML::LoadFile(storage_config_uri);
        YAML::convert<McapWriterOptions>::decode(yaml_node, options);
      }

      auto status = mcap_writer_->open(relative_path_, options);
      if (!status.ok()) {
        throw std::runtime_error(status.message);
      }
      break;
    }
  }
  opened_as_ = io_flag;
  metadata_.relative_file_paths = {get_relative_file_path()};
}

/** BaseInfoInterface **/
rosbag2_storage::BagMetadata MCAPStorage::get_metadata()
{
  ensure_summary_read();

  metadata_.version = 2;
  metadata_.storage_identifier = get_storage_identifier();
  metadata_.bag_size = get_bagfile_size();
  metadata_.relative_file_paths = {get_relative_file_path()};

  // Fill out summary metadata from the Statistics record
  const mcap::Statistics & stats = mcap_reader_->statistics().value();
  metadata_.message_count = stats.messageCount;
  metadata_.duration = std::chrono::nanoseconds(stats.messageEndTime - stats.messageStartTime);
  metadata_.starting_time = time_point(std::chrono::nanoseconds(stats.messageStartTime));

  // Build a list of topic information along with per-topic message counts
  metadata_.topics_with_message_count.clear();
  for (const auto & [channel_id, channel_ptr] : mcap_reader_->channels()) {
    const mcap::Channel & channel = *channel_ptr;

    // Look up the Schema for this topic
    const auto schema_ptr = mcap_reader_->schema(channel.schemaId);
    if (!schema_ptr) {
      throw std::runtime_error("Could not find schema for topic " + channel.topic);
    }

    // Create a TopicInformation for this topic
    rosbag2_storage::TopicInformation topic_info{};
    topic_info.topic_metadata.name = channel.topic;
    topic_info.topic_metadata.serialization_format = channel.messageEncoding;
    topic_info.topic_metadata.type = schema_ptr->name;

    // Look up the offered_qos_profiles metadata entry
    const auto metadata_it = channel.metadata.find("offered_qos_profiles");
    if (metadata_it != channel.metadata.end()) {
      topic_info.topic_metadata.offered_qos_profiles = metadata_it->second;
    }
    const auto type_hash_it = channel.metadata.find("topic_type_hash");
    if (type_hash_it != channel.metadata.end()) {
      topic_info.topic_metadata.type_description_hash = type_hash_it->second;
    }

    // Look up the message count for this Channel
    const auto message_count_it = stats.channelMessageCounts.find(channel_id);
    if (message_count_it != stats.channelMessageCounts.end()) {
      topic_info.message_count = message_count_it->second;
    } else {
      topic_info.message_count = 0;
    }

    metadata_.topics_with_message_count.push_back(topic_info);
  }

  return metadata_;
}

std::string MCAPStorage::get_relative_file_path() const
{
  return relative_path_;
}

uint64_t MCAPStorage::get_bagfile_size() const
{
  if (opened_as_ == rosbag2_storage::storage_interfaces::IOFlag::READ_ONLY) {
    return data_source_ ? data_source_->size() : 0;
  } else {
    if (!mcap_writer_) {
      return 0;
    }
    const auto * data_sink = mcap_writer_->dataSink();
    return data_sink ? data_sink->size() : 0;
  }
}

std::string MCAPStorage::get_storage_identifier() const
{
  return "mcap";
}

/** BaseReadInterface **/
bool MCAPStorage::read_and_enqueue_message()
{
  // The recording has not been opened.
  if (!linear_iterator_) {
    return false;
  }

  auto & it = *linear_iterator_;

  // At the end of the recording
  if (it == linear_view_->end()) {
    next_ = nullptr;
    return false;
  }

  const auto & messageView = *it;
  auto msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  last_enqueued_message_offset_ = messageView.messageOffset;
  msg->time_stamp = rcutils_time_point_value_t(messageView.message.logTime);
  msg->topic_name = messageView.channel->topic;
  msg->serialized_data = rosbag2_storage::make_serialized_message(messageView.message.data,
                                                                  messageView.message.dataSize);

  // enqueue this message to be used
  next_ = msg;

  ++it;
  return true;
}

void MCAPStorage::reset_iterator()
{
  ensure_summary_read();
  mcap::ReadMessageOptions options;
  if (read_order_ == mcap::ReadMessageOptions::ReadOrder::ReverseLogTimeOrder) {
    options.startTime = 0;
    // endTime is an exclusive range endpoint, but we want to start from `last_read_time_point_`.
    // therefore, the time range we pass to the MCAP library is one nanosecond later than the
    // seek point specified.
    options.endTime = mcap::Timestamp(last_read_time_point_ + 1);
  } else {
    options.startTime = mcap::Timestamp(last_read_time_point_);
    options.endTime = mcap::MaxTime;
  }
  options.readOrder = read_order_;
  if (!storage_filter_.topics.empty()) {
    options.topicFilter = [this](std::string_view topic) {
      for (const auto & match_topic : storage_filter_.topics) {
        if (match_topic == topic) {
          return true;
        }
      }
      return false;
    };
  }
#ifdef ROSBAG2_STORAGE_MCAP_HAS_STORAGE_FILTER_TOPIC_REGEX
  if (!storage_filter_.topics_regex.empty()) {
    options.topicFilter = [this](std::string_view topic) {
      std::smatch m;
      std::string topic_string(topic);
      std::regex re(storage_filter_.topics_regex);
      return std::regex_match(topic_string, m, re);
    };
  }
#endif
  linear_view_ =
    std::make_unique<mcap::LinearMessageView>(mcap_reader_->readMessages(OnProblem, options));
  linear_iterator_ = std::make_unique<mcap::LinearMessageView::Iterator>(linear_view_->begin());
  if (!read_and_enqueue_message()) {
    return;
  }
  while (enqueued_message_is_already_read()) {
    if (!read_and_enqueue_message()) {
      return;
    }
  }
}

bool MCAPStorage::enqueued_message_is_already_read()
{
  if (last_read_message_offset_ == std::nullopt) {
    return false;
  }
  if (last_enqueued_message_offset_ == std::nullopt) {
    return false;
  }
  if (next_ == nullptr) {
    return false;
  }
  if (last_read_time_point_ != next_->time_stamp) {
    return false;
  }
  if (read_order_ == mcap::ReadMessageOptions::ReadOrder::ReverseLogTimeOrder) {
    return (*last_enqueued_message_offset_ >= *last_read_message_offset_);
  }
  return (*last_enqueued_message_offset_ <= *last_read_message_offset_);
}

void MCAPStorage::ensure_summary_read()
{
  if (!has_read_summary_) {
    const auto status = mcap_reader_->readSummary(mcap::ReadSummaryMethod::AllowFallbackScan);

    if (!status.ok()) {
      throw std::runtime_error(status.message);
    }
    has_read_summary_ = true;
  }
}

bool MCAPStorage::message_indexes_present()
{
  ensure_summary_read();
  for (const auto & ci : mcap_reader_->chunkIndexes()) {
    if (ci.messageIndexLength > 0) {
      return true;
    }
  }
  return false;
}

#ifdef ROSBAG2_STORAGE_MCAP_HAS_SET_READ_ORDER
bool MCAPStorage::set_read_order(const rosbag2_storage::ReadOrder & read_order)
{
  if (!has_read_summary_) {
    throw std::runtime_error("set_read_order called before open()");
  }
  switch (read_order.sort_by) {
    case rosbag2_storage::ReadOrder::ReceivedTimestamp:
      if (!message_indexes_present()) {
        RCUTILS_LOG_WARN_NAMED(
          LOG_NAME, "attempted to read in receive timestamp order with no message index");
        return false;
      }
      if (read_order.reverse) {
        read_order_ = mcap::ReadMessageOptions::ReadOrder::ReverseLogTimeOrder;
      } else {
        read_order_ = mcap::ReadMessageOptions::ReadOrder::LogTimeOrder;
      }
      break;
    case rosbag2_storage::ReadOrder::File:
      if (!read_order.reverse) {
        read_order_ = mcap::ReadMessageOptions::ReadOrder::FileOrder;
      } else {
        RCUTILS_LOG_WARN_NAMED(LOG_NAME, "reverse file-order reading not implemented");
        return false;
      }
      break;
    case rosbag2_storage::ReadOrder::PublishedTimestamp:
      RCUTILS_LOG_WARN_NAMED(LOG_NAME, "publish timestamp order reading not implemented");
      return false;
      break;
  }
  reset_iterator();
  return true;
}
#endif

bool MCAPStorage::has_next()
{
  if (!linear_iterator_) {
    return false;
  }
  // Have already verified next message and enqueued it for use.
  if (next_) {
    return true;
  }

  return read_and_enqueue_message();
}

std::shared_ptr<rosbag2_storage::SerializedBagMessage> MCAPStorage::read_next()
{
  if (!has_next()) {
    throw std::runtime_error{"No next message is available."};
  }
  last_read_time_point_ = next_->time_stamp;
  last_read_message_offset_ = last_enqueued_message_offset_;
  // Importantly, clear next_ via move so that a next message can be read.
  return std::move(next_);
}

std::vector<rosbag2_storage::TopicMetadata> MCAPStorage::get_all_topics_and_types()
{
  auto metadata = get_metadata();
  std::vector<rosbag2_storage::TopicMetadata> out;
  for (const auto & topic : metadata.topics_with_message_count) {
    out.push_back(topic.topic_metadata);
  }
  return out;
}

/** ReadOnlyInterface **/
void MCAPStorage::set_filter(const rosbag2_storage::StorageFilter & storage_filter)
{
  storage_filter_ = storage_filter;
  reset_iterator();
}

void MCAPStorage::reset_filter()
{
  set_filter(rosbag2_storage::StorageFilter());
}

void MCAPStorage::seek(const rcutils_time_point_value_t & time_stamp)
{
  if (time_stamp != last_read_time_point_) {
    last_read_message_offset_ = std::nullopt;
  }
  last_read_time_point_ = time_stamp;
  reset_iterator();
}

/** ReadWriteInterface **/
uint64_t MCAPStorage::get_minimum_split_file_size() const
{
  return 1024;
}

/** BaseWriteInterface **/
void MCAPStorage::write(std::shared_ptr<const rosbag2_storage::SerializedBagMessage> msg)
{
  const auto topic_it = topics_.find(msg->topic_name);
  if (topic_it == topics_.end()) {
    throw std::runtime_error{"Unknown message topic \"" + msg->topic_name + "\""};
  }

  // Get Channel reference
  const auto channel_it = channel_ids_.find(msg->topic_name);
  if (channel_it == channel_ids_.end()) {
    // This should never happen since we adding channel on topic creation
    throw std::runtime_error{"Channel reference not found for topic: \"" + msg->topic_name + "\""};
  }

  mcap::Message mcap_msg;
  mcap_msg.channelId = channel_it->second;
  mcap_msg.sequence = 0;
  if (msg->time_stamp < 0) {
    RCUTILS_LOG_WARN_NAMED(LOG_NAME, "Invalid message timestamp %ld", msg->time_stamp);
  }
  mcap_msg.logTime = mcap::Timestamp(msg->time_stamp);
  mcap_msg.publishTime = mcap_msg.logTime;
  mcap_msg.dataSize = msg->serialized_data->buffer_length;
  mcap_msg.data = reinterpret_cast<const std::byte *>(msg->serialized_data->buffer);
  const auto status = mcap_writer_->write(mcap_msg);
  if (!status.ok()) {
    throw std::runtime_error{std::string{"Failed to write "} +
                             std::to_string(msg->serialized_data->buffer_length) +
                             " byte message to MCAP file: " + status.message};
  }

  /// Update metadata
  // Increment individual topic message count
  topic_it->second.message_count++;
  // Increment global message count
  metadata_.message_count++;
  // Determine recording duration
  const auto message_time = time_point(std::chrono::nanoseconds(msg->time_stamp));
  metadata_.duration = std::max(metadata_.duration, message_time - metadata_.starting_time);
}

void MCAPStorage::write(
  const std::vector<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>> & msgs)
{
  for (const auto & msg : msgs) {
    write(msg);
  }
}

void MCAPStorage::create_topic(const rosbag2_storage::TopicMetadata & topic,
                               const rosbag2_storage::MessageDefinition & message_definition)
{
  auto topic_info = rosbag2_storage::TopicInformation{topic, 0};
  const auto topic_it = topics_.find(topic.name);
  if (topic_it == topics_.end()) {
    topics_.emplace(topic.name, topic_info);
  } else {
    RCUTILS_LOG_WARN_NAMED(LOG_NAME, "Topic with name: %s already exist!", topic.name.c_str());
    return;
  }

  // Create Schema for topic if it doesn't exist yet
  const auto & datatype = topic_info.topic_metadata.type;
  const auto schema_it = schema_ids_.find(datatype);
  mcap::SchemaId schema_id;
  if (schema_it == schema_ids_.end()) {
    mcap::Schema schema;
    schema.name = datatype;
    schema.encoding = message_definition.encoding;
    const auto & full_text = message_definition.encoded_message_definition;
    schema.data.assign(reinterpret_cast<const std::byte *>(full_text.data()),
                       reinterpret_cast<const std::byte *>(full_text.data() + full_text.size()));
    mcap_writer_->addSchema(schema);
    schema_ids_.emplace(datatype, schema.id);
    schema_id = schema.id;
  } else {
    schema_id = schema_it->second;
  }

  // Create Channel for topic if it doesn't exist yet
  const auto channel_it = channel_ids_.find(topic.name);
  if (channel_it == channel_ids_.end()) {
    mcap::Channel channel;
    channel.topic = topic.name;
    channel.messageEncoding = topic_info.topic_metadata.serialization_format;
    channel.schemaId = schema_id;
    channel.metadata.emplace("offered_qos_profiles",
                             topic_info.topic_metadata.offered_qos_profiles);
    channel.metadata.emplace("topic_type_hash", topic_info.topic_metadata.type_description_hash);
    mcap_writer_->addChannel(channel);
    channel_ids_.emplace(topic.name, channel.id);
  }
}

void MCAPStorage::remove_topic(const rosbag2_storage::TopicMetadata & topic)
{
  const auto topic_it = topics_.find(topic.name);
  if (topic_it != topics_.end()) {
    const auto & datatype = topic_it->second.topic_metadata.type;
    schema_ids_.erase(datatype);
    topics_.erase(topic.name);
  }
}

#ifdef ROSBAG2_STORAGE_MCAP_HAS_UPDATE_METADATA
void MCAPStorage::update_metadata(const rosbag2_storage::BagMetadata & bag_metadata)
{
  if (bag_metadata.compression_mode == "message") {
    throw std::runtime_error(
      "MCAP storage plugin does not support message compression, "
      "consider using chunk compression by setting `compression: 'Zstd'` in storage config");
  }
}
#endif

}  // namespace rosbag2_storage_plugins

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(rosbag2_storage_plugins::MCAPStorage,
                       rosbag2_storage::storage_interfaces::ReadWriteInterface)

#ifdef _WIN32
  #pragma warning(pop)
#endif
