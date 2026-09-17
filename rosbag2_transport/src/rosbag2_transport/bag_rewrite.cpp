// Copyright 2021 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

#include "rosbag2_transport/bag_rewrite.hpp"

#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_transport/reader_writer_factory.hpp"
#include "rosbag2_transport/topic_filter.hpp"

namespace
{

/// \brief Type for a pair of rosbag2_cpp::Reader and rosbag2_storage::StorageOptions.
using reader_storage_options_pair_t = \
  std::pair<std::unique_ptr<rosbag2_cpp::Reader>, const rosbag2_storage::StorageOptions>;

/// \brief Type for a pair of rosbag2_cpp::Writer and rosbag2_transport::RecordOptions.
using writer_record_options_pair_t = \
  std::pair<std::unique_ptr<rosbag2_cpp::Writer>, rosbag2_transport::RecordOptions>;

/// Find the next chronological message from all opened input bags.
/// Updates the next_messages queue as necessary.
/// next_messages is needed because Reader has no "peek" interface, we cannot put a message back.
/// Returns nullptr when all input bags have been fully read.
std::shared_ptr<rosbag2_storage::SerializedBagMessage> get_next(
  std::vector<reader_storage_options_pair_t> & input_bags,
  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> & next_messages)
{
  // Find message with the lowest timestamp
  std::shared_ptr<rosbag2_storage::SerializedBagMessage> earliest_msg = nullptr;
  size_t earliest_msg_index = -1;
  for (size_t i = 0; i < next_messages.size(); i++) {
    auto & [reader, storage_options] = input_bags[i];
    auto & next_msg = next_messages[i];
    // refill queue if bag not empty
    if (next_msg == nullptr && reader && reader->has_next()) {
      next_msg = reader->read_next();
      // If we just read a message at or past the end time, close and release reader
      if (storage_options.end_time_ns > 0 && next_msg != nullptr &&
        // Use strict greater than to correctly handle a case when multiple messages have the same
        // timestamp equal to end_time_ns. That is possible with the sim time.
        next_msg->recv_timestamp > storage_options.end_time_ns)
      {
        // Use reset to close and release the reader, since has_next() from the closed reader
        // throwing exception.
        reader.reset();
        next_msg = nullptr;  // Clear out the just-read message that is past the end time
      }
    }

    if (next_msg == nullptr) {continue;}

    if (earliest_msg == nullptr || next_msg->recv_timestamp < earliest_msg->recv_timestamp) {
      earliest_msg = next_msg;
      earliest_msg_index = i;
    }
  }

  // clear returned message from queue before returning it, so it can be refilled next time
  if (earliest_msg != nullptr) {
    next_messages[earliest_msg_index].reset();
  }
  return earliest_msg;
}

/// Return if the message at message_index on topic_name shall be written to an output bag file
bool message_in_range(
  const rosbag2_transport::RecordOptions & record_options,
  const std::string & topic_name, size_t message_index)
{
  auto range_it = record_options.topic_message_ranges.find(topic_name);
  if (range_it == record_options.topic_message_ranges.end()) {
    // Topic is not specified in the message ranges member -> message can be written
    return true;
  }
  // Do not write if the index is outside the range
  return message_index >= range_it->second.first && message_index <= range_it->second.second;
}

/// Validate the per-topic message ranges requested by the output bags
void validate_message_ranges(
  const std::vector<reader_storage_options_pair_t> & input_bags,
  const std::vector<writer_record_options_pair_t> & output_bags)
{
  // Get all topic names and their message counts from the bag input data
  std::unordered_map<std::string, size_t> available_message_counts;
  for (const auto & input_bag : input_bags) {
    const auto & metadata = input_bag.first->get_metadata();
    for (const auto & topic_information : metadata.topics_with_message_count) {
      available_message_counts[topic_information.topic_metadata.name] +=
        topic_information.message_count;
    }
  }

  // Main validation
  for (const auto & output_bag : output_bags) {
    const auto & record_options = output_bag.second;
    for (const auto & [topic_name, range] : record_options.topic_message_ranges) {
      auto count_it = available_message_counts.find(topic_name);
      if (count_it == available_message_counts.end()) {
        throw std::invalid_argument(
          "Invalid message range for topic '" + topic_name +
          "': topic does not exist in any input bag.");
      }
      const size_t message_count = count_it->second;
      if (message_count == 0) {
        throw std::invalid_argument(
          "Invalid message range for topic '" + topic_name +
          "': topic has no messages in the input bags.");
      }

      const size_t start = range.first;
      const size_t end = range.second;
      if (start > end) {
        throw std::invalid_argument(
          "Invalid message range for topic '" + topic_name + "': start index " +
          std::to_string(start) + " is greater than end index " +
          std::to_string(end) + ".");
      }

      if (start >= message_count) {
        throw std::invalid_argument(
          "Invalid message range for topic '" + topic_name + "': start index " +
          std::to_string(start) + " is out of range [0, " +
          std::to_string(message_count - 1) + "]." +
          " The topic has " + std::to_string(message_count) + " messages.");
      }
      if (end >= message_count) {
        throw std::invalid_argument(
          "Invalid message range for topic '" + topic_name + "': end index " +
          std::to_string(end) + " is out of range [0, " +
          std::to_string(message_count - 1) + "]." +
          " The topic has " + std::to_string(message_count) + " messages.");
      }
    }
  }
}

/// \brief An output Writer together with the RecordOptions that configured it.
/// Required to evaluate per-topic message ranges when writing.
struct FilteredOutput
{
  rosbag2_cpp::Writer * writer;
  const rosbag2_transport::RecordOptions * record_options;
};


/// Discover what topics are in the inputs, filter out topics that can't be processed,
/// create_topic on Writers that will receive topics.
/// Return a map f topic -> vector of which Writers want to receive that topic,
/// based on the RecordOptions.
/// The output vector has bare pointers to the uniquely owned Writers,
/// so this may not outlive the output_bags Writers.
std::unordered_map<std::string, std::vector<FilteredOutput>>
setup_topic_filtering(
  const std::vector<reader_storage_options_pair_t> & input_bags,
  const std::vector<writer_record_options_pair_t> & output_bags)
{
  std::unordered_map<std::string, std::vector<FilteredOutput>> filtered_outputs;
  std::map<std::string, std::vector<std::string>> input_topics;
  std::unordered_map<std::string, std::vector<rclcpp::QoS>> input_topics_qos_profiles;
  std::unordered_map<std::string, std::string> input_topics_serialization_format;
  // message_definitions_map mapping topic_type to message_definition
  std::unordered_map<std::string, rosbag2_storage::MessageDefinition> message_definitions_map;

  for (const auto & input_bag : input_bags) {
    const auto & reader = input_bag.first;
    auto bag_topics_and_types = reader->get_all_topics_and_types();
    for (const auto & topic_metadata : bag_topics_and_types) {
      const std::string & topic_name = topic_metadata.name;
      input_topics.try_emplace(topic_name);
      input_topics[topic_name].push_back(topic_metadata.type);
      input_topics_serialization_format[topic_name] = topic_metadata.serialization_format;

      // Gather all offered qos profiles from all inputs
      input_topics_qos_profiles.try_emplace(topic_name);
      input_topics_qos_profiles[topic_name].insert(
        input_topics_qos_profiles[topic_name].end(),
        topic_metadata.offered_qos_profiles.begin(),
        topic_metadata.offered_qos_profiles.end()
      );
    }
    // Fill message_definitions_map
    std::vector<rosbag2_storage::MessageDefinition> msg_definitions;
    reader->get_all_message_definitions(msg_definitions);
    for (const auto & msg_definition : msg_definitions) {
      message_definitions_map[msg_definition.topic_type] = msg_definition;
    }
  }

  for (const auto & [writer, record_options] : output_bags) {
    rosbag2_transport::TopicFilter topic_filter{record_options, nullptr, true};
    auto filtered_topics_and_types = topic_filter.filter_topics(input_topics);

    std::string output_serialization_format = record_options.output_serialization_format;
    // Fall back to the deprecated rmw_serialization_format if output format is unspecified
    if (!record_options.rmw_serialization_format.empty() && output_serialization_format.empty()) {
      output_serialization_format = record_options.rmw_serialization_format;
    }

    // Done filtering - set up writer
    for (const auto & [topic_name, topic_type] : filtered_topics_and_types) {
      rosbag2_storage::TopicMetadata topic_metadata;
      topic_metadata.name = topic_name;
      topic_metadata.type = topic_type;

      // Take source serialization format for the topic if output format is unspecified
      if (output_serialization_format.empty()) {
        topic_metadata.serialization_format = input_topics_serialization_format[topic_name];
      } else {
        topic_metadata.serialization_format = output_serialization_format;
      }

      topic_metadata.offered_qos_profiles = input_topics_qos_profiles[topic_name];

      auto message_definition_ptr = message_definitions_map.find(topic_type);
      if (message_definition_ptr != message_definitions_map.end()) {
        writer->create_topic(topic_metadata, message_definition_ptr->second);
      } else {
        // Give a chance to find message definition in local environment
        writer->create_topic(topic_metadata);
      }
      filtered_outputs.try_emplace(topic_name);
      filtered_outputs[topic_name].push_back({writer.get(), &record_options});
    }
  }

  return filtered_outputs;
}

void perform_rewrite(
  std::vector<reader_storage_options_pair_t> & input_bags,
  const std::vector<writer_record_options_pair_t> & output_bags
)
{
  if (input_bags.empty() || output_bags.empty()) {
    throw std::runtime_error("Must provide at least one input and one output bag to rewrite.");
  }

  // Fail on invalid per-topic message ranges before anything is written
  validate_message_ranges(input_bags, output_bags);

  auto topic_outputs = setup_topic_filtering(input_bags, output_bags);

  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> next_messages;
  next_messages.resize(input_bags.size(), nullptr);

  // Store the position of messages within their topic's message sequence
  std::unordered_map<std::string, size_t> topic_message_indices;

  std::shared_ptr<rosbag2_storage::SerializedBagMessage> next_msg;
  while ((next_msg = get_next(input_bags, next_messages))) {
    auto iterator = topic_outputs.find(next_msg->topic_name);
    if (iterator != topic_outputs.end()) {
      // Determine specific message position
      const size_t message_index = topic_message_indices[next_msg->topic_name]++;
      for (const auto & output : iterator->second) {
        if (message_in_range(*output.record_options, next_msg->topic_name, message_index)) {
          output.writer->write(next_msg);
        }
      }
    }
  }
}

}  // namespace

namespace rosbag2_transport
{
void bag_rewrite(
  const std::vector<rosbag2_storage::StorageOptions> & input_options,
  const std::vector<
    std::pair<rosbag2_storage::StorageOptions, rosbag2_transport::RecordOptions>
  > & output_options
)
{
  std::vector<reader_storage_options_pair_t> input_bags;
  std::vector<writer_record_options_pair_t> output_bags;

  for (const auto & storage_options : input_options) {
    auto reader = ReaderWriterFactory::make_reader(storage_options);
    reader->open(storage_options);
    // Seek to start time if specified
    if (storage_options.start_time_ns > 0) {
      reader->seek(storage_options.start_time_ns);
    }
    input_bags.emplace_back(std::move(reader), storage_options);
  }

  for (auto & [storage_options, record_options] : output_options) {
    // TODO(emersonknapp) - utilize cache to get better performance.
    // For now, zero cache allows for synchronous writes which are guaranteed to go through.
    // With cache enabled, the buffer could overflow and drop messages in this fast-write loop.
    // To enable the cache we will need to implement a mechanism for the writer to take messages
    // only when it is able to, which will likely require some new APIs.
    auto zero_cache_storage_options = storage_options;
    zero_cache_storage_options.max_cache_size = 0u;
    auto writer = ReaderWriterFactory::make_writer(record_options);
    writer->open(zero_cache_storage_options);
    output_bags.emplace_back(std::move(writer), record_options);
  }

  perform_rewrite(input_bags, output_bags);
}
}  // namespace rosbag2_transport
