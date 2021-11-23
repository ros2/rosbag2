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

#include <memory>

#include "rosbag2_compression/compression_options.hpp"
#include "rosbag2_compression/sequential_compression_reader.hpp"
#include "rosbag2_compression/sequential_compression_writer.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_cpp/writers/sequential_writer.hpp"
#include "rosbag2_transport/bag_rewrite.hpp"

#include "logging.hpp"
#include "topic_filter.hpp"

namespace
{

/// Create a Reader with the appropriate underlying implementation.
std::unique_ptr<rosbag2_cpp::Reader> make_reader(
  const rosbag2_storage::StorageOptions & storage_options)
{
  rosbag2_storage::MetadataIo metadata_io;
  std::unique_ptr<rosbag2_cpp::reader_interfaces::BaseReaderInterface> reader_impl;

  if (metadata_io.metadata_file_exists(storage_options.uri)) {
    auto metadata = metadata_io.read_metadata(storage_options.uri);
    if (!metadata.compression_format.empty()) {
      reader_impl = std::make_unique<rosbag2_compression::SequentialCompressionReader>();
    }
  }
  if (!reader_impl) {
    reader_impl = std::make_unique<rosbag2_cpp::readers::SequentialReader>();
  }

  return std::make_unique<rosbag2_cpp::Reader>(std::move(reader_impl));
}


/// Create a Writer with the appropriate underlying implementation.
std::unique_ptr<rosbag2_cpp::Writer> make_writer(
  const rosbag2_transport::RecordOptions & record_options)
{
  std::unique_ptr<rosbag2_cpp::writer_interfaces::BaseWriterInterface> writer_impl;
  if (!record_options.compression_format.empty()) {
    rosbag2_compression::CompressionOptions compression_options {
      record_options.compression_format,
      rosbag2_compression::compression_mode_from_string(record_options.compression_mode),
      record_options.compression_queue_size,
      record_options.compression_threads
    };
    if (compression_options.compression_threads < 1) {
      compression_options.compression_threads = std::thread::hardware_concurrency();
    }
    writer_impl = std::make_unique<rosbag2_compression::SequentialCompressionWriter>(
      compression_options);
  } else {
    writer_impl = std::make_unique<rosbag2_cpp::writers::SequentialWriter>();
  }

  return std::make_unique<rosbag2_cpp::Writer>(std::move(writer_impl));
}

/// Find the next chronological message from all opened input bags.
/// Updates the next_messages queue as necessary.
/// next_messages is needed because Reader has no "peek" interface, we cannot put a message back.
std::shared_ptr<rosbag2_storage::SerializedBagMessage> get_next(
  const std::vector<std::unique_ptr<rosbag2_cpp::Reader>> & input_bags,
  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> & next_messages)
{
  // find message with lowest timestamp
  std::shared_ptr<rosbag2_storage::SerializedBagMessage> earliest_msg = nullptr;
  size_t earliest_msg_index = -1;
  for (size_t i = 0; i < next_messages.size(); i++) {
    // refill queue if bag not empty
    if (next_messages[i] == nullptr && input_bags[i]->has_next()) {
      next_messages[i] = input_bags[i]->read_next();
    };

    auto & msg = next_messages[i];
    if (msg == nullptr) {
      continue;
    }
    if (earliest_msg == nullptr || msg->time_stamp < earliest_msg->time_stamp) {
      earliest_msg = msg;
      earliest_msg_index = i;
    }
  }

  // clear returned message from queue before returning it, so it can be refilled next time
  if (earliest_msg != nullptr) {
    next_messages[earliest_msg_index].reset();
  }
  return earliest_msg;
}

/// Discover what topics are in the inputs, filter out topics that can't be processed,
/// create_topic on Writers that will receive topics.
/// Return a map f topic -> vector of which Writers want to receive that topic,
/// based on the RecordOptions.
/// The output vector has bare pointers to the uniquely owned Writers,
/// so this may not outlive the output_bags Writers.
std::unordered_map<std::string, std::vector<rosbag2_cpp::Writer *>>
setup_topic_filtering(
  const std::vector<std::unique_ptr<rosbag2_cpp::Reader>> & input_bags,
  const std::vector<
      std::pair<std::unique_ptr<rosbag2_cpp::Writer>, rosbag2_transport::RecordOptions>
    > & output_bags)
{
  std::unordered_map<std::string, std::vector<rosbag2_cpp::Writer *>> filtered_outputs;
  std::unordered_map<std::string, std::string> input_topics;
  std::unordered_map<std::string, YAML::Node> input_topics_qos_profiles;

  // Filter inputs
  {
    std::unordered_map<std::string, std::unordered_set<std::string>> topic_names_and_types;
    std::unordered_set<std::string> unknown_types;

    for (const auto & input_bag : input_bags) {
      auto bag_topics_and_types = input_bag->get_all_topics_and_types();
      for (const auto & topic_metadata : bag_topics_and_types) {
        const std::string & topic_name = topic_metadata.name;
        topic_names_and_types.try_emplace(topic_name);
        topic_names_and_types[topic_name].insert(topic_metadata.type);

        // Gather all offered qos profiles from all inputs
        input_topics_qos_profiles.try_emplace(topic_name);
        YAML::Node & all_offered = input_topics_qos_profiles[topic_name];
        YAML::Node offered_qos_profiles = YAML::Load(topic_metadata.offered_qos_profiles);
        for (auto qos : offered_qos_profiles) {
          all_offered.push_back(qos);
        }
      }
    }

    // Filter topics with more than one type
    for (const auto & [topic_name, topic_types] : topic_names_and_types) {
      if (topic_types.size() > 1) {
        ROSBAG2_TRANSPORT_LOG_WARN_STREAM(
          "Topic '" << topic_name << "' has multiple types from inputs. " <<
          "Topics must have a single type, skipping topic.");
      } else {
        std::string topic_type = *topic_types.begin();
        input_topics[topic_name] = topic_type;
      }
    }

    input_topics = rosbag2_transport::topic_filter::filter_topics_with_known_type(
      input_topics, unknown_types);
  }

  // Filter to outputs
  for (const auto & [writer, record_options] : output_bags) {
    auto filtered_topics_and_types = input_topics;
    if (!record_options.topics.empty()) {
      std::vector<std::string> expanded_topics;
      for (const std::string & topic : record_options.topics) {
        auto expanded_topic = rclcpp::expand_topic_or_service_name(
          topic, "dummy_node_name", "/", false);
        expanded_topics.push_back(expanded_topic);
      }
      filtered_topics_and_types = rosbag2_transport::topic_filter::filter_topics(
        expanded_topics, input_topics);
    }
    if (!record_options.regex.empty() || !record_options.exclude.empty()) {
      filtered_topics_and_types = rosbag2_transport::topic_filter::filter_topics_using_regex(
        filtered_topics_and_types,
        record_options.regex,
        record_options.exclude,
        record_options.all);
    }

    // Done filtering - set up writer
    for (const auto & [topic_name, topic_type] : filtered_topics_and_types) {
      rosbag2_storage::TopicMetadata topic_metadata;
      topic_metadata.name = topic_name;
      topic_metadata.type = topic_type;
      topic_metadata.serialization_format = record_options.rmw_serialization_format;
      std::stringstream qos_profiles;
      qos_profiles << input_topics_qos_profiles[topic_name];
      topic_metadata.offered_qos_profiles = qos_profiles.str();
      writer->create_topic(topic_metadata);

      filtered_outputs.try_emplace(topic_name);
      filtered_outputs[topic_name].push_back(writer.get());
    }
  }

  return filtered_outputs;
}

}  // namespace

namespace rosbag2_transport
{

void bag_rewrite(
  const std::vector<std::unique_ptr<rosbag2_cpp::Reader>> & input_bags,
  const std::vector<
      std::pair<std::unique_ptr<rosbag2_cpp::Writer>, rosbag2_transport::RecordOptions>
    > & output_bags
)
{
  if (input_bags.empty() || output_bags.empty()) {
    throw std::runtime_error("Must provide at least one input and one output bag to rewrite.");
  }

  auto topic_outputs = setup_topic_filtering(input_bags, output_bags);

  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> next_messages;
  next_messages.resize(input_bags.size(), nullptr);

  std::shared_ptr<rosbag2_storage::SerializedBagMessage> next_msg;
  while (next_msg = get_next(input_bags, next_messages)) {
    auto topic_writers = topic_outputs.find(next_msg->topic_name);
    if (topic_writers != topic_outputs.end()) {
      for (auto writer : topic_writers->second) {
        writer->write(next_msg);
      }
    }
  }
}

void bag_rewrite(
  const std::vector<rosbag2_storage::StorageOptions> & input_options,
  const std::vector<std::pair<rosbag2_storage::StorageOptions, rosbag2_transport::RecordOptions>>
    & output_options
)
{
  std::vector<std::unique_ptr<rosbag2_cpp::Reader>> input_bags;
  std::vector<
      std::pair<std::unique_ptr<rosbag2_cpp::Writer>, rosbag2_transport::RecordOptions>
    > output_bags;

  for (const auto & storage_options : input_options) {
    auto reader = make_reader(storage_options);
    reader->open(storage_options);
    input_bags.push_back(std::move(reader));
  }

  for (auto & [storage_options, record_options] : output_options) {
    // TODO(emersonknapp) - utilize cache to get better performance.
    // For now though, this allows for synchronous writes which are guaranteed to go through,
    // whereas the cache buffer could overflow, dropping messages, in a fast-write loop.
    // To get around that we will have to be careful to not push new messages until there is
    // room in the cache, which may require some new APIs
    auto modified_storage_options = storage_options;
    modified_storage_options.max_cache_size = 0u;
    auto writer = make_writer(record_options);
    writer->open(modified_storage_options);
    output_bags.push_back(std::make_pair(std::move(writer), record_options));
  }

  bag_rewrite(input_bags, output_bags);
}

}  // namespace rosbag2_transport
