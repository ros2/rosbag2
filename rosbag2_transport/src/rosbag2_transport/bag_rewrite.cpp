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

#include "logging.hpp"
#include "topic_filter.hpp"

namespace
{

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
    }

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
  std::map<std::string, std::vector<std::string>> input_topics;
  std::unordered_map<std::string, YAML::Node> input_topics_qos_profiles;
  std::unordered_map<std::string, std::string> input_topics_serialization_format;

  for (const auto & input_bag : input_bags) {
    auto bag_topics_and_types = input_bag->get_all_topics_and_types();
    for (const auto & topic_metadata : bag_topics_and_types) {
      const std::string & topic_name = topic_metadata.name;
      input_topics.try_emplace(topic_name);
      input_topics[topic_name].push_back(topic_metadata.type);
      input_topics_serialization_format[topic_name] = topic_metadata.serialization_format;

      // Gather all offered qos profiles from all inputs
      input_topics_qos_profiles.try_emplace(topic_name);
      YAML::Node & all_offered = input_topics_qos_profiles[topic_name];
      YAML::Node offered_qos_profiles = YAML::Load(topic_metadata.offered_qos_profiles);
      for (auto qos : offered_qos_profiles) {
        all_offered.push_back(qos);
      }
    }
  }

  for (const auto & [writer, record_options] : output_bags) {
    rosbag2_transport::TopicFilter topic_filter{record_options};
    auto filtered_topics_and_types = topic_filter.filter_topics(input_topics);

    // Done filtering - set up writer
    for (const auto & [topic_name, topic_type] : filtered_topics_and_types) {
      rosbag2_storage::TopicMetadata topic_metadata;
      topic_metadata.name = topic_name;
      topic_metadata.type = topic_type;

      // Take source serialization format for the topic if output format is unspecified
      if (record_options.rmw_serialization_format.empty()) {
        topic_metadata.serialization_format = input_topics_serialization_format[topic_name];
      } else {
        topic_metadata.serialization_format = record_options.rmw_serialization_format;
      }

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

void perform_rewrite(
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
  std::vector<std::unique_ptr<rosbag2_cpp::Reader>> input_bags;
  std::vector<
    std::pair<std::unique_ptr<rosbag2_cpp::Writer>, rosbag2_transport::RecordOptions>
  > output_bags;

  for (const auto & storage_options : input_options) {
    auto reader = ReaderWriterFactory::make_reader(storage_options);
    reader->open(storage_options);
    input_bags.push_back(std::move(reader));
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
    output_bags.push_back(std::make_pair(std::move(writer), record_options));
  }

  perform_rewrite(input_bags, output_bags);
}
}  // namespace rosbag2_transport
