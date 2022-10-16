// Copyright 2022, Foxglove Technologies. All rights reserved.
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

/// @file single_benchmark.cpp
/// @brief writes some number of messages to a bag file using the
/// rosbag2_storage::ReadWriteInterface API, and measures timing and memory usage data for each
/// write call.
///
/// Arguments:
///   1. A YAML string to configure the benchmark run.
///   2. a path to write the bag file to.
///
/// Prints a csv-formatted table containing metrics from each write call. This binary isn't meant
/// to be called manually by humans, instead use `sweep.py` to run a parametric sweep across a range
/// of configs.
///
/// Example usage:
///  ./single_benchmark "$(cat your_config.yaml)" /tmp/some_existing_temp_dir > results.csv

#include <malloc.h>
#include <random>
#include <fstream>
#include <climits>
#include <algorithm>
#include <functional>
#include <vector>
#include <iostream>
#include <unordered_map>

#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_storage/ros_helper.hpp"
#include "rcutils/logging_macros.h"

#include "config.hpp"

using RandomEngine = std::independent_bits_engine<std::default_random_engine, CHAR_BIT,
    unsigned char>;
using hrc = std::chrono::high_resolution_clock;
using Batch = std::vector<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>>;

/**
 * @brief Generates the TopicMetadata objects needed by ReadWriteInterface before writing the
 * topics produced by this benchmark.
 * @param config the benchmark config for this run.
 * @return std::vector<rosbag2_storage::TopicMetadata> all TopicMetadata objects to go into the bag.
 */
std::vector<rosbag2_storage::TopicMetadata> generate_topics(const Config & config)
{
  std::vector<rosbag2_storage::TopicMetadata> topics;
  for (const auto & topic : config.topics) {
    rosbag2_storage::TopicMetadata metadata;
    metadata.name = topic.name;
    // The topic type doesn't matter here - we're not doing any serialization,
    // just throwing random bytes into the serialized message.
    // It still needs to be a valid typename because the MCAP plugin will search the resource
    // index for it.
    metadata.type = "std_msgs/String";
    metadata.serialization_format = "cdr";
    metadata.offered_qos_profiles = "";
    topics.push_back(metadata);
  }
  return topics;
}

/**
 * @brief Produce a random byte array, suitable for the serialized_data field of SerializedBagMessage.
 *
 * @param size The size of the array to produce
 * @param engine The random number generator, shared between runs.
 * @return std::shared_ptr<rcutils_uint8_array_t> A bytearray full of random data of length `size`.
 */
std::shared_ptr<rcutils_uint8_array_t> random_uint8_array(size_t size, RandomEngine & engine)
{
  std::vector<unsigned char> data(size);
  // std::generate(begin(data), end(data), std::ref(engine));
  return rosbag2_storage::make_serialized_message(data.data(), data.size());
}

/**
 * @brief Generates all messages to write to the bag.
 *
 * @param config The benchmark config for this run.
 * @return std::vector<Batch> The messages to write in this run, broken up into Batches. Each
 * batch is written to the bag in its own write() call.
 */
std::vector<Batch> generate_messages(Config config)
{
  std::vector<Batch> out;
  Batch current_batch;
  size_t current_batch_bytes = 0;
  RandomEngine engine;
  // We use a second random engine for shuffling the vector of messages, to ensure
  // roughly-evenly-sized batches.
  std::default_random_engine shuffle_engine(0);

  // build a long vector of pointers to the relevant topic config for each message that will be
  // created.
  std::vector<const Config::TopicConfig *> config_to_write;
  double proportion_sum = 0.0;
  for (size_t i = 0; i < config.topics.size(); ++i) {
    const auto * topic_config = &config.topics[i];
    proportion_sum += topic_config->write_proportion;
    double this_topic_bytes = (double)(config.write_total_bytes) * topic_config->write_proportion;
    size_t num_msgs = (size_t)(this_topic_bytes / double(topic_config->message_size));
    for (size_t j = 0; j < num_msgs; ++j) {
      config_to_write.push_back(topic_config);
    }
  }
  if ((proportion_sum > 1.0001) || (proportion_sum < 0.9999)) {
    throw std::runtime_error("topic config proportions do not sum close to 1");
  }
  // shuffle the messages to be created, to make the batches all roughly the same size.
  std::shuffle(config_to_write.begin(), config_to_write.end(), shuffle_engine);

  for (auto topic_config: config_to_write) {
    auto msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
    msg->topic_name = topic_config->name;
    msg->serialized_data = random_uint8_array(topic_config->message_size, engine);
    current_batch_bytes += topic_config->message_size;
    current_batch.push_back(msg);
    if (current_batch_bytes >= config.min_batch_size_bytes) {
      out.emplace_back(std::move(current_batch));
      current_batch = {};
      current_batch_bytes = 0;
    }
  }
  if (current_batch.size() != 0) {
    out.emplace_back(std::move(current_batch));
  }
  return out;
}

/**
 * @brief Returns the number of bytes in all messages in this batch.
 */
size_t message_bytes(const Batch & batch)
{
  size_t total = 0;
  for (const auto & msgPtr : batch) {
    total += msgPtr->serialized_data->buffer_length;
  }
  return total;
}

/**
 * @brief Contains the baseline memory usage of the application before creating a writer and calling
 * `write()`.
 */
struct BaselineStat
{
  size_t arena_bytes;
  size_t in_use_bytes;
  size_t mmap_bytes;

  BaselineStat()
  {
    struct mallinfo2 info = mallinfo2();
    arena_bytes = info.arena;
    in_use_bytes = info.uordblks;
    mmap_bytes = info.hblkhd;
  }
};

/**
 * @brief The metrics measured from a single writer.write() call.
 */
struct WriteStat
{
  uint32_t sqc;
  size_t bytes_written;
  size_t num_msgs;
  hrc::duration write_duration;
  ssize_t arena_bytes;
  ssize_t in_use_bytes;
  ssize_t mmap_bytes;

  WriteStat(
    const BaselineStat & baseline_stat, uint32_t sqc_, const Batch & batch,
    hrc::duration write_duration)
  : sqc(sqc_),
    bytes_written(message_bytes(batch)),
    num_msgs(batch.size()),
    write_duration(write_duration)
  {
    struct mallinfo2 info = mallinfo2();
    arena_bytes = info.arena - baseline_stat.arena_bytes;
    in_use_bytes = info.uordblks - baseline_stat.in_use_bytes;
    mmap_bytes = info.hblkhd - baseline_stat.mmap_bytes;
  }
};


int main(int argc, const char ** argv)
{
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << " <config yaml string> <output dir>" << std::endl;
    std::cerr <<
      "Use ros2 run rosbag2_storage_plugin_comparison sweep.py for a more ergonomic experience" <<
      std::endl;
    return 1;
  }
  YAML::Node config_yaml = YAML::Load(argv[1]);
  Config config = config_yaml.as<Config>();
  RCUTILS_LOG_INFO_NAMED("single_benchmark", "generating %ld topics", config.topics.size());
  auto topics = generate_topics(config);
  RCUTILS_LOG_INFO_NAMED("single_benchmark", "generating some messages");
  auto messages = generate_messages(config);
  RCUTILS_LOG_INFO_NAMED("single_benchmark", "configuring writer");
  rosbag2_storage::StorageFactory factory;
  rosbag2_storage::StorageOptions options;
  options.uri = std::string(argv[2]) + "/out";
  options.storage_id = config.storage_id;

  if (config.storage_options.IsMap()) {
    std::string storage_options_uri = std::string(argv[2]) + "/storage_options.yaml";
    RCUTILS_LOG_INFO_NAMED(
      "single_benchmark", "using storage options %s",
      storage_options_uri.c_str());
    std::ofstream fout(storage_options_uri.c_str());
    fout << config.storage_options;
    options.storage_config_uri = storage_options_uri;
  }

  std::vector<WriteStat> write_stats;
  write_stats.reserve(messages.size());
  std::chrono::high_resolution_clock::time_point close_start_time;
  RCUTILS_LOG_INFO_NAMED("single_benchmark", "writing messages");
  BaselineStat baseline;
  {
    auto writer = factory.open_read_write(options);

    for (const auto & topic : topics) {
      writer->create_topic(topic);
    }

    uint32_t sqc = 0;
    // write messages, timing each write
    for (const auto & message_batch : messages) {
      auto start_time = hrc::now();
      writer->write(message_batch);
      hrc::duration duration = hrc::now() - start_time;
      write_stats.emplace_back(baseline, sqc, message_batch, duration);
      sqc++;
    }

    close_start_time = hrc::now();
    // writer destructor closes the output file, so we time that too.
  }
  auto close_duration = hrc::now() - close_start_time;

  std::cout << "sqc,num_bytes,num_msgs,write_ns,arena_bytes,in_use_bytes,mmap_bytes,close_ns" <<
    std::endl;
  for (const auto & stat : write_stats) {
    auto duration =
      std::chrono::duration_cast<std::chrono::nanoseconds>(stat.write_duration).count();
    std::cout <<
      stat.sqc << "," <<
      stat.bytes_written << "," <<
      stat.num_msgs << "," <<
      duration << "," <<
      stat.arena_bytes << "," <<
      stat.in_use_bytes << "," <<
      stat.mmap_bytes << "," <<
      std::endl;
  }
  std::cout << ",,,,,,," <<
    std::chrono::duration_cast<std::chrono::nanoseconds>(close_duration).count() << std::endl;
  return 0;
}
