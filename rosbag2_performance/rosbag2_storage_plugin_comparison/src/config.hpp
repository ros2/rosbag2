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
#include <string>
#include "yaml-cpp/yaml.h"

struct Config
{
  /// The storage ID to pass in StorageOptions.
  std::string storage_id = "sqlite3";
  /// The number of bytes to reach in a given batch before starting a new one.
  /// Writes are always batched when passed to rosbag2_storage::ReadWriteInterface.
  /// A small limit (like 10) means that only one message will be in each batch.
  size_t min_batch_size_bytes = 10;
  /// The number of bytes to write in total. Defaults to 1G.
  size_t write_total_bytes = 250'000'000;

  /// A set of topics to write to the bag. All messages contain only random data and the
  /// topics are meaningless except as unique labels for data of different sizes.
  struct TopicConfig
  {
    /// The topic name. Should be unique.
    std::string name;
    /// The size of each message to write in bytes.
    size_t message_size;
    /// The proportion of `write_total_bytes` that should be written as messages of this topic.
    /// If using multiple topics, these proportion values should sum to 1.0.
    double write_proportion = 1.0;
  };
  std::vector<TopicConfig> topics = {{"/large", 1'000'000, 0.9}, {"/small", 1000, 0.1}};

  YAML::Node storage_options;
};


/// Conversions to/from YAML for Config.
namespace YAML
{
template<>
struct convert<Config>
{
  static Node encode(const Config & rhs)
  {
    Node node;
    node["storage_id"] = rhs.storage_id;
    node["min_batch_size_bytes"] = rhs.min_batch_size_bytes;
    node["write_total_bytes"] = rhs.write_total_bytes;
    Node topic_configs_node;
    for (const auto & topic_config : rhs.topics) {
      Node topic_config_node;
      topic_config_node["name"] = topic_config.name;
      topic_config_node["message_size"] = topic_config.message_size;
      topic_config_node["write_proportion"] = topic_config.write_proportion;
      topic_configs_node.push_back(topic_config_node);
    }
    node["topics"] = topic_configs_node;
    node["storage_options"] = rhs.storage_options;
    return node;
  }

  static bool decode(const Node & node, Config & rhs)
  {
    optional_assign<std::string>(node, "storage_id", rhs.storage_id);
    optional_assign<size_t>(node, "min_batch_size_bytes", rhs.min_batch_size_bytes);
    optional_assign<size_t>(node, "write_total_bytes", rhs.write_total_bytes);
    if (node["topics"]) {
      rhs.topics.clear();
      auto topics_node = node["topics"];
      for (auto it = topics_node.begin(); it != topics_node.end(); ++it) {
        Config::TopicConfig topic;
        topic.name = (*it)["name"].as<std::string>();
        topic.message_size = (*it)["message_size"].as<size_t>();
        optional_assign<double>(*it, "write_proportion", topic.write_proportion);
        rhs.topics.push_back(topic);
      }
    }
    optional_assign<Node>(node, "storage_options", rhs.storage_options);
    return true;
  }
};
}
