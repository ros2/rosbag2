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

#include "rosbag2_storage/metadata_io.hpp"

#include <fstream>
#include <string>
#include <vector>

#include "rosbag2_storage/bag_metadata.hpp"
#include "rosbag2_storage/topic_with_type.hpp"
#include "yaml-cpp/yaml.h"
#include "yaml-cpp/parser.h"
#include "yaml-cpp/emitter.h"

namespace YAML
{
template<>
struct convert<rosbag2_storage::TopicWithType>
{
  static Node encode(const rosbag2_storage::TopicWithType & topic)
  {
    Node node;
    node["name"] = topic.name;
    node["type"] = topic.type;
    return node;
  }

  static bool decode(const Node & node, rosbag2_storage::TopicWithType & topic)
  {
    topic.name = node["name"].as<std::string>();
    topic.type = node["type"].as<std::string>();
    return true;
  }
};

template<>
struct convert<rosbag2_storage::TopicMetadata>
{
  static Node encode(const rosbag2_storage::TopicMetadata & metadata)
  {
    Node node;
    node["topic_and_type"] = metadata.topic_with_type;
    node["message_count"] = metadata.message_count;
    return node;
  }

  static bool decode(const Node & node, rosbag2_storage::TopicMetadata & metadata)
  {
    metadata.topic_with_type = node["topic_and_type"].as<rosbag2_storage::TopicWithType>();
    metadata.message_count = node["message_count"].as<size_t>();
    return true;
  }
};

template<>
struct convert<std::chrono::nanoseconds>
{
  static Node encode(const std::chrono::nanoseconds & time_in_ns)
  {
    Node node;
    node["nanoseconds"] = time_in_ns.count();
    return node;
  }

  static bool decode(const Node & node, std::chrono::nanoseconds & time_in_ns)
  {
    time_in_ns = std::chrono::nanoseconds(node["nanoseconds"].as<size_t>());
    return true;
  }
};

template<>
struct convert<rosbag2_storage::BagMetadata>
{
  static Node encode(const rosbag2_storage::BagMetadata & metadata)
  {
    Node node;
    node["storage_identifier"] = metadata.storage_identifier;
    node["encoding"] = metadata.encoding;
    node["relative_file_paths"] = metadata.relative_file_paths;
    node["combined_bag_size"] = metadata.combined_bag_size;
    node["duration"] = metadata.duration_in_nanoseconds;
    node["time_start"] = metadata.time_start_in_nanoseconds;
    node["message_count"] = metadata.message_count;
    node["topics_with_message_count"] = metadata.topics_with_message_count;
    return node;
  }

  static bool decode(const Node & node, rosbag2_storage::BagMetadata & metadata)
  {
    metadata.storage_identifier = node["storage_identifier"].as<std::string>();
    metadata.encoding = node["encoding"].as<std::string>();
    metadata.relative_file_paths = node["relative_file_paths"].as<std::vector<std::string>>();
    metadata.combined_bag_size = node["combined_bag_size"].as<size_t>();
    metadata.duration_in_nanoseconds = node["duration"].as<std::chrono::nanoseconds>();
    metadata.time_start_in_nanoseconds = node["time_start"].as<std::chrono::nanoseconds>();
    metadata.message_count = node["message_count"].as<size_t>();
    metadata.topics_with_message_count =
      node["topics_with_message_count"].as<std::vector<rosbag2_storage::TopicMetadata>>();
    return true;
  }
};

}  // namespace YAML

namespace rosbag2_storage
{

void write_metadata(std::string filename, BagMetadata metadata)
{
  YAML::Node metadata_node;
  metadata_node["rosbag2_bagfile_information"] = metadata;
  std::ofstream fout(filename);
  fout << metadata_node;
}

BagMetadata read_metadata(std::string filename)
{
  YAML::Node yaml_file = YAML::LoadFile(filename);
  auto metadata = yaml_file["rosbag2_bagfile_information"].as<rosbag2_storage::BagMetadata>();
  return metadata;
}

}  // namespace rosbag2_storage
