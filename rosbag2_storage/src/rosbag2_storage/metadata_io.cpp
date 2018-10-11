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

#include "rosbag2_storage/filesystem_helper.hpp"

#ifdef _WIN32
// This is necessary because of a bug in yaml-cpp's cmake
#define YAML_CPP_DLL
// This is necessary because yaml-cpp does not always use dllimport/dllexport consistently
# pragma warning(push)
# pragma warning(disable:4251)
# pragma warning(disable:4275)
#endif
#include "yaml-cpp/yaml.h"
#ifdef _WIN32
# pragma warning(pop)
#endif

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
struct convert<std::chrono::time_point<std::chrono::high_resolution_clock>>
{
  static Node encode(const std::chrono::time_point<std::chrono::high_resolution_clock> & start_time)
  {
    Node node;
    node["nanoseconds_since_epoch"] = start_time.time_since_epoch().count();
    return node;
  }

  static bool decode(
    const Node & node, std::chrono::time_point<std::chrono::high_resolution_clock> & start_time)
  {
    start_time = std::chrono::time_point<std::chrono::high_resolution_clock>(
      std::chrono::nanoseconds(node["nanoseconds_since_epoch"].as<size_t>()));
    return true;
  }
};

template<>
struct convert<rosbag2_storage::BagMetadata>
{
  static Node encode(const rosbag2_storage::BagMetadata & metadata)
  {
    Node node;
    node["version"] = metadata.version;
    node["storage_identifier"] = metadata.storage_identifier;
    node["storage_format"] = metadata.storage_format;
    node["relative_file_paths"] = metadata.relative_file_paths;
    node["duration"] = metadata.duration;
    node["starting_time"] = metadata.starting_time;
    node["message_count"] = metadata.message_count;
    node["topics_with_message_count"] = metadata.topics_with_message_count;
    return node;
  }

  static bool decode(const Node & node, rosbag2_storage::BagMetadata & metadata)
  {
    metadata.version = node["version"].as<int>();
    metadata.storage_identifier = node["storage_identifier"].as<std::string>();
    metadata.storage_format = node["storage_format"].as<std::string>();
    metadata.relative_file_paths = node["relative_file_paths"].as<std::vector<std::string>>();
    metadata.duration = node["duration"].as<std::chrono::nanoseconds>();
    metadata.starting_time = node["starting_time"]
      .as<std::chrono::time_point<std::chrono::high_resolution_clock>>();
    metadata.message_count = node["message_count"].as<size_t>();
    metadata.topics_with_message_count =
      node["topics_with_message_count"].as<std::vector<rosbag2_storage::TopicMetadata>>();
    return true;
  }
};

}  // namespace YAML

namespace rosbag2_storage
{

void MetadataIo::write_metadata(const std::string & uri, BagMetadata metadata)
{
  YAML::Node metadata_node;
  metadata_node["rosbag2_bagfile_information"] = metadata;
  std::ofstream fout(get_metadata_file_name(uri));
  fout << metadata_node;
}

BagMetadata MetadataIo::read_metadata(const std::string & uri)
{
  try {
    YAML::Node yaml_file = YAML::LoadFile(get_metadata_file_name(uri));
    auto metadata = yaml_file["rosbag2_bagfile_information"].as<rosbag2_storage::BagMetadata>();
    metadata.bag_size = FilesystemHelper::calculate_directory_size(uri);
    return metadata;
  } catch (const YAML::Exception & ex) {
    throw std::runtime_error(std::string("Exception on parsing info file: ") + ex.what());
  }
}

std::string MetadataIo::get_metadata_file_name(const std::string & uri)
{
  std::string metadata_file = rosbag2_storage::FilesystemHelper::concat({uri, metadata_filename});

  return metadata_file;
}

}  // namespace rosbag2_storage
