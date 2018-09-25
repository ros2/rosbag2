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

#ifndef ROSBAG2_STORAGE__METADATA_IO_HPP_
#define ROSBAG2_STORAGE__METADATA_IO_HPP_

#include <string>
#include <vector>

#include "rosbag2_storage/metadata_io_iface.hpp"
#include "rosbag2_storage/topic_with_type.hpp"
#include "rosbag2_storage/visibility_control.hpp"

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
    node["storage_identifier"] = metadata.storage_identifier;
    node["encoding"] = metadata.encoding;
    node["relative_file_paths"] = metadata.relative_file_paths;
    node["combined_bag_size"] = metadata.combined_bag_size;
    node["duration"] = metadata.duration;
    node["starting_time"] = metadata.starting_time;
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

class MetadataIO : public MetadataIOIface
{
public:
  explicit MetadataIO(const std::string & uri);
  ~MetadataIO() override = default;

  ROSBAG2_STORAGE_PUBLIC void write_metadata(BagMetadata metadata) override;
  ROSBAG2_STORAGE_PUBLIC BagMetadata read_metadata() override;

private:
  std::string get_metadata_file_name(const std::string & uri);

  std::string file_name_;
};

}  // namespace rosbag2_storage

#endif  // ROSBAG2_STORAGE__METADATA_IO_HPP_
