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
#ifndef ROSBAG2_STORAGE__YAML_HPP_
#define ROSBAG2_STORAGE__YAML_HPP_

#include <algorithm>
#include <string>
#include <unordered_map>
#include <vector>

#ifdef _WIN32
// This is necessary because yaml-cpp does not always use dllimport/dllexport consistently
# pragma warning(push)
# pragma warning(disable:4251)
# pragma warning(disable:4275)
#endif
#include "yaml-cpp/yaml.h"
#ifdef _WIN32
# pragma warning(pop)
#endif

#include "rclcpp/duration.hpp"
#include "rosbag2_storage/bag_metadata.hpp"
#include "rosbag2_storage/qos.hpp"

namespace YAML
{

/// If "node" has a field named "field", then assign its value to "assign_to" destination.
template<typename T>
void optional_assign(const Node & node, std::string field, T & assign_to)
{
  if (node[field]) {
    YAML::convert<T>::decode(node[field], assign_to);
  }
}

template<>
struct convert<std::unordered_map<std::string, std::string>>
{
  static Node encode(const std::unordered_map<std::string, std::string> & custom_data)
  {
    Node node;
    for (const auto & it : custom_data) {
      node[it.first] = it.second;
    }
    return node;
  }

  static bool decode(const Node & node, std::unordered_map<std::string, std::string> & custom_data)
  {
    for (YAML::const_iterator it = node.begin(); it != node.end(); ++it) {
      custom_data.emplace(it->first.as<std::string>(), it->second.as<std::string>());
    }
    return true;
  }
};

template<>
struct convert<rclcpp::Duration>
{
  static Node encode(const rclcpp::Duration & duration)
  {
    Node node;
    node["sec"] = duration.nanoseconds() / 1000000000;
    node["nsec"] = duration.nanoseconds() % 1000000000;
    return node;
  }

  static bool decode(const Node & node, rclcpp::Duration & duration)
  {
    duration = rclcpp::Duration(node["sec"].as<int32_t>(), node["nsec"].as<uint32_t>());
    return true;
  }
};

template<>
struct convert<std::chrono::milliseconds>
{
  static Node encode(const std::chrono::milliseconds & duration)
  {
    Node node;
    node["milliseconds"] = duration.count();
    return node;
  }

  static bool decode(const Node & node, std::chrono::milliseconds & duration)
  {
    duration = std::chrono::milliseconds(node["milliseconds"].as<std::chrono::milliseconds::rep>());
    return true;
  }
};

template<>
struct convert<rosbag2_storage::TopicMetadata>
{
  static Node encode(const rosbag2_storage::TopicMetadata & topic, int version)
  {
    Node node;
    node["name"] = topic.name;
    node["type"] = topic.type;
    node["serialization_format"] = topic.serialization_format;
    if (version < 9) {
      node["offered_qos_profiles"] = rosbag2_storage::serialize_rclcpp_qos_vector(
        topic.offered_qos_profiles, version);
    } else {
      node["offered_qos_profiles"] = YAML::convert<std::vector<rclcpp::QoS>>::encode(
        topic.offered_qos_profiles, version);
    }
    node["type_description_hash"] = topic.type_description_hash;
    return node;
  }

  static bool decode(const Node & node, rosbag2_storage::TopicMetadata & topic, int version)
  {
    topic.name = node["name"].as<std::string>();
    topic.type = node["type"].as<std::string>();
    topic.serialization_format = node["serialization_format"].as<std::string>();
    if (version >= 9) {
      topic.offered_qos_profiles =
        YAML::decode_for_version<std::vector<rclcpp::QoS>>(node["offered_qos_profiles"], version);
    } else if (version >= 4) {
      std::string qos_str = node["offered_qos_profiles"].as<std::string>();
      topic.offered_qos_profiles = rosbag2_storage::to_rclcpp_qos_vector(qos_str, version);
    }
    if (version >= 7) {
      topic.type_description_hash = node["type_description_hash"].as<std::string>();
    } else {
      topic.type_description_hash = "";
    }
    return true;
  }
};

template<>
struct convert<rosbag2_storage::TopicInformation>
{
  static Node encode(const rosbag2_storage::TopicInformation & topic_info, int version)
  {
    Node node;
    node["topic_metadata"] =
      convert<rosbag2_storage::TopicMetadata>::encode(topic_info.topic_metadata, version);
    node["message_count"] = topic_info.message_count;
    return node;
  }

  static bool decode(const Node & node, rosbag2_storage::TopicInformation & metadata, int version)
  {
    metadata.topic_metadata = decode_for_version<rosbag2_storage::TopicMetadata>(
      node["topic_metadata"], version);
    metadata.message_count = node["message_count"].as<uint64_t>();
    return true;
  }
};

template<>
struct convert<std::vector<rosbag2_storage::TopicInformation>>
{
  static Node encode(const std::vector<rosbag2_storage::TopicInformation> & rhs, int version)
  {
    Node node{NodeType::Sequence};
    for (const auto & value : rhs) {
      node.push_back(convert<rosbag2_storage::TopicInformation>::encode(value, version));
    }
    return node;
  }

  static bool decode(
    const Node & node, std::vector<rosbag2_storage::TopicInformation> & rhs, int version)
  {
    if (!node.IsSequence()) {
      return false;
    }

    rhs.clear();
    for (const auto & value : node) {
      rhs.push_back(decode_for_version<rosbag2_storage::TopicInformation>(value, version));
    }
    return true;
  }
};

template<>
struct convert<rosbag2_storage::FileInformation>
{
  static Node encode(const rosbag2_storage::FileInformation & metadata)
  {
    Node node;
    node["path"] = metadata.path;
    node["starting_time"] = metadata.starting_time;
    node["duration"] = metadata.duration;
    node["message_count"] = metadata.message_count;
    return node;
  }

  static bool decode(const Node & node, rosbag2_storage::FileInformation & metadata)
  {
    metadata.path = node["path"].as<std::string>();
    metadata.starting_time =
      node["starting_time"].as<std::chrono::time_point<std::chrono::high_resolution_clock>>();
    metadata.duration = node["duration"].as<std::chrono::nanoseconds>();
    metadata.message_count = node["message_count"].as<uint64_t>();
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
    time_in_ns = std::chrono::nanoseconds(node["nanoseconds"].as<uint64_t>());
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
      std::chrono::nanoseconds(node["nanoseconds_since_epoch"].as<uint64_t>()));
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
    node["duration"] = metadata.duration;
    node["starting_time"] = metadata.starting_time;
    node["message_count"] = metadata.message_count;
    node["topics_with_message_count"] =
      convert<std::vector<rosbag2_storage::TopicInformation>>::encode(
      metadata.topics_with_message_count, metadata.version);
    node["compression_format"] = metadata.compression_format;
    node["compression_mode"] = metadata.compression_mode;
    node["relative_file_paths"] = metadata.relative_file_paths;
    node["files"] = metadata.files;
    node["custom_data"] = metadata.custom_data;
    node["ros_distro"] = metadata.ros_distro;
    return node;
  }

  static bool decode(const Node & node, rosbag2_storage::BagMetadata & metadata)
  {
    metadata.version = node["version"].as<int>();
    metadata.storage_identifier = node["storage_identifier"].as<std::string>();
    metadata.duration = node["duration"].as<std::chrono::nanoseconds>();
    metadata.starting_time = node["starting_time"]
      .as<std::chrono::time_point<std::chrono::high_resolution_clock>>();
    metadata.message_count = node["message_count"].as<uint64_t>();
    metadata.topics_with_message_count =
      decode_for_version<std::vector<rosbag2_storage::TopicInformation>>(
      node["topics_with_message_count"], metadata.version);

    metadata.relative_file_paths = node["relative_file_paths"].as<std::vector<std::string>>();

    if (metadata.version >= 3) {  // fields introduced by rosbag2_compression
      metadata.compression_format = node["compression_format"].as<std::string>();
      metadata.compression_mode = node["compression_mode"].as<std::string>();
    }
    if (metadata.version >= 5) {
      metadata.files =
        node["files"].as<std::vector<rosbag2_storage::FileInformation>>();
    }

    if (metadata.version >= 6) {
      metadata.custom_data = node["custom_data"].as<std::unordered_map<std::string, std::string>>();
    }
    if (metadata.version >= 8) {
      metadata.ros_distro = node["ros_distro"].as<std::string>();
    }
    return true;
  }
};

}  // namespace YAML

#endif  // ROSBAG2_STORAGE__YAML_HPP_
