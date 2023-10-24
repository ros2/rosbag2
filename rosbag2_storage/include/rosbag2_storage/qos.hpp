// Copyright 2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

#ifndef ROSBAG2_STORAGE__QOS_HPP_
#define ROSBAG2_STORAGE__QOS_HPP_

#include <map>
#include <string>
#include <vector>

#include "rclcpp/node_interfaces/node_graph_interface.hpp"
#include "rclcpp/qos.hpp"
#include "yaml-cpp/yaml.h"


#include "rosbag2_storage/visibility_control.hpp"

namespace rosbag2_storage
{
/// Simple wrapper around rclcpp::QoS to provide a default constructor for YAML deserialization.
class ROSBAG2_STORAGE_PUBLIC Rosbag2QoS : public rclcpp::QoS
{
public:
  Rosbag2QoS()
  : rclcpp::QoS(rmw_qos_profile_default.depth) {}

  explicit Rosbag2QoS(const rclcpp::QoS & value)
  : rclcpp::QoS(value) {}

  Rosbag2QoS & default_history()
  {
    keep_last(rmw_qos_profile_default.depth);
    return *this;
  }

  // Create an adaptive QoS profile to use for subscribing to a set of offers from publishers.
  /**
    * - Uses rosbag2_storage defaults for History since they do not affect compatibility.
    * - Adapts Durability and Reliability, falling back to the least strict publisher when there
    * is a mixed offer. This behavior errs on the side of forming connections with all publishers.
    * - Does not specify Lifespan, Deadline, or Liveliness to be maximally compatible, because
    * these policies do not affect message delivery.
    */
  static Rosbag2QoS adapt_request_to_offers(
    const std::string & topic_name,
    const std::vector<rclcpp::TopicEndpointInfo> & endpoints);

  // Create a QoS profile to offer for playback.
  /**
    * This logic exists because rosbag2 does not record on a per-publisher basis, so we try to
    * get as close as possible to the original system's behavior, given a single publisher.
    * If all profiles are the same (excepting History & Lifespan, which are purely local),
    * that exact value is returned.
    * Otherwise, fall back to the rosbag2 default and emit a warning.
    */
  static Rosbag2QoS adapt_offer_to_recorded_offers(
    const std::string & topic_name,
    const std::vector<Rosbag2QoS> & profiles);
};
}  // namespace rosbag2_storage

namespace YAML
{

/// Pass metadata version to the sub-structs of BagMetadata for deserializing.
/**
  * Encoding should always use the current metadata version, so it does not need this value.
  * We cannot extend the YAML::Node class to include this, so we must call it
  * as a function with the node as an argument.
  */

template<typename T>
T decode_for_version(const Node & node, int version)
{
  static_assert(
    std::is_default_constructible<T>::value,
    "Type passed to decode_for_version that has is not default constructible.");
  if (!node.IsDefined()) {
    throw TypedBadConversion<T>(node.Mark());
  }
  T value{};
  if (convert<T>::decode(node, value, version)) {
    return value;
  }
  throw TypedBadConversion<T>(node.Mark());
}

template<>
struct ROSBAG2_STORAGE_PUBLIC convert<rmw_qos_history_policy_t>
{
  static Node encode(const rmw_qos_history_policy_t & policy);
  static bool decode(const Node & node, rmw_qos_history_policy_t & policy);
};

template<>
struct ROSBAG2_STORAGE_PUBLIC convert<rmw_qos_reliability_policy_t>
{
  static Node encode(const rmw_qos_reliability_policy_t & policy);
  static bool decode(const Node & node, rmw_qos_reliability_policy_t & policy);
};

template<>
struct ROSBAG2_STORAGE_PUBLIC convert<rmw_qos_durability_policy_t>
{
  static Node encode(const rmw_qos_durability_policy_t & policy);
  static bool decode(const Node & node, rmw_qos_durability_policy_t & policy);
};

template<>
struct ROSBAG2_STORAGE_PUBLIC convert<rmw_qos_liveliness_policy_t>
{
  static Node encode(const rmw_qos_liveliness_policy_t & policy);
  static bool decode(const Node & node, rmw_qos_liveliness_policy_t & policy);
};

template<>
struct ROSBAG2_STORAGE_PUBLIC convert<rmw_time_t>
{
  static Node encode(const rmw_time_t & time);
  static bool decode(const Node & node, rmw_time_t & time);
};

template<>
struct ROSBAG2_STORAGE_PUBLIC convert<rosbag2_storage::Rosbag2QoS>
{
  static Node encode(const rosbag2_storage::Rosbag2QoS & qos);
  static bool decode(const Node & node, rosbag2_storage::Rosbag2QoS & qos, int version);
};

template<>
struct ROSBAG2_STORAGE_PUBLIC convert<std::vector<rosbag2_storage::Rosbag2QoS>>
{
  static Node encode(const std::vector<rosbag2_storage::Rosbag2QoS> & rhs)
  {
    Node node{NodeType::Sequence};
    for (const auto & value : rhs) {
      node.push_back(value);
    }
    return node;
  }

  static bool decode(
    const Node & node, std::vector<rosbag2_storage::Rosbag2QoS> & rhs, int version)
  {
    if (!node.IsSequence()) {
      return false;
    }

    rhs.clear();
    for (const auto & value : node) {
      auto temp = decode_for_version<rosbag2_storage::Rosbag2QoS>(value, version);
      rhs.push_back(temp);
    }
    return true;
  }
};

template<>
struct ROSBAG2_STORAGE_PUBLIC convert<std::map<std::string, rosbag2_storage::Rosbag2QoS>>
{
  static Node encode(const std::map<std::string, rosbag2_storage::Rosbag2QoS> & rhs)
  {
    Node node{NodeType::Sequence};
    for (const auto & [key, value] : rhs) {
      node.force_insert(key, value);
    }
    return node;
  }

  static bool decode(
    const Node & node, std::map<std::string, rosbag2_storage::Rosbag2QoS> & rhs, int version)
  {
    if (!node.IsMap()) {
      return false;
    }

    rhs.clear();
    for (const auto & element : node) {
      auto temp = decode_for_version<rosbag2_storage::Rosbag2QoS>(element.second, version);
      rhs[element.first.as<std::string>()] = temp;
    }
    return true;
  }
};
}  // namespace YAML

#endif  // ROSBAG2_STORAGE__QOS_HPP_
