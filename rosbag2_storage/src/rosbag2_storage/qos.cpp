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

#include <string>
#include <vector>

#include "rosbag2_storage/qos.hpp"
#include "rosbag2_storage/logging.hpp"
#include "rmw/qos_string_conversions.h"

namespace
{
/**
 * The following constants were the "Inifinity" value returned by RMW implementations before
 * the introduction of RMW_DURATION_INFINITE and associated RMW fixes
 * RMW: https://github.com/ros2/rmw/pull/301
 * Fast-DDS: https://github.com/ros2/rmw_fastrtps/pull/515
 * CycloneDDS: https://github.com/ros2/rmw_cyclonedds/pull/288
 * RTI Connext: https://github.com/ros2/rmw_connext/pull/491
 *
 * These values exist in bags recorded in Foxy, they need to be translated to RMW_DURATION_INFINITE
 * to be consistently understood for playback.
 * With those values, if a bag is played back in a different implementation than it was recorded,
 * the publishers will fail to be created with an error indicating an invalid QoS value..
 */
static const rmw_time_t RMW_CYCLONEDDS_FOXY_INFINITE = rmw_time_from_nsec(0x7FFFFFFFFFFFFFFFll);
static const rmw_time_t RMW_FASTRTPS_FOXY_INFINITE {0x7FFFFFFFll, 0xFFFFFFFFll};
static const rmw_time_t RMW_CONNEXT_FOXY_INFINITE  {0x7FFFFFFFll, 0x7FFFFFFFll};
}  // namespace

namespace YAML
{

Node convert<rmw_qos_history_policy_t>::encode(const rmw_qos_history_policy_t & policy, int version)
{
  if (version < 9) {
    return Node(static_cast<int>(policy));
  }
  if (policy == RMW_QOS_POLICY_HISTORY_UNKNOWN) {
    return Node(std::string("unknown"));
  } else {
    return Node(std::string(rmw_qos_history_policy_to_str(policy)));
  }
}

bool convert<rmw_qos_history_policy_t>::decode(const Node & node, rmw_qos_history_policy_t & policy)
{
  policy = rmw_qos_history_policy_from_str(node.as<std::string>().c_str());
  return true;
}

Node convert<rmw_qos_reliability_policy_t>::encode(
  const rmw_qos_reliability_policy_t & policy,
  int version)
{
  if (version < 9) {
    return Node(static_cast<int>(policy));
  }
  if (policy == RMW_QOS_POLICY_RELIABILITY_UNKNOWN) {
    return Node(std::string("unknown"));
  } else {
    return Node(std::string(rmw_qos_reliability_policy_to_str(policy)));
  }
}

bool convert<rmw_qos_reliability_policy_t>::decode(
  const Node & node,
  rmw_qos_reliability_policy_t & policy)
{
  policy = rmw_qos_reliability_policy_from_str(node.as<std::string>().c_str());
  return true;
}

Node convert<rmw_qos_durability_policy_t>::encode(
  const rmw_qos_durability_policy_t & policy,
  int version)
{
  if (version < 9) {
    return Node(static_cast<int>(policy));
  }
  if (policy == RMW_QOS_POLICY_DURABILITY_UNKNOWN) {
    return Node(std::string("unknown"));
  } else {
    return Node(std::string(rmw_qos_durability_policy_to_str(policy)));
  }
}

bool convert<rmw_qos_durability_policy_t>::decode(
  const Node & node,
  rmw_qos_durability_policy_t & policy)
{
  policy = rmw_qos_durability_policy_from_str(node.as<std::string>().c_str());
  return true;
}

Node convert<rmw_qos_liveliness_policy_t>::encode(
  const rmw_qos_liveliness_policy_t & policy,
  int version)
{
  if (version < 9) {
    return Node(static_cast<int>(policy));
  }
  if (policy == RMW_QOS_POLICY_LIVELINESS_UNKNOWN) {
    return Node(std::string("unknown"));
  } else {
    return Node(std::string(rmw_qos_liveliness_policy_to_str(policy)));
  }
}

bool convert<rmw_qos_liveliness_policy_t>::decode(
  const Node & node,
  rmw_qos_liveliness_policy_t & policy)
{
  policy = rmw_qos_liveliness_policy_from_str(node.as<std::string>().c_str());
  return true;
}

Node convert<rmw_time_t>::encode(const rmw_time_t & time)
{
  Node node;
  node["sec"] = time.sec;
  node["nsec"] = time.nsec;
  return node;
}

bool convert<rmw_time_t>::decode(const Node & node, rmw_time_t & time)
{
  time.sec = node["sec"].as<uint64_t>();
  time.nsec = node["nsec"].as<uint64_t>();
  if (
    rmw_time_equal(time, RMW_CYCLONEDDS_FOXY_INFINITE) ||
    rmw_time_equal(time, RMW_FASTRTPS_FOXY_INFINITE) ||
    rmw_time_equal(time, RMW_CONNEXT_FOXY_INFINITE))
  {
    time = RMW_DURATION_INFINITE;
  }
  return true;
}

Node convert<rclcpp::QoS>::encode(const rclcpp::QoS & qos, int version)
{
  const auto & p = qos.get_rmw_qos_profile();
  Node node;
  node["history"] = convert<rmw_qos_history_policy_t>::encode(p.history, version);
  node["depth"] = p.depth;
  node["reliability"] = convert<rmw_qos_reliability_policy_t>::encode(p.reliability, version);
  node["durability"] = convert<rmw_qos_durability_policy_t>::encode(p.durability, version);
  node["deadline"] = p.deadline;
  node["lifespan"] = p.lifespan;
  node["liveliness"] = convert<rmw_qos_liveliness_policy_t>::encode(p.liveliness, version);
  node["liveliness_lease_duration"] = p.liveliness_lease_duration;
  node["avoid_ros_namespace_conventions"] = p.avoid_ros_namespace_conventions;
  return node;
}

bool convert<rclcpp::QoS>::decode(const Node & node, rclcpp::QoS & qos, int version)
{
  rmw_qos_history_policy_t history;
  rmw_qos_reliability_policy_t reliability;
  rmw_qos_durability_policy_t durability;
  rmw_qos_liveliness_policy_t liveliness;

  // Try to auto-detect qos serialization format
  int history_int = -1;
  if (convert<int>::decode(node["history"], history_int) &&
    history_int >= RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT &&
    history_int <= RMW_QOS_POLICY_HISTORY_UNKNOWN)
  {
    history = static_cast<rmw_qos_history_policy_t>(history_int);
    version = 8;
  } else {
    history = node["history"].as<rmw_qos_history_policy_t>();
    version = 9;
  }

  if (version <= 8) {
    reliability = static_cast<rmw_qos_reliability_policy_t>(node["reliability"].as<int>());
    durability = static_cast<rmw_qos_durability_policy_t>(node["durability"].as<int>());
    liveliness = static_cast<rmw_qos_liveliness_policy_t>(node["liveliness"].as<int>());
  } else {
    reliability = node["reliability"].as<rmw_qos_reliability_policy_t>();
    durability = node["durability"].as<rmw_qos_durability_policy_t>();
    liveliness = node["liveliness"].as<rmw_qos_liveliness_policy_t>();
  }

  if (history == RMW_QOS_POLICY_HISTORY_KEEP_LAST) {
    qos.keep_last(node["depth"].as<int>());
  } else {
    qos.history(history);
  }

  qos
  .reliability(reliability)
  .durability(durability)
  .deadline(node["deadline"].as<rmw_time_t>())
  .lifespan(node["lifespan"].as<rmw_time_t>())
  .liveliness(liveliness)
  .liveliness_lease_duration(node["liveliness_lease_duration"].as<rmw_time_t>())
  .avoid_ros_namespace_conventions(node["avoid_ros_namespace_conventions"].as<bool>());

  return true;
}

Node convert<rosbag2_storage::Rosbag2QoS>::encode(
  const rosbag2_storage::Rosbag2QoS & qos, int version)
{
  return convert<rclcpp::QoS>::encode(static_cast<rclcpp::QoS>(qos), version);
}

bool convert<rosbag2_storage::Rosbag2QoS>::decode(
  const Node & node, rosbag2_storage::Rosbag2QoS & qos, int version)
{
  return convert<rclcpp::QoS>::decode(node, qos, version);
}

Node convert<std::vector<rosbag2_storage::Rosbag2QoS>>::encode(
  const std::vector<rosbag2_storage::Rosbag2QoS> & rhs)
{
  Node node{NodeType::Sequence};
  for (const auto & value : rhs) {
    node.push_back(value);
  }
  return node;
}

bool convert<std::vector<rosbag2_storage::Rosbag2QoS>>::decode(
  const Node & node, std::vector<rosbag2_storage::Rosbag2QoS> & rhs, int version)
{
  if (!node.IsSequence()) {
    return false;
  }

  rhs.clear();
  for (const auto & value : node) {
    rhs.push_back(decode_for_version<rosbag2_storage::Rosbag2QoS>(value, version));
  }
  return true;
}

Node convert<std::vector<rclcpp::QoS>>::encode(const std::vector<rclcpp::QoS> & rhs, int version)
{
  Node node{NodeType::Sequence};
  for (const auto & value : rhs) {
    node.push_back(convert<rclcpp::QoS>::encode(value, version));
  }
  return node;
}

bool convert<std::vector<rclcpp::QoS>>::decode(
  const Node & node, std::vector<rclcpp::QoS> & rhs, int version)
{
  if (!node.IsSequence()) {
    return false;
  }

  rhs.clear();
  for (const auto & value : node) {
    // Using rosbag2_storage::Rosbag2QoS for decoding because rclcpp::QoS is not default
    // constructable. Note: It is safe to use upcast when adding to the vector<rclcpp::QoS>
    auto rosbag2_qos = decode_for_version<rosbag2_storage::Rosbag2QoS>(value, version);
    rhs.push_back(rosbag2_qos);
  }
  return true;
}

Node convert<std::unordered_map<std::string, rclcpp::QoS>>::encode(
  const std::unordered_map<std::string, rclcpp::QoS> & rhs)
{
  Node node{NodeType::Sequence};
  for (const auto & [key, value] : rhs) {
    node.force_insert(key, value);
  }
  return node;
}

bool convert<std::unordered_map<std::string, rclcpp::QoS>>::decode(
  const Node & node, std::unordered_map<std::string, rclcpp::QoS> & rhs, int version)
{
  if (!node.IsMap()) {
    return false;
  }

  rhs.clear();
  for (const auto & element : node) {
    // Using rosbag2_storage::Rosbag2QoS for decoding because rclcpp::QoS is not default
    // constructable. Note: It is safe to use upcast when inserting into the unordered_map
    rhs.insert(
      {element.first.as<std::string>(),
        decode_for_version<rosbag2_storage::Rosbag2QoS>(element.second, version)
      });
  }
  return true;
}
}  // namespace YAML

namespace rosbag2_storage
{

std::string Rosbag2QoS::to_string() const
{
  YAML::Node yaml_qos_node = YAML::convert<rosbag2_storage::Rosbag2QoS>::encode(*this);
  std::string serialized_qos = YAML::Dump(yaml_qos_node);
  return serialized_qos;
}

Rosbag2QoS Rosbag2QoS::adapt_request_to_offers(
  const std::string & topic_name, const std::vector<rclcpp::TopicEndpointInfo> & endpoints)
{
  if (endpoints.empty()) {
    return Rosbag2QoS{};
  }
  size_t num_endpoints = endpoints.size();
  size_t reliability_reliable_endpoints_count = 0;
  size_t durability_transient_local_endpoints_count = 0;
  size_t max_history_depth = 0;
  for (const auto & endpoint : endpoints) {
    const auto & profile = endpoint.qos_profile().get_rmw_qos_profile();
    if (profile.reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE) {
      reliability_reliable_endpoints_count++;
    }
    if (profile.durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL) {
      durability_transient_local_endpoints_count++;
    }
    if (profile.depth > max_history_depth) {
      max_history_depth = profile.depth;
    }
  }

  // We set policies in order as defined in rmw_qos_profile_t
  Rosbag2QoS request_qos{};
  // Policy: history, depth
  // History does not affect compatibility. However, it could affect messages drop on the DDS side.
  if (max_history_depth > 1) {
    request_qos.keep_last(max_history_depth);
  } else {
    if (max_history_depth == 1) {
      ROSBAG2_STORAGE_LOG_DEBUG_STREAM(
        "Publishers on topic \"" << topic_name << "\" are offering "
          "RMW_QOS_POLICY_HISTORY_KEEP_LAST with depth = 1. There is a high probability that "
          "messages will be dropped. Falling back to the RMW_QOS_POLICY_HISTORY_KEEP_ALL.");
    }
    request_qos.keep_all();
  }

  // Policy: reliability
  if (reliability_reliable_endpoints_count == num_endpoints) {
    request_qos.reliable();
  } else {
    if (reliability_reliable_endpoints_count > 0) {
      ROSBAG2_STORAGE_LOG_WARN_STREAM(
        "Some, but not all, publishers on topic \"" << topic_name << "\" "
          "are offering RMW_QOS_POLICY_RELIABILITY_RELIABLE. "
          "Falling back to RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT "
          "as it will connect to all publishers. "
          "Some messages from Reliable publishers could be dropped.");
    }
    request_qos.best_effort();
  }

  // Policy: durability
  // If all publishers offer transient_local, we can request it and receive latched messages
  if (durability_transient_local_endpoints_count == num_endpoints) {
    request_qos.transient_local();
  } else {
    if (durability_transient_local_endpoints_count > 0) {
      ROSBAG2_STORAGE_LOG_WARN_STREAM(
        "Some, but not all, publishers on topic \"" << topic_name << "\" "
          "are offering RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL. "
          "Falling back to RMW_QOS_POLICY_DURABILITY_VOLATILE "
          "as it will connect to all publishers. "
          "Previously-published latched messages will not be retrieved.");
    }
    request_qos.durability_volatile();
  }
  // Policy: deadline
  // Deadline does not affect delivery of messages,
  // and we do not record Deadline"Missed events.
  // We can always use unspecified deadline, which will be compatible with all publishers.

  // Policy: lifespan
  // Lifespan does not affect compatibiliy

  // Policy: liveliness, liveliness_lease_duration
  // Liveliness does not affect delivery of messages,
  // and we do not record LivelinessChanged events.
  // We can always use unspecified liveliness, which will be compatible with all publishers.
  return request_qos;
}

namespace
{
bool operator==(const rmw_time_t & lhs, const rmw_time_t & rhs)
{
  return lhs.sec == rhs.sec && lhs.nsec == rhs.nsec;
}

/** Check if all QoS profiles in the vector are identical when only looking at
  * policies that affect compatibility.
  * This means it excludes history and lifespan from the equality check.
  */
bool all_profiles_effectively_same(const std::vector<Rosbag2QoS> & profiles)
{
  auto iterator = profiles.begin();
  const auto p_ref = iterator->get_rmw_qos_profile();
  iterator++;
  for (; iterator != profiles.end(); iterator++) {
    const auto p_next = iterator->get_rmw_qos_profile();
    bool compatibility_equals_previous = (
      // excluding history
      p_ref.reliability == p_next.reliability &&
      p_ref.durability == p_next.durability &&
      p_ref.deadline == p_next.deadline &&
      // excluding lifespan
      p_ref.liveliness == p_next.liveliness &&
      p_ref.liveliness_lease_duration == p_next.liveliness_lease_duration
    );
    if (!compatibility_equals_previous) {
      return false;
    }
  }
  return true;
}
}  // unnamed namespace

Rosbag2QoS Rosbag2QoS::adapt_offer_to_recorded_offers(
  const std::string & topic_name, const std::vector<Rosbag2QoS> & profiles)
{
  if (profiles.empty()) {
    return Rosbag2QoS{};
  }
  if (all_profiles_effectively_same(profiles)) {
    auto result = profiles[0];
    return result.default_history();
  }

  ROSBAG2_STORAGE_LOG_WARN_STREAM(
    "Not all original publishers on topic " << topic_name << " offered the same QoS profiles. "
      "Rosbag2 cannot yet choose an adapted profile to offer for this mixed case. "
      "Falling back to the rosbag2_storage default publisher offer.");
  return Rosbag2QoS{};
}

std::vector<rosbag2_storage::Rosbag2QoS> from_rclcpp_qos_vector(const std::vector<rclcpp::QoS> & in)
{
  std::vector<rosbag2_storage::Rosbag2QoS> out;
  out.reserve(in.size());
  std::transform(
    in.begin(), in.end(), std::back_inserter(out),
    [](auto & qos) {return static_cast<rosbag2_storage::Rosbag2QoS>(qos);});
  return out;
}

std::string serialize_rclcpp_qos_vector(const std::vector<rclcpp::QoS> & in, int version)
{
  auto node = YAML::convert<std::vector<rclcpp::QoS>>::encode(in, version);
  return YAML::Dump(node);
}

std::vector<rclcpp::QoS> to_rclcpp_qos_vector(const std::string & serialized, int version)
{
  if (serialized.empty()) {return {};}
  auto node = YAML::Load(serialized);
  return YAML::decode_for_version<std::vector<rclcpp::QoS>>(node, version);
}

}  // namespace rosbag2_storage
