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

#include "rosbag2_transport/logging.hpp"

#include "qos.hpp"
#include "./rmw_time.h"

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
    // [foxy compatibility] Though we've backported rmw/time.c into this library
    // the Foxy RMW layers don't know what to do with the RMW_DURATION_INFINITE value,
    // just leave it unspecified for the same behavior.
    time = RMW_DURATION_UNSPECIFIED;
  }
  return true;
}

Node convert<rosbag2_transport::Rosbag2QoS>::encode(const rosbag2_transport::Rosbag2QoS & qos)
{
  const auto & p = qos.get_rmw_qos_profile();
  Node node;
  node["history"] = static_cast<int>(p.history);
  node["depth"] = p.depth;
  node["reliability"] = static_cast<int>(p.reliability);
  node["durability"] = static_cast<int>(p.durability);
  node["deadline"] = p.deadline;
  node["lifespan"] = p.lifespan;
  node["liveliness"] = static_cast<int>(p.liveliness);
  node["liveliness_lease_duration"] = p.liveliness_lease_duration;
  node["avoid_ros_namespace_conventions"] = p.avoid_ros_namespace_conventions;
  return node;
}

bool convert<rosbag2_transport::Rosbag2QoS>::decode(
  const Node & node, rosbag2_transport::Rosbag2QoS & qos)
{
  auto history = static_cast<rmw_qos_history_policy_t>(node["history"].as<int>());
  auto reliability = static_cast<rmw_qos_reliability_policy_t>(node["reliability"].as<int>());
  auto durability = static_cast<rmw_qos_durability_policy_t>(node["durability"].as<int>());
  auto liveliness = static_cast<rmw_qos_liveliness_policy_t>(node["liveliness"].as<int>());

  qos
  .keep_last(node["depth"].as<int>())
  .history(history)
  .reliability(reliability)
  .durability(durability)
  .deadline(node["deadline"].as<rmw_time_t>())
  .lifespan(node["lifespan"].as<rmw_time_t>())
  .liveliness(liveliness)
  .liveliness_lease_duration(node["liveliness_lease_duration"].as<rmw_time_t>())
  .avoid_ros_namespace_conventions(node["avoid_ros_namespace_conventions"].as<bool>());
  return true;
}
}  // namespace YAML

namespace rosbag2_transport
{
Rosbag2QoS Rosbag2QoS::adapt_request_to_offers(
  const std::string & topic_name, const std::vector<rclcpp::TopicEndpointInfo> & endpoints)
{
  if (endpoints.empty()) {
    return Rosbag2QoS{};
  }
  size_t num_endpoints = endpoints.size();
  size_t reliability_reliable_endpoints_count = 0;
  size_t durability_transient_local_endpoints_count = 0;
  for (const auto & endpoint : endpoints) {
    const auto & profile = endpoint.qos_profile().get_rmw_qos_profile();
    if (profile.reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE) {
      reliability_reliable_endpoints_count++;
    }
    if (profile.durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL) {
      durability_transient_local_endpoints_count++;
    }
  }

  // We set policies in order as defined in rmw_qos_profile_t
  Rosbag2QoS request_qos{};
  // Policy: history, depth
  // History does not affect compatibility

  // Policy: reliability
  if (reliability_reliable_endpoints_count == num_endpoints) {
    request_qos.reliable();
  } else {
    if (reliability_reliable_endpoints_count > 0) {
      ROSBAG2_TRANSPORT_LOG_WARN_STREAM(
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
      ROSBAG2_TRANSPORT_LOG_WARN_STREAM(
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

  ROSBAG2_TRANSPORT_LOG_WARN_STREAM(
    "Not all original publishers on topic " << topic_name << " offered the same QoS profiles. "
      "Rosbag2 cannot yet choose an adapted profile to offer for this mixed case. "
      "Falling back to the rosbag2_transport default publisher offer.");
  return Rosbag2QoS{};
}
}  // namespace rosbag2_transport
