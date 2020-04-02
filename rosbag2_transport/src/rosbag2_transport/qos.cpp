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

#include "qos.hpp"

namespace rosbag2_transport
{
std::ostream & operator<<(std::ostream & stream, const Rosbag2QoS & qos)
{
  const auto profile = qos.get_rmw_qos_profile();
  stream << "history: " << profile.history << std::endl <<
    "depth: " << profile.depth << std::endl <<
    "reliability: " << profile.reliability << std::endl <<
    "durability: " << profile.durability << std::endl <<
    "deadline: " << profile.deadline.sec << "." << profile.deadline.nsec << std::endl <<
    "lifespan: " << profile.lifespan.sec << "." << profile.lifespan.nsec << std::endl <<
    "liveliness: " << profile.liveliness << std::endl <<
    "liveliness_lease_duration: " << profile.liveliness_lease_duration.sec << "." <<
    profile.liveliness_lease_duration.nsec << std::endl <<
    "avoid_ros_namespace_conventions: " << profile.avoid_ros_namespace_conventions << std::endl;
  return stream;
}
}

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
