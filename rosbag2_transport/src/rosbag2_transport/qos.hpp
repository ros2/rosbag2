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

#ifndef ROSBAG2_TRANSPORT__QOS_HPP_
#define ROSBAG2_TRANSPORT__QOS_HPP_

#include "rclcpp/qos.hpp"

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

namespace rosbag2_transport
{
class Rosbag2QoS : public rclcpp::QoS
{
public:
  explicit Rosbag2QoS() : rclcpp::QoS(10) {}
  explicit Rosbag2QoS(const rclcpp::QoS & other) : rclcpp::QoS(other) {}
};
}  // namespace rosbag2_transport


namespace YAML
{
template<>
struct convert<rmw_time_t>
{
  static Node encode(const rmw_time_t & time)
  {
    Node node;
    node["sec"] = time.sec;
    node["nsec"] = time.nsec;
    return node;
  }
  static bool decode(const Node & node, rmw_time_t & time)
  {
    time.sec = node["sec"].as<uint>();
    time.nsec = node["nsec"].as<uint>();
    return true;
  }
};

template<>
struct convert<rosbag2_transport::Rosbag2QoS>
{
  static Node encode(const rosbag2_transport::Rosbag2QoS & qos)
  {
    const auto & p = qos.get_rmw_qos_profile();
    Node node;
    node["history"] = (int)p.history;
    node["depth"] = p.depth;
    node["reliability"] = (int)p.reliability;
    node["durability"] = (int)p.durability;
    node["deadline"] = p.deadline;
    node["lifespan"] = p.lifespan;
    node["liveliness"] = (int)p.liveliness;
    node["liveliness_lease_duration"] = p.liveliness_lease_duration;
    node["avoid_ros_namespace_conventions"] = p.avoid_ros_namespace_conventions;
    return node;
  }
  static bool decode(const Node & node, rosbag2_transport::Rosbag2QoS & qos)
  {
    qos
      .keep_last(node["depth"].as<int>())
      .history((rmw_qos_history_policy_t)node["history"].as<int>())
      .reliability((rmw_qos_reliability_policy_t)node["reliability"].as<int>())
      .durability((rmw_qos_durability_policy_t)node["durability"].as<int>())
      .deadline(node["deadline"].as<rmw_time_t>())
      .lifespan(node["lifespan"].as<rmw_time_t>())
      .liveliness((rmw_qos_liveliness_policy_t)node["liveliness"].as<int>())
      .liveliness_lease_duration(node["liveliness_lease_duration"].as<rmw_time_t>())
      .avoid_ros_namespace_conventions(node["avoid_ros_namespace_conventions"].as<bool>())
    ;
    return true;
  }
};
}  // namespace YAML


#endif
