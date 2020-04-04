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
/// Simple wrapper around rclcpp::QoS to provide a default constructor for YAML deserialization.
class Rosbag2QoS : public rclcpp::QoS
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

  /// Determine if all policies that affect QoS compatibility are equal.
  /*
    Note that this method does not check if the QoS policies are compatible.
    This is a simple check for exact equality in all policies that affect compatibility.
  */
  bool compatibility_policies_exactly_equal(const rclcpp::QoS & other) const;
};
}  // namespace rosbag2_transport

namespace YAML
{
template<>
struct convert<rmw_time_t>
{
  static Node encode(const rmw_time_t & time);
  static bool decode(const Node & node, rmw_time_t & time);
};

template<>
struct convert<rosbag2_transport::Rosbag2QoS>
{
  static Node encode(const rosbag2_transport::Rosbag2QoS & qos);
  static bool decode(const Node & node, rosbag2_transport::Rosbag2QoS & qos);
};
}  // namespace YAML

#endif  // ROSBAG2_TRANSPORT__QOS_HPP_
