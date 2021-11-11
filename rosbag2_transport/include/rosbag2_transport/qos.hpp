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

#include <string>
#include <vector>

#include "rclcpp/node_interfaces/node_graph_interface.hpp"
#include "rclcpp/qos.hpp"

#include "rosbag2_transport/visibility_control.hpp"

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
  ROSBAG2_TRANSPORT_PUBLIC
  Rosbag2QoS()
  : rclcpp::QoS(rmw_qos_profile_default.depth) {}

  ROSBAG2_TRANSPORT_PUBLIC
  explicit Rosbag2QoS(const rclcpp::QoS & value)
  : rclcpp::QoS(value) {}

  ROSBAG2_TRANSPORT_PUBLIC
  Rosbag2QoS & default_history()
  {
    keep_last(rmw_qos_profile_default.depth);
    return *this;
  }

  // Create an adaptive QoS profile to use for subscribing to a set of offers from publishers.
  /**
    * - Uses rosbag2_transport defaults for History since they do not affect compatibility.
    * - Adapts Durability and Reliability, falling back to the least strict publisher when there
    * is a mixed offer. This behavior errs on the side of forming connections with all publishers.
    * - Does not specify Lifespan, Deadline, or Liveliness to be maximally compatible, because
    * these policies do not affect message delivery.
    */
  ROSBAG2_TRANSPORT_PUBLIC
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
  ROSBAG2_TRANSPORT_PUBLIC
  static Rosbag2QoS adapt_offer_to_recorded_offers(
    const std::string & topic_name,
    const std::vector<Rosbag2QoS> & profiles);
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
