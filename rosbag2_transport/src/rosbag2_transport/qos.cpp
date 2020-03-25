#include "qos.hpp"

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
  time.sec = node["sec"].as<uint>();
  time.nsec = node["nsec"].as<uint>();
  return true;
}

Node convert<rosbag2_transport::Rosbag2QoS>::encode(const rosbag2_transport::Rosbag2QoS & qos)
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

bool convert<rosbag2_transport::Rosbag2QoS>::decode(
  const Node & node, rosbag2_transport::Rosbag2QoS & qos)
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
}  // namespace YAML
