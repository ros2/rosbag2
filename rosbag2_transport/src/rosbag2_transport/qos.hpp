
#ifndef ROSBAG2_TRANSPORT__QOS_HPP_
#define ROSBAG2_TRANSPORT__QOS_HPP_

#include "rclcpp/qos.hpp"
#include "enum.h"
#ifdef _WIN2
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
  // explicit Rosbag2QoS(const Rosbag2QoS & other) : rclcpp::QoS(other) {}
};

BETTER_ENUM(Reliability, int, SYSTEM_DEFAULT, RELIABLE, BEST_EFFORT, UNKNOWN)
BETTER_ENUM(History, int, SYSTEM_DEFAULT, KEEP_LAST, KEEP_ALL, UNKNOWN)
BETTER_ENUM(Durability, int, SYSTEM_DEFAULT, TRANSIENT_LOCAL, VOLATILE, UNKNOWN)
BETTER_ENUM(Liveliness, int, SYSTEM_DEFAULT, AUTOMATIC, MANUAL_BY_NODE, MANUAL_BY_TOPIC, UNKNOWN)


std::ostream& operator<<(std::ostream& os, rmw_time_t time);
std::ostream& operator<<(std::ostream& os, const rclcpp::QoS& qos);
bool operator==(const rmw_time_t & left, const rmw_time_t & right);
bool operator==(const rclcpp::QoS& left, const rclcpp::QoS & right);

}


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
