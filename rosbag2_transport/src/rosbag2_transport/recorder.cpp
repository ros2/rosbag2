// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#include "recorder.hpp"

#include <algorithm>
#include <future>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rosbag2_cpp/writer.hpp"

#include "rosbag2_transport/logging.hpp"

#include "enum.h"
#include "generic_subscription.hpp"
#include "rosbag2_node.hpp"
#include "types.hpp"

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
    time.sec = node["sec"].as<int>();
    time.nsec = node["nsec"].as<int>();
    return true;
  }
};

template<>
struct convert<rclcpp::QoS>
{
  static Node encode(const rclcpp::QoS & qos)
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
  static bool decode(const Node & node, rclcpp::QoS & qos)
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

namespace rosbag2_transport
{

BETTER_ENUM(Reliability, int, SYSTEM_DEFAULT, RELIABLE, BEST_EFFORT, UNKNOWN)
BETTER_ENUM(History, int, SYSTEM_DEFAULT, KEEP_LAST, KEEP_ALL, UNKNOWN)
BETTER_ENUM(Durability, int, SYSTEM_DEFAULT, TRANSIENT_LOCAL, VOLATILE, UNKNOWN)
BETTER_ENUM(Liveliness, int, SYSTEM_DEFAULT, AUTOMATIC, MANUAL_BY_NODE, MANUAL_BY_TOPIC, UNKNOWN)


std::ostream& operator<<(std::ostream& os, rmw_time_t time)
{
  os.precision(3);
  double ms = (time.sec * 1000L) + (time.nsec / 1000000.0);
  os << ms << " ms";
  return os;
}


std::ostream& operator<<(std::ostream& os, const rclcpp::QoS& qos)
{
  const auto & p = qos.get_rmw_qos_profile();
  os << "History: " << History::_from_integral(p.history) << " (" << p.depth << ")" << std::endl;
  os << "Reliability: " << Reliability::_from_integral(p.reliability) << std::endl;
  os << "Durability: " << Durability::_from_integral(p.durability) << std::endl;
  os << "Deadline: " << p.deadline << std::endl;
  os << "Lifespan: " << p.lifespan << std::endl;
  os << "Liveliness: " << Liveliness::_from_integral(p.liveliness)
     << " (" << p.liveliness_lease_duration << ")" << std::endl;
  return os;
}

Recorder::Recorder(std::shared_ptr<rosbag2_cpp::Writer> writer, std::shared_ptr<Rosbag2Node> node)
: writer_(std::move(writer)), node_(std::move(node)) {}

void Recorder::record(const RecordOptions & record_options)
{
  if (record_options.rmw_serialization_format.empty()) {
    throw std::runtime_error("No serialization format specified!");
  }
  serialization_format_ = record_options.rmw_serialization_format;
  ROSBAG2_TRANSPORT_LOG_INFO("Listening for topics...");
  subscribe_topics(get_requested_or_available_topics(record_options.topics));

  std::future<void> discovery_future;
  if (!record_options.is_discovery_disabled) {
    auto discovery = std::bind(
      &Recorder::topics_discovery, this,
      record_options.topic_polling_interval, record_options.topics);
    discovery_future = std::async(std::launch::async, discovery);
  }

  record_messages();

  if (discovery_future.valid()) {
    discovery_future.wait();
  }

  subscriptions_.clear();
}

void Recorder::topics_discovery(
  std::chrono::milliseconds topic_polling_interval,
  const std::vector<std::string> & requested_topics)
{
  while (rclcpp::ok()) {
    auto topics_to_subscribe = get_requested_or_available_topics(requested_topics);
    auto missing_topics = get_missing_topics(topics_to_subscribe);
    subscribe_topics(missing_topics);

    if (!requested_topics.empty() && subscribed_topics_.size() == requested_topics.size()) {
      ROSBAG2_TRANSPORT_LOG_INFO("All requested topics are subscribed. Stopping discovery...");
      return;
    }
    std::this_thread::sleep_for(topic_polling_interval);
  }
}

TopicsMap
Recorder::get_requested_or_available_topics(const std::vector<std::string> & requested_topics)
{
  return requested_topics.empty() ?
         node_->get_all_topics_with_types() :
         node_->get_topics_with_types(requested_topics);
}

TopicsMap Recorder::get_missing_topics(const TopicsMap & topics)
{
  TopicsMap missing_topics;
  for (const auto & i : topics) {
    if (subscribed_topics_.find(i.first) == subscribed_topics_.end()) {
      missing_topics.emplace(i.first, i.second);
    }
  }
  return missing_topics;
}

void Recorder::subscribe_topics(const TopicsMap & topics_and_types)
{
  for (const auto & topic_with_type : topics_and_types) {
    subscribe_topic(topic_with_type.first, topic_with_type.second, serialization_format_);
  }
}

void Recorder::subscribe_topic(std::string name, std::string type, std::string serialization_format)
{
  rosbag2_storage::TopicMetadata topic {name, type, serialization_format, {}};

  auto publishers_info = node_->get_publishers_info_by_topic(topic.name);
  ROSBAG2_TRANSPORT_LOG_ERROR_STREAM("Endpoints for topic " << topic.name);
  for (auto info : publishers_info) {
    YAML::Node profile_node;
    profile_node = info.qos_profile();
    std::string serialized = YAML::Dump(profile_node);

    ROSBAG2_TRANSPORT_LOG_ERROR_STREAM("  Node " << info.node_namespace() << "/" << info.node_name());
    ROSBAG2_TRANSPORT_LOG_ERROR_STREAM("  QoS " << info.qos_profile());
    topic.offered_qos_profiles.push_back(serialized);
  }

  // Need to create topic in writer before we are trying to create subscription. Since in
  // callback for subscription we are calling writer_->write(bag_message); and it could happened
  // that callback called before we reached out the line: writer_->create_topic(topic)
  writer_->create_topic(topic);
  auto subscription = create_subscription(topic.name, topic.type);
  if (subscription) {
    subscribed_topics_.insert(topic.name);
    subscriptions_.push_back(subscription);
    ROSBAG2_TRANSPORT_LOG_INFO_STREAM("Subscribed to topic '" << topic.name << "'");
  } else {
    writer_->remove_topic(topic);
    subscribed_topics_.erase(topic.name);
  }
}

std::shared_ptr<GenericSubscription>
Recorder::create_subscription(
  const std::string & topic_name, const std::string & topic_type)
{
  auto subscription = node_->create_generic_subscription(
    topic_name,
    topic_type,
    [this, topic_name](std::shared_ptr<rmw_serialized_message_t> message) {
      auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
      bag_message->serialized_data = message;
      bag_message->topic_name = topic_name;
      rcutils_time_point_value_t time_stamp;
      int error = rcutils_system_time_now(&time_stamp);
      if (error != RCUTILS_RET_OK) {
        ROSBAG2_TRANSPORT_LOG_ERROR_STREAM(
          "Error getting current time. Error:" << rcutils_get_error_string().str);
      }
      bag_message->time_stamp = time_stamp;

      writer_->write(bag_message);
    });
  return subscription;
}

void Recorder::record_messages() const
{
  spin(node_);
}

}  // namespace rosbag2_transport
