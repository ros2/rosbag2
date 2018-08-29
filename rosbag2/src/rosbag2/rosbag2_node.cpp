// Copyright 2018, Bosch Software Innovations GmbH.
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

#include "rosbag2_node.hpp"

#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "typesupport_helpers.hpp"

namespace rosbag2
{

Rosbag2Node::Rosbag2Node(const std::string & node_name)
: rclcpp::Node(node_name)
{}

std::shared_ptr<GenericPublisher> Rosbag2Node::create_generic_publisher(
  const std::string & topic, const std::string & type)
{
  auto type_support = get_typesupport(type);
  return std::make_shared<GenericPublisher>(get_node_base_interface().get(), topic, *type_support);
}

std::shared_ptr<GenericSubscription> Rosbag2Node::create_generic_subscription(
  const std::string & topic,
  const std::string & type,
  std::function<void(std::shared_ptr<rmw_serialized_message_t>)> callback)
{
  auto type_support = get_typesupport(type);

  auto subscription = std::shared_ptr<GenericSubscription>();

  try {
    subscription = std::make_shared<GenericSubscription>(
      get_node_base_interface()->get_shared_rcl_node_handle(),
      *type_support,
      topic,
      callback);

    get_node_topics_interface()->add_subscription(subscription, nullptr);
  } catch (const std::runtime_error & ex) {
    RCUTILS_LOG_ERROR_NAMED(
      "rosbag2", "Error subscribing to topic %s. Error: %s", topic.c_str(), ex.what());
  }

  return subscription;
}
std::map<std::string, std::string> Rosbag2Node::get_topics_with_types(
  const std::vector<std::string> & topic_names)
{
  std::vector<std::string> sanitized_topic_names;
  std::transform(topic_names.begin(), topic_names.end(), std::back_inserter(sanitized_topic_names),
    [](std::string topic_name) {
      return topic_name[0] != '/' ? "/" + topic_name : topic_name;
    });

  // TODO(Martin-Idel-SI): This is a short sleep to allow the node some time to discover the topic
  // This should be replaced by an auto-discovery system in the future
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  auto topics_and_types = this->get_topic_names_and_types();

  std::map<std::string, std::vector<std::string>> filtered_topics_and_types;
  std::remove_copy_if(topics_and_types.begin(), topics_and_types.end(),
    std::inserter(filtered_topics_and_types, filtered_topics_and_types.end()),
    [sanitized_topic_names](auto element) {
      return std::find(sanitized_topic_names.begin(), sanitized_topic_names.end(), element.first) ==
      sanitized_topic_names.end();
    });

  return sanitize_topics_and_types(filtered_topics_and_types);
}

std::map<std::string, std::string>
Rosbag2Node::get_all_topics_with_types()
{
  // TODO(Martin-Idel-SI): This is a short sleep to allow the node some time to discover the topic
  // This should be replaced by an auto-discovery system in the future
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  return sanitize_topics_and_types(this->get_topic_names_and_types());
}

std::map<std::string, std::string> Rosbag2Node::sanitize_topics_and_types(
  std::map<std::string, std::vector<std::string>> topics_and_types)
{
  std::map<std::string, std::vector<std::string>> filtered_topics_and_types;
  std::remove_copy_if(topics_and_types.begin(),
    topics_and_types.end(),
    std::inserter(filtered_topics_and_types, filtered_topics_and_types.end()),
    [](auto element) {
      if (element.second.size() > 1) {
        RCUTILS_LOG_ERROR_NAMED(
          "rosbag2",
          "Topic '%s' has several types associated. Only topics with one type are supported.",
          element.first.c_str());
        return true;
      } else {
        char type_separator = '/';
        auto sep_position_back = element.second[0].find_last_of(type_separator);
        auto sep_position_front = element.second[0].find_first_of(type_separator);
        if (sep_position_back == std::string::npos ||
        sep_position_back != sep_position_front ||
        sep_position_back == 0 ||
        sep_position_back == element.second[0].length() - 1)
        {
          RCUTILS_LOG_ERROR_NAMED(
            "rosbag2",
            "Topic '%s' has non-ROS type %s . Only ROS topics are supported.",
            element.first.c_str(), element.second[0].c_str());
          return true;
        }
        return false;
      }
    });

  std::map<std::string, std::string> topics_and_types_to_record;
  std::transform(
    filtered_topics_and_types.begin(),
    filtered_topics_and_types.end(),
    std::inserter(topics_and_types_to_record, topics_and_types_to_record.end()),
    [](auto element) {
      return std::make_pair(element.first, element.second[0]);
    });
  return topics_and_types_to_record;
}

}  // namespace rosbag2
