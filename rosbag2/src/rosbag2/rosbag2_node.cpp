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

#include "rosbag2/logging.hpp"
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
    ROSBAG2_LOG_ERROR_STREAM("Error subscribing to topic '" << topic << "'. Error: " << ex.what());
  }

  return subscription;
}
std::map<std::string, std::string> Rosbag2Node::get_topics_with_types(
  const std::vector<std::string> & topic_names)
{
  std::vector<std::string> sanitized_topic_names;
  for (const auto & topic_name : topic_names) {
    sanitized_topic_names.push_back(topic_name[0] != '/' ? "/" + topic_name : topic_name);
  }

  // TODO(Martin-Idel-SI): This is a short sleep to allow the node some time to discover the topic
  // This should be replaced by an auto-discovery system in the future
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  auto topics_and_types = this->get_topic_names_and_types();

  std::map<std::string, std::vector<std::string>> filtered_topics_and_types;
  for (const auto & topic_and_type : topics_and_types) {
    if (std::find(sanitized_topic_names.begin(), sanitized_topic_names.end(),
      topic_and_type.first) != sanitized_topic_names.end())
    {
      filtered_topics_and_types.insert(topic_and_type);
    }
  }

  return reduce_multiple_types_to_one(filter_topics_with_wrong_types(filtered_topics_and_types));
}

std::map<std::string, std::string>
Rosbag2Node::get_all_topics_with_types()
{
  // TODO(Martin-Idel-SI): This is a short sleep to allow the node some time to discover the topic
  // This should be replaced by an auto-discovery system in the future
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  return reduce_multiple_types_to_one(
    filter_topics_with_wrong_types(this->get_topic_names_and_types()));
}

bool type_is_of_incorrect_form(const std::string & type)
{
  char type_separator = '/';
  auto sep_position_back = type.find_last_of(type_separator);
  auto sep_position_front = type.find_first_of(type_separator);
  return sep_position_back == std::string::npos ||
         sep_position_back != sep_position_front ||
         sep_position_back == 0 ||
         sep_position_back == type.length() - 1;
}

std::map<std::string, std::vector<std::string>> Rosbag2Node::filter_topics_with_wrong_types(
  std::map<std::string, std::vector<std::string>> topics_and_types)
{
  std::map<std::string, std::vector<std::string>> filtered_topics_and_types;
  for (const auto & topic_and_type : topics_and_types) {
    if (topic_and_type.second.size() > 1) {
      ROSBAG2_LOG_ERROR_STREAM("Topic '" << topic_and_type.first <<
        "' has several types associated. Only topics with one type are supported");
    } else if (type_is_of_incorrect_form(topic_and_type.second[0])) {
      ROSBAG2_LOG_ERROR_STREAM("Topic '" << topic_and_type.first << "' has non-ROS type '" <<
        topic_and_type.second[0] << "'. Only ROS topics are supported.");
    } else {
      filtered_topics_and_types.insert(topic_and_type);
    }
  }
  return filtered_topics_and_types;
}

std::map<std::string, std::string> Rosbag2Node::reduce_multiple_types_to_one(
  std::map<std::string, std::vector<std::string>> topics_and_types)
{
  std::map<std::string, std::string> topics_and_types_to_record;
  for (const auto & topic_and_types : topics_and_types) {
    topics_and_types_to_record.insert({topic_and_types.first, topic_and_types.second[0]});
  }
  return topics_and_types_to_record;
}

}  // namespace rosbag2
