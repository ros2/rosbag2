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

#include "rcl/expand_topic_name.h"

#include "rosbag2_cpp/typesupport_helpers.hpp"

#include "rosbag2_transport/logging.hpp"

namespace rosbag2_transport
{

Rosbag2Node::Rosbag2Node(const std::string & node_name)
: rclcpp::Node(node_name)
{}

std::shared_ptr<GenericPublisher> Rosbag2Node::create_generic_publisher(
  const std::string & topic, const std::string & type)
{
  auto type_support = rosbag2_cpp::get_typesupport(type, "rosidl_typesupport_cpp");
  return std::make_shared<GenericPublisher>(get_node_base_interface().get(), topic, *type_support);
}

std::shared_ptr<GenericSubscription> Rosbag2Node::create_generic_subscription(
  const std::string & topic,
  const std::string & type,
  std::function<void(std::shared_ptr<rmw_serialized_message_t>)> callback)
{
  auto type_support = rosbag2_cpp::get_typesupport(type, "rosidl_typesupport_cpp");

  auto subscription = std::shared_ptr<GenericSubscription>();

  try {
    subscription = std::make_shared<GenericSubscription>(
      get_node_base_interface().get(),
      *type_support,
      topic,
      callback);

    get_node_topics_interface()->add_subscription(subscription, nullptr);
  } catch (const std::runtime_error & ex) {
    ROSBAG2_TRANSPORT_LOG_ERROR_STREAM(
      "Error subscribing to topic '" << topic << "'. Error: " << ex.what());
  }

  return subscription;
}

std::shared_ptr<rcutils_string_map_t> get_initialized_string_map()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  auto substitutions_map = new rcutils_string_map_t;
  *substitutions_map = rcutils_get_zero_initialized_string_map();
  rcutils_ret_t map_init = rcutils_string_map_init(substitutions_map, 0, allocator);
  if (map_init != RCUTILS_RET_OK) {
    ROSBAG2_TRANSPORT_LOG_ERROR("Failed to initialize string map within rcutils.");
    return std::shared_ptr<rcutils_string_map_t>();
  }
  return std::shared_ptr<rcutils_string_map_t>(
    substitutions_map,
    [](rcutils_string_map_t * map) {
      rcl_ret_t cleanup = rcutils_string_map_fini(map);
      delete map;
      if (cleanup != RCL_RET_OK) {
        ROSBAG2_TRANSPORT_LOG_ERROR("Failed to deallocate string map when expanding topic.");
      }
    });
}

std::string Rosbag2Node::expand_topic_name(const std::string & topic_name)
{
  rcl_allocator_t allocator = rcl_get_default_allocator();
  auto substitutions_map = get_initialized_string_map();
  if (!substitutions_map) {
    ROSBAG2_TRANSPORT_LOG_ERROR("Failed to initialize string map within rcutils.");
    return "";
  }
  rcl_ret_t ret = rcl_get_default_topic_name_substitutions(substitutions_map.get());
  if (ret != RCL_RET_OK) {
    ROSBAG2_TRANSPORT_LOG_ERROR("Failed to initialize map with default values.");
    return "";
  }
  char * expanded_topic_name = nullptr;
  ret = rcl_expand_topic_name(
    topic_name.c_str(),
    get_name(),
    get_namespace(),
    substitutions_map.get(),
    allocator,
    &expanded_topic_name);

  if (ret != RCL_RET_OK) {
    ROSBAG2_TRANSPORT_LOG_ERROR_STREAM(
      "Failed to expand topic name " << topic_name << " with error: " <<
        rcutils_get_error_string().str);
    return "";
  }
  std::string expanded_topic_name_std(expanded_topic_name);
  allocator.deallocate(expanded_topic_name, allocator.state);
  return expanded_topic_name_std;
}

TopicNamesToTypes Rosbag2Node::get_topics_with_types(
  const std::vector<std::string> & topic_names)
{
  std::vector<std::string> sanitized_topic_names;
  for (const auto & topic_name : topic_names) {
    auto sanitized_topic_name = expand_topic_name(topic_name);
    if (!sanitized_topic_name.empty()) {
      sanitized_topic_names.push_back(sanitized_topic_name);
    }
  }

  auto topics_and_types = this->get_topic_names_and_types();

  std::map<std::string, std::vector<std::string>> filtered_topics_and_types;
  for (const auto & topic_and_type : topics_and_types) {
    if (
      std::find(
        sanitized_topic_names.begin(), sanitized_topic_names.end(),
        topic_and_type.first) != sanitized_topic_names.end())
    {
      filtered_topics_and_types.insert(topic_and_type);
    }
  }

  return filter_topics_with_more_than_one_type(filtered_topics_and_types);
}

TopicNamesToTypes Rosbag2Node::get_all_topics_with_types()
{
  return filter_topics_with_more_than_one_type(this->get_topic_names_and_types());
}

TopicNamesToTypes Rosbag2Node::filter_topics_with_more_than_one_type(
  std::map<std::string, std::vector<std::string>> topics_and_types)
{
  TopicNamesToTypes filtered_topics_and_types;
  for (const auto & topic_and_type : topics_and_types) {
    if (topic_and_type.second.size() > 1) {
      ROSBAG2_TRANSPORT_LOG_ERROR_STREAM(
        "Topic '" << topic_and_type.first <<
          "' has several types associated. Only topics with one type are supported");
    } else {
      filtered_topics_and_types.insert({topic_and_type.first, topic_and_type.second[0]});
    }
  }
  return filtered_topics_and_types;
}

}  // namespace rosbag2_transport
