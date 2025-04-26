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

#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "rmw/rmw.h"
#include "rosbag2_cpp/action_utils.hpp"
#include "rosbag2_cpp/info.hpp"
#include "rosbag2_cpp/service_utils.hpp"
#include "rosbag2_storage/metadata_io.hpp"
#include "rosbag2_storage/storage_factory.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "service_msgs/msg/service_event_info.hpp"

namespace fs = std::filesystem;

namespace rosbag2_cpp
{

rosbag2_storage::BagMetadata Info::read_metadata(
  const std::string & uri, const std::string & storage_id)
{
  const fs::path bag_path{uri};
  if (!fs::exists(bag_path)) {
    throw std::runtime_error("Bag path " + uri + " does not exist.");
  }

  rosbag2_storage::MetadataIo metadata_io;
  if (metadata_io.metadata_file_exists(uri)) {
    return metadata_io.read_metadata(uri);
  }

  if (fs::is_directory(bag_path)) {
    throw std::runtime_error("Could not find metadata in bag directory " + uri);
  }

  rosbag2_storage::StorageFactory factory;
  auto storage = factory.open_read_only({uri, storage_id});
  if (!storage) {
    throw std::runtime_error("No plugin detected that could open file " + uri);
  }
  return storage->get_metadata();
}

namespace
{
using client_id = service_msgs::msg::ServiceEventInfo::_client_gid_type;
using sequence_set = std::unordered_set<int64_t>;
struct service_req_resp_info
{
  std::unordered_map<client_id, sequence_set, client_id_hash> request;
  std::unordered_map<client_id, sequence_set, client_id_hash> response;
};

struct action_service_req_resp_info
{
  service_req_resp_info send_goal_service;
  service_req_resp_info cancel_goal_service;
  service_req_resp_info get_result_service;
};

inline size_t calculate_number_of_messages(
  std::unordered_map<client_id, sequence_set, client_id_hash> & message_map)
{
  size_t message_count = 0;
  for (auto & [client_id, message_list] : message_map) {
    message_count += message_list.size();
  }

  return message_count;
}

using action_analysis =
  std::unordered_map<std::string, std::shared_ptr<action_service_req_resp_info>>;

inline void update_action_service_info_with_num_req_resp(
  action_analysis & action_process_info,
  std::unordered_map<std::string, std::shared_ptr<rosbag2_action_info_t>> & all_action_info)
{
  for (auto & [action_name, action_info] : action_process_info) {
    // Get the number of request from all clients for send_goal
    all_action_info[action_name]->send_goal_service_msg_count.first =
      calculate_number_of_messages(action_info->send_goal_service.request);

    // Get the number of request from all clients for cancel_goal
    all_action_info[action_name]->cancel_goal_service_msg_count.first =
      calculate_number_of_messages(action_info->cancel_goal_service.request);

    // Get the number of request from all clients for get_result
    all_action_info[action_name]->get_result_service_msg_count.first =
      calculate_number_of_messages(action_info->get_result_service.request);

    // Get the number of response from all clients for send_goal
    all_action_info[action_name]->send_goal_service_msg_count.second =
      calculate_number_of_messages(action_info->send_goal_service.response);

    // Get the number of response from all clients for cancel_goal
    all_action_info[action_name]->cancel_goal_service_msg_count.second =
      calculate_number_of_messages(action_info->cancel_goal_service.response);

    // Get the number of response from all clients for get_result
    all_action_info[action_name]->get_result_service_msg_count.second =
      calculate_number_of_messages(action_info->get_result_service.response);
  }
}

using service_analysis =
  std::unordered_map<std::string, std::shared_ptr<service_req_resp_info>>;

inline void update_service_info_with_num_req_resp(
  service_analysis & service_process_info,
  std::unordered_map<std::string, std::shared_ptr<rosbag2_service_info_t>> & all_service_info)
{
  for (auto & [topic_name, service_info] : service_process_info) {
    // Get the number of request from all clients
    all_service_info[topic_name]->request_count =
      calculate_number_of_messages(service_info->request);

    // Get the number of response from all clients
    all_service_info[topic_name]->response_count =
      calculate_number_of_messages(service_info->response);
  }
}
}  // namespace

std::pair<
  std::vector<std::shared_ptr<rosbag2_service_info_t>>,
  std::vector<std::shared_ptr<rosbag2_action_info_t>>
>
Info::read_service_and_action_info(const std::string & uri, const std::string & storage_id)
{
  rosbag2_storage::StorageFactory factory;
  auto storage = factory.open_read_only({uri, storage_id});
  if (!storage) {
    throw std::runtime_error("No plugin detected that could open file " + uri);
  }

  rosbag2_storage::ReadOrder read_order;
  if (!storage->set_read_order(read_order)) {
    throw std::runtime_error("Failed to set read order on " + uri);
  }

  // Service event topic name as key
  service_analysis service_process_info;
  std::unordered_map<std::string, std::shared_ptr<rosbag2_service_info_t>> all_service_info;

  // Action name as key
  action_analysis action_process_info;
  std::unordered_map<std::string, std::shared_ptr<rosbag2_action_info_t>> all_action_info;

  std::vector<std::shared_ptr<rosbag2_action_info_t>> output_action_info;
  std::vector<std::shared_ptr<rosbag2_service_info_t>> output_service_info;

  std::unordered_map<std::string, std::string> action_interface_name_to_action_name_map;
  std::unordered_map<std::string, std::string> service_event_name_to_service_name_map;

  auto all_topics_types = storage->get_all_topics_and_types();
  for (auto & t : all_topics_types) {
    if (is_topic_belong_to_action(t.name, t.type)) {
      std::shared_ptr<rosbag2_action_info_t> action_info;
      std::string action_name = action_interface_name_to_action_name(t.name);
      if (all_action_info.find(action_name) == all_action_info.end()) {
        action_info = std::make_shared<rosbag2_action_info_t>();
        action_info->name = action_name;
        action_info->serialization_format = t.serialization_format;
        all_action_info.emplace(action_name, action_info);
        action_process_info[action_name] = std::make_shared<action_service_req_resp_info>();
      } else {
        action_info = all_action_info[action_name];
      }

      // Update action type. Note: cancel_goal event topic and status topic cannot get type
      if (action_info->type.empty()) {
        action_info->type = get_action_type_for_info(t.type);
      }

      // Update action_interface_name_to_action_name_map to speed up following code.
      action_interface_name_to_action_name_map[t.name] = action_name;
    } else if (is_service_event_topic(t.name, t.type)) {
      auto service_info = std::make_shared<rosbag2_cpp::rosbag2_service_info_t>();
      service_info->name = service_event_topic_name_to_service_name(t.name);
      service_info->type = service_event_topic_type_to_service_type(t.type);
      service_info->serialization_format = t.serialization_format;
      all_service_info.emplace(t.name, service_info);
      service_process_info[t.name] = std::make_shared<service_req_resp_info>();

      // Update service_event_name_to_service_name_map to speed up following code.
      service_event_name_to_service_name_map[t.name] = service_info->name;
    }
  }

  if (!all_service_info.empty() || !all_action_info.empty()) {
    auto msg = service_msgs::msg::ServiceEventInfo();
    const rosidl_message_type_support_t * service_event_type_support_info =
      rosidl_typesupport_cpp::
      get_message_type_support_handle<service_msgs::msg::ServiceEventInfo>();

    while (storage->has_next()) {
      auto bag_msg = storage->read_next();

      if (action_interface_name_to_action_name_map.count(bag_msg->topic_name) == 0 &&
        service_event_name_to_service_name_map.count(bag_msg->topic_name) == 0)
      {
        continue;  // Skip the regular topics
      }

      auto action_interface_type = get_action_interface_type(bag_msg->topic_name);
      if (action_interface_type == ActionInterfaceType::Feedback ||
        action_interface_type == ActionInterfaceType::Status)
      {
        auto action_name = action_interface_name_to_action_name_map[bag_msg->topic_name];
        if (action_interface_type == ActionInterfaceType::Feedback) {
          all_action_info[action_name]->feedback_topic_msg_count++;
        } else {
          all_action_info[action_name]->status_topic_msg_count++;
        }
        continue;
      }

      auto ret = rmw_deserialize(
        bag_msg->serialized_data.get(),
        service_event_type_support_info,
        reinterpret_cast<void *>(&msg)
      );
      if (ret != RMW_RET_OK) {
        throw std::runtime_error(
                "Failed to deserialize message from " + bag_msg->topic_name + " !");
      }

      if (action_interface_type != ActionInterfaceType::Unknown) {
        auto action_name = action_interface_name_to_action_name_map[bag_msg->topic_name];
        auto action = action_process_info[action_name];

        // Handle action service event topic
        switch (msg.event_type) {
          case service_msgs::msg::ServiceEventInfo::REQUEST_SENT:
          case service_msgs::msg::ServiceEventInfo::REQUEST_RECEIVED:
            {
              if (action_interface_type == ActionInterfaceType::SendGoalEvent) {
                action->send_goal_service.request[msg.client_gid].emplace(msg.sequence_number);
              } else if (action_interface_type == ActionInterfaceType::GetResultEvent) {
                action->get_result_service.request[msg.client_gid].emplace(msg.sequence_number);
              } else {
                // TopicsInAction::CancelGoalEvent
                action->cancel_goal_service.request[msg.client_gid].emplace(msg.sequence_number);
              }
              break;
            }
          case service_msgs::msg::ServiceEventInfo::RESPONSE_SENT:
          case service_msgs::msg::ServiceEventInfo::RESPONSE_RECEIVED:
            {
              if (action_interface_type == ActionInterfaceType::SendGoalEvent) {
                action->send_goal_service.response[msg.client_gid].emplace(msg.sequence_number);
              } else if (action_interface_type == ActionInterfaceType::GetResultEvent) {
                action->get_result_service.response[msg.client_gid].emplace(msg.sequence_number);
              } else {
                // TopicsInAction::CancelGoalEvent
                action->cancel_goal_service.response[msg.client_gid].emplace(msg.sequence_number);
              }
              break;
            }
          default:
            throw std::range_error("Invalid service event type " +
              std::to_string(msg.event_type) + " !");
        }
      } else {
        // Handle service event topic
        auto service_info = service_process_info[bag_msg->topic_name];
        switch (msg.event_type) {
          case service_msgs::msg::ServiceEventInfo::REQUEST_SENT:
          case service_msgs::msg::ServiceEventInfo::REQUEST_RECEIVED:
            service_info->request[msg.client_gid].emplace(msg.sequence_number);
            break;
          case service_msgs::msg::ServiceEventInfo::RESPONSE_SENT:
          case service_msgs::msg::ServiceEventInfo::RESPONSE_RECEIVED:
            service_info->response[msg.client_gid].emplace(msg.sequence_number);
            break;
          default:
            throw std::range_error("Invalid service event type " +
              std::to_string(msg.event_type) + " !");
        }
      }
    }

    // Process action_process_info to get the number of request and response
    update_action_service_info_with_num_req_resp(action_process_info, all_action_info);

    // Covert all_action_info to output_action_info
    for (auto & [action_name, action_info] : all_action_info) {
      output_action_info.emplace_back(std::move(action_info));
    }

    // Process service_process_info to get the number of request and response
    update_service_info_with_num_req_resp(service_process_info, all_service_info);

    // Convert all_service_info to output_service_info
    for (auto & [topic_name, service_info] : all_service_info) {
      output_service_info.emplace_back(std::move(service_info));
    }
  }

  return std::make_pair(std::move(output_service_info), std::move(output_action_info));
}

std::unordered_map<std::string, uint64_t> Info::compute_messages_size_contribution(
  const std::string & uri, const std::string & storage_id)
{
  rosbag2_storage::StorageFactory factory;
  auto storage = factory.open_read_only({uri, storage_id});
  if (!storage) {
    throw std::runtime_error("No plugin detected that could open file " + uri);
  }

  std::unordered_map<std::string, uint64_t> messages_size;
  while (storage->has_next()) {
    auto bag_msg = storage->read_next();
    messages_size[bag_msg->topic_name] += bag_msg->serialized_data->buffer_length;
  }

  return messages_size;
}

}  // namespace rosbag2_cpp
