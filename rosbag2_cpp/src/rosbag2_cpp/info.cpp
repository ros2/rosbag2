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
}  // namespace

std::vector<std::shared_ptr<rosbag2_service_info_t>> Info::read_service_info(
  const std::string & uri, const std::string & storage_id)
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

  using service_analysis =
    std::unordered_map<std::string, std::shared_ptr<service_req_resp_info>>;

  std::unordered_map<std::string, std::shared_ptr<rosbag2_service_info_t>> all_service_info;
  service_analysis service_process_info;

  auto all_topics_types = storage->get_all_topics_and_types();
  for (auto & t : all_topics_types) {
    if (is_service_event_topic(t.name, t.type)) {
      auto service_info = std::make_shared<rosbag2_cpp::rosbag2_service_info_t>();
      service_info->name = service_event_topic_name_to_service_name(t.name);
      service_info->type = service_event_topic_type_to_service_type(t.type);
      service_info->serialization_format = t.serialization_format;
      all_service_info.emplace(t.name, service_info);
      service_process_info[t.name] = std::make_shared<service_req_resp_info>();
    }
  }

  std::vector<std::shared_ptr<rosbag2_service_info_t>> ret_service_info;

  if (!all_service_info.empty()) {
    auto msg = service_msgs::msg::ServiceEventInfo();
    const rosidl_message_type_support_t * type_support_info =
      rosidl_typesupport_cpp::
      get_message_type_support_handle<service_msgs::msg::ServiceEventInfo>();

    while (storage->has_next()) {
      auto bag_msg = storage->read_next();

      // Check if topic is service event topic
      auto one_service_info = all_service_info.find(bag_msg->topic_name);
      if (one_service_info == all_service_info.end()) {
        continue;
      }

      auto ret = rmw_deserialize(
        bag_msg->serialized_data.get(),
        type_support_info,
        reinterpret_cast<void *>(&msg));
      if (ret != RMW_RET_OK) {
        throw std::runtime_error(
                "Failed to deserialize message from " + bag_msg->topic_name + " !");
      }

      switch (msg.event_type) {
        case service_msgs::msg::ServiceEventInfo::REQUEST_SENT:
        case service_msgs::msg::ServiceEventInfo::REQUEST_RECEIVED:
          service_process_info[bag_msg->topic_name]->request[msg.client_gid].emplace(
            msg.sequence_number);
          break;
        case service_msgs::msg::ServiceEventInfo::RESPONSE_SENT:
        case service_msgs::msg::ServiceEventInfo::RESPONSE_RECEIVED:
          service_process_info[bag_msg->topic_name]->response[msg.client_gid].emplace(
            msg.sequence_number);
          break;
      }
    }

    for (auto & [topic_name, service_info] : service_process_info) {
      size_t count = 0;
      // Get the number of request from all clients
      for (auto &[client_id, request_list] : service_info->request) {
        count += request_list.size();
      }
      all_service_info[topic_name]->request_count = count;

      count = 0;
      // Get the number of response from all clients
      for (auto &[client_id, response_list] : service_info->response) {
        count += response_list.size();
      }
      all_service_info[topic_name]->response_count = count;
    }

    for (auto & [topic_name, service_info] : all_service_info) {
      ret_service_info.emplace_back(std::move(service_info));
    }
  }

  return ret_service_info;
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
