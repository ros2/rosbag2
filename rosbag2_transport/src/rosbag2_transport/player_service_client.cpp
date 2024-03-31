// Copyright 2023 Sony Group Corporation.
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

#include <memory>
#include <utility>

#include "rosbag2_transport/player_service_client.hpp"

#include "rosbag2_cpp/service_utils.hpp"

#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"

#include "logging.hpp"

namespace rosbag2_transport
{

PlayerServiceClient::PlayerServiceClient(
  std::shared_ptr<rclcpp::GenericClient> generic_client,
  std::string service_name,
  const std::string & service_event_type,
  rclcpp::Logger logger,
  std::shared_ptr<PlayerServiceClientManager> player_service_client_manager)
: client_(std::move(generic_client)),
  service_name_(std::move(service_name)),
  logger_(std::move(logger)),
  player_service_client_manager_(std::move(player_service_client_manager))
{
  service_event_ts_lib_ =
    rclcpp::get_typesupport_library(service_event_type, "rosidl_typesupport_cpp");

  service_event_type_ts_ = rclcpp::get_message_typesupport_handle(
    service_event_type, "rosidl_typesupport_cpp", *service_event_ts_lib_);

  auto service_event_ts_introspection = get_message_typesupport_handle(
    service_event_type_ts_, rosidl_typesupport_introspection_cpp::typesupport_identifier);

  service_event_members_ =
    reinterpret_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(
    service_event_ts_introspection->data);

  // Sanity checks for service_event_members_
  if (service_event_members_ == nullptr) {
    throw std::invalid_argument("service_event_members_ for `" + service_name_ + "` is nullptr");
  }
  if (service_event_members_->member_count_ != 3) {
    // members_[0]: service_info, members_[1]: request[<=1], members_[2]: response[<=1]
    std::stringstream ss;
    ss << "Expected 3 fields in the service introspection message, but got " <<
      service_event_members_->member_count_;
    throw std::invalid_argument(ss.str());
  }

  if (!service_event_members_->members_[1].is_array_) {
    std::stringstream ss;
    ss << "The service request for '" << service_name_ << "' is not array.\n";
    throw std::invalid_argument(ss.str());
  }

  if (service_event_members_->members_[1].size_function == nullptr) {
    std::stringstream ss;
    ss << "size_function() for service request '" << service_name_ << "' is not set.\n";
    throw std::invalid_argument(ss.str());
  }

  if (service_event_members_->members_[1].get_function == nullptr) {
    std::stringstream ss;
    ss << "get_function() for service request '" << service_name_ << "' is not set.\n";
    throw std::invalid_argument(ss.str());
  }

  if (service_event_members_->init_function == nullptr) {
    std::stringstream ss;
    ss << "service_event_members_->init_function for '" << service_name_ << "' is not set.\n";
    throw std::invalid_argument(ss.str());
  }

  if (service_event_members_->fini_function == nullptr) {
    std::stringstream ss;
    ss << "service_event_members_->fini_function for '" << service_name_ << "' is not set.\n";
    throw std::invalid_argument(ss.str());
  }
}

bool PlayerServiceClient::is_include_request_message(const rcl_serialized_message_t & message)
{
  auto [type, client_id, sequence_number] = get_msg_event_type(message);

  // Ignore response message
  if (type == service_msgs::msg::ServiceEventInfo::RESPONSE_SENT ||
    type == service_msgs::msg::ServiceEventInfo::RESPONSE_RECEIVED)
  {
    return false;
  }

  bool ret = false;

  // For each Client, decide which request data to use based on the first message related to
  // the request that is obtained from the record data.
  // e.g.

  auto iter = request_info_.find(client_id);
  if (type == service_msgs::msg::ServiceEventInfo::REQUEST_RECEIVED) {
    if (!service_set_introspection_content_) {
      if (rosbag2_cpp::service_event_include_metadata_and_contents(message.buffer_length)) {
        service_set_introspection_content_ = true;
      }
    }

    if (iter != request_info_.end()) {
      switch (iter->second) {
        case request_info_from::CLIENT:
          {
            // Already decide using request data from client.
            break;
          }
        case request_info_from::NO_CONTENT:
          {
            if (service_set_introspection_content_) {
              // introspection type is changed from metadata to metadata + contents
              request_info_[client_id] = request_info_from::SERVICE;
              ret = true;
            } else {
              RCUTILS_LOG_WARN_ONCE_NAMED(
                ROSBAG2_TRANSPORT_PACKAGE_NAME,
                "The configuration of introspection for '%s' is metadata on service side !",
                service_name_.c_str());
            }
            break;
          }
        default:  // request_info_from::SERVICE:
          {
            // Already decide using request data from service.
            ret = true;
          }
      }
    } else {
      if (service_set_introspection_content_) {
        request_info_[client_id] = request_info_from::SERVICE;
        ret = true;
      } else {
        request_info_[client_id] = request_info_from::NO_CONTENT;  // Only have metadata
      }
    }

    return ret;
  }

  // type is service_msgs::msg::ServiceEventInfo::REQUEST_SENT
  if (iter != request_info_.end()) {
    switch (iter->second) {
      case request_info_from::CLIENT:
        {
          // Already decide using request data from client.
          ret = true;
          break;
        }
      case request_info_from::NO_CONTENT:
        {
          if (rosbag2_cpp::service_event_include_metadata_and_contents(message.buffer_length)) {
            // introspection type is changed from metadata to metadata + contents
            request_info_[client_id] = request_info_from::CLIENT;
            ret = true;
          } else {
            RCUTILS_LOG_WARN_ONCE_NAMED(
              ROSBAG2_TRANSPORT_PACKAGE_NAME,
              "The configuration of introspection for '%s' client [ID: %s]` is metadata !",
              rosbag2_cpp::client_id_to_string(client_id).c_str(),
              service_name_.c_str());
          }
          break;
        }
      default:  // request_info_from::SERVICE:
        {
          // Already decide using request data from service.
          ret = false;
        }
    }
  } else {
    if (rosbag2_cpp::service_event_include_metadata_and_contents(message.buffer_length)) {
      request_info_[client_id] = request_info_from::CLIENT;
      ret = true;
    } else {
      request_info_[client_id] = request_info_from::NO_CONTENT;
    }
  }

  return ret;
}

const std::string & PlayerServiceClient::get_service_name()
{
  return service_name_;
}

std::shared_ptr<uint8_t[]>
PlayerServiceClient::deserialize_service_event(const rcl_serialized_message_t & message)
{
  auto type_erased_service_event = std::shared_ptr<uint8_t[]>(
    new uint8_t[service_event_members_->size_of_],
    [fini_function = this->service_event_members_->fini_function](uint8_t * msg) {
      fini_function(msg);
      delete[] msg;
    });

  service_event_members_->init_function(
    type_erased_service_event.get(), rosidl_runtime_cpp::MessageInitialization::ZERO);

  rmw_ret_t ret =
    rmw_deserialize(&message, service_event_type_ts_, type_erased_service_event.get());
  if (ret != RMW_RET_OK) {  // Failed to deserialize service event message
    type_erased_service_event.reset();
  }
  return type_erased_service_event;
}

std::tuple<PlayerServiceClient::ServiceEventType, PlayerServiceClient::ClientGidType>
PlayerServiceClient::get_service_event_type_and_client_gid(
  const std::shared_ptr<uint8_t[]> type_erased_service_event)
{
  if (type_erased_service_event) {
    // members_[0]: service_info, members_[1]: request[<=1], members_[2]: response[<=1]
    const auto & info_member = service_event_members_->members_[0];

    auto service_event_info_ptr = reinterpret_cast<service_msgs::msg::ServiceEventInfo *>(
      type_erased_service_event.get() + info_member.offset_);
    if (service_event_info_ptr == nullptr) {
      throw std::runtime_error("Error: The service_event_info_ptr is nullptr");
    }
    return {service_event_info_ptr->event_type, service_event_info_ptr->client_gid};
  } else {
    throw std::invalid_argument("Error: The type_erased_service_event is nullptr");
  }
}

bool PlayerServiceClient::is_service_event_include_request_message(
  const std::shared_ptr<uint8_t[]> type_erased_service_event)
{
  if (type_erased_service_event) {
    // members_[0]: service_info, members_[1]: request[<=1], members_[2]: response[<=1]
    const auto & request_member = service_event_members_->members_[1];
    void * request_sequence_ptr = type_erased_service_event.get() + request_member.offset_;
    if (request_member.size_function(request_sequence_ptr) > 0) {
      return true;
    }   // else { /* No service request */ }
  } else {
    throw std::invalid_argument("Error: The type_erased_service_event is nullptr");
  }
  return false;
}

void PlayerServiceClient::async_send_request(
  const std::shared_ptr<uint8_t[]> type_erased_service_event)
{
  // members_[0]: service_info, members_[1]: request[<=1], members_[2]: response[<=1]
  const auto & request_member = service_event_members_->members_[1];
  void * request_sequence_ptr = type_erased_service_event.get() + request_member.offset_;
  if (request_member.size_function(request_sequence_ptr) > 0) {
    void * request_ptr = request_member.get_function(request_sequence_ptr, 0);
    auto future_and_request_id = client_->async_send_request(request_ptr);
    player_service_client_manager_->register_request_future(future_and_request_id, client_);
  }  // else { /* No service request in the service event. Do nothing, just skip it. */ }
}

void PlayerServiceClient::async_send_request(const rcl_serialized_message_t & message)
{
  if (!client_->service_is_ready()) {
    RCLCPP_ERROR(
      logger_, "Service request hasn't been sent. The '%s' service isn't ready !",
      service_name_.c_str());
    return;
  }

  auto type_erased_ros_message = deserialize_service_event(message);

  if (type_erased_ros_message) {
    async_send_request(type_erased_ros_message);
  } else {
    throw std::runtime_error(
            "Failed to deserialize service event message for " + service_name_ + " !");
  }
}

std::tuple<uint8_t, PlayerServiceClient::ClientGidType, int64_t>
PlayerServiceClient::get_msg_event_type(const rcl_serialized_message_t & message)
{
  auto msg = service_msgs::msg::ServiceEventInfo();

  const rosidl_message_type_support_t * type_support_info =
    rosidl_typesupport_cpp::get_message_type_support_handle<service_msgs::msg::ServiceEventInfo>();
  if (type_support_info == nullptr) {
    throw std::runtime_error("Failed to get message type support handle of service event info !");
  }

  // Partially deserialize service event message. Deserializing only first member ServiceEventInfo
  // with assumption that it is going to be the first in serialized message.
  // TODO(morlov): We can't rely on this assumption. It is up to the underlying RMW and
  //  serialization format implementation!
  if (rmw_deserialize(&message, type_support_info, reinterpret_cast<void *>(&msg)) != RMW_RET_OK) {
    throw std::runtime_error("Failed to deserialize message !");
  }

  return {msg.event_type, msg.client_gid, msg.sequence_number};
}

PlayerServiceClientManager::PlayerServiceClientManager(
  std::chrono::seconds requst_future_timeout,
  size_t maximum_request_future_queue)
: request_future_timeout_(std::chrono::seconds(requst_future_timeout)),
  maximum_request_future_queue_(maximum_request_future_queue)
{
}

bool PlayerServiceClientManager::request_future_queue_is_full()
{
  std::lock_guard<std::mutex> lock(request_futures_list_mutex_);

  // To improve performance, it's not necessary to clean up completed requests and timeout requests
  // every time.
  if (request_futures_list_.size() < maximum_request_future_queue_) {
    return false;
  }

  remove_complete_request_future();
  remove_all_timeout_request_future();

  if (request_futures_list_.size() == maximum_request_future_queue_) {
    return true;
  }

  return false;
}

bool PlayerServiceClientManager::register_request_future(
  rclcpp::GenericClient::FutureAndRequestId & request_future,
  std::weak_ptr<rclcpp::GenericClient> client)
{
  auto future_and_request_id =
    std::make_unique<rclcpp::GenericClient::FutureAndRequestId>(std::move(request_future));

  if (!request_future_queue_is_full()) {
    std::lock_guard<std::mutex> lock(request_futures_list_mutex_);
    request_futures_list_[std::chrono::steady_clock::now()] =
    {std::move(future_and_request_id), client};
    return true;
  } else {
    ROSBAG2_TRANSPORT_LOG_WARN(
      "Client request queue is full. "
      "Please consider increasing the length of the queue.");
  }

  return false;
}

void PlayerServiceClientManager::remove_complete_request_future()
{
  std::vector<time_point> remove_keys;
  for (auto & [timestamp, request_id_and_client] : request_futures_list_) {
    if (request_id_and_client.first->wait_for(std::chrono::seconds(0)) ==
      std::future_status::ready)
    {
      auto client = request_id_and_client.second.lock();
      if (client) {
        client->remove_pending_request(request_id_and_client.first->request_id);
      }
      remove_keys.emplace_back(timestamp);
    }
  }
  for (auto & key : remove_keys) {
    request_futures_list_.erase(key);
  }
}

void PlayerServiceClientManager::remove_all_timeout_request_future()
{
  auto current_time = std::chrono::steady_clock::now();
  auto first_iter_without_timeout =
    request_futures_list_.lower_bound(current_time - request_future_timeout_);

  if (first_iter_without_timeout == request_futures_list_.begin()) {
    return;
  }

  auto last_iter_with_timeout = --first_iter_without_timeout;
  for (auto iter = request_futures_list_.begin(); iter != last_iter_with_timeout; iter++) {
    auto client = iter->second.second.lock();
    if (client) {
      client->remove_pending_request(iter->second.first->request_id);
    }
  }
  request_futures_list_.erase(request_futures_list_.begin(), last_iter_with_timeout);
  ROSBAG2_TRANSPORT_LOG_WARN(
    "Client requests are discarded since timeout. "
    "Please consider setting a longer timeout.");
}
}  // namespace rosbag2_transport
