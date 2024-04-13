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

using namespace std::chrono_literals;

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
    player_service_client_manager_->register_request_future(
      std::move(future_and_request_id), client_);
  }  // else { /* No service request in the service event. Do nothing, just skip it. */ }
}

bool PlayerServiceClient::wait_for_sent_requests_to_finish(std::chrono::duration<double> timeout)
{
  return player_service_client_manager_->wait_for_sent_requests_to_finish(client_, timeout);
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
  rclcpp::GenericClient::FutureAndRequestId && future_and_request_id,
  std::weak_ptr<rclcpp::GenericClient> client)
{
  auto future_and_request_id_shared_ptr =
    std::make_shared<rclcpp::GenericClient::FutureAndRequestId>(std::move(future_and_request_id));

  if (!request_future_queue_is_full()) {
    std::lock_guard<std::mutex> lock(request_futures_list_mutex_);
    auto emplace_result = request_futures_list_.emplace(
      std::chrono::steady_clock::now(),
      std::make_pair(std::move(future_and_request_id_shared_ptr), client));
    return emplace_result.second;
  } else {
    ROSBAG2_TRANSPORT_LOG_WARN(
      "Client request queue is full. "
      "Please consider increasing the length of the queue.");
  }
  return false;
}

bool PlayerServiceClientManager::wait_for_sent_requests_to_finish(
  std::shared_ptr<rclcpp::GenericClient> client,
  std::chrono::duration<double> timeout)
{
  auto is_all_futures_ready = [&]() {
      for (auto & [timestamp, future_request_id_and_client] : request_futures_list_) {
        if (client) {
          auto current_client = future_request_id_and_client.second.lock();
          if (current_client == nullptr) {
            throw std::runtime_error("request's client is not valid\n");
          }
          // Wait for futures only from specified client
          if (client != current_client) {
            continue;
          }
        }  // else { /* if the client is not specified, wait for futures from all clients */ }

        const auto & future_and_request_id = future_request_id_and_client.first;
        if (!future_and_request_id->future.valid()) {
          std::stringstream ss;
          ss << "request's " << future_and_request_id->request_id << " future is not valid!\n";
          throw std::runtime_error(ss.str());
        }
        if (future_and_request_id->wait_for(0s) != std::future_status::ready) {
          return false;
        }
      }
      return true;
    };

  auto sleep_time = std::chrono::milliseconds(10);
  if (timeout < std::chrono::seconds(1)) {
    sleep_time = std::chrono::duration_cast<std::chrono::milliseconds>(timeout);
  }
  using clock = std::chrono::system_clock;
  auto start = clock::now();

  std::lock_guard<std::mutex> lock(request_futures_list_mutex_);
  while (!is_all_futures_ready() && (clock::now() - start) < timeout) {
    std::this_thread::sleep_for(sleep_time);
  }

  return is_all_futures_ready();
}

void PlayerServiceClientManager::remove_complete_request_future()
{
  std::vector<time_point> remove_keys;
  std::lock_guard<std::mutex> lock(request_futures_list_mutex_);
  for (auto & [timestamp, future_request_id_and_client] : request_futures_list_) {
    if (future_request_id_and_client.first->wait_for(0s) == std::future_status::ready) {
      auto client = future_request_id_and_client.second.lock();
      if (client) {
        client->remove_pending_request(future_request_id_and_client.first->request_id);
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
  std::lock_guard<std::mutex> lock(request_futures_list_mutex_);
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
