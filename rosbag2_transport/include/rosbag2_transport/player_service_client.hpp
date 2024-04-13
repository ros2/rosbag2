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

#ifndef ROSBAG2_TRANSPORT__PLAYER_SERVICE_CLIENT_HPP_
#define ROSBAG2_TRANSPORT__PLAYER_SERVICE_CLIENT_HPP_

#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <utility>
#include <string>
#include <tuple>

#include "rcl/types.h"
#include "rclcpp/generic_client.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rosbag2_cpp/service_utils.hpp"

namespace rosbag2_transport
{

class PlayerServiceClientManager;

class PlayerServiceClient final
{
public:
  using ServiceEventType = service_msgs::msg::ServiceEventInfo::_event_type_type;
  using ClientGidType = service_msgs::msg::ServiceEventInfo::_client_gid_type;

  explicit
  PlayerServiceClient(
    std::shared_ptr<rclcpp::GenericClient> generic_client,
    std::string service_name,
    const std::string & service_event_type,
    rclcpp::Logger logger,
    std::shared_ptr<PlayerServiceClientManager> player_service_client_manager);

  const std::string & get_service_name();

  /// \brief Deserialize message to the type erased service event
  /// \param message - Serialized message
  /// \return Shared pointer to the byte array with deserialized service event if success,
  /// otherwise nullptr
  std::shared_ptr<uint8_t[]> deserialize_service_event(const rcl_serialized_message_t & message);

  std::tuple<PlayerServiceClient::ServiceEventType, PlayerServiceClient::ClientGidType>
  get_service_event_type_and_client_gid(const std::shared_ptr<uint8_t[]> type_erased_service_event);

  bool is_service_event_include_request_message(
    const std::shared_ptr<uint8_t[]> type_erased_service_event);

  void async_send_request(const std::shared_ptr<uint8_t[]> type_erased_service_event);

  /// Wait until sent service requests will receive responses from service servers.
  /// \param timeout - Timeout in fraction of seconds to wait for.
  /// \return true if service requests successfully finished, otherwise false.
  bool wait_for_sent_requests_to_finish(
    std::chrono::duration<double> timeout = std::chrono::seconds(5));

  std::shared_ptr<rclcpp::GenericClient> generic_client()
  {
    return client_;
  }

private:
  std::shared_ptr<rclcpp::GenericClient> client_;
  std::string service_name_;
  const rclcpp::Logger logger_;
  std::shared_ptr<PlayerServiceClientManager> player_service_client_manager_;
  // Note: The service_event_ts_lib_ shall be a member variable to make sure that library loaded
  // during the liveliness of the instance of this class, since we have raw pointers to its members.
  std::shared_ptr<rcpputils::SharedLibrary> service_event_ts_lib_;

  const rosidl_message_type_support_t * service_event_type_ts_;
  const rosidl_typesupport_introspection_cpp::MessageMembers * service_event_members_;

  rcutils_allocator_t allocator_ = rcutils_get_default_allocator();
};

class PlayerServiceClientManager final
{
public:
  explicit PlayerServiceClientManager(
    std::chrono::seconds request_future_timeout = std::chrono::minutes(30),
    size_t maximum_request_future_queue = 100);

  // Timeout future will be discarded and check queue.
  bool request_future_queue_is_full();

  bool register_request_future(
    rclcpp::GenericClient::FutureAndRequestId && future_and_request_id,
    std::weak_ptr<rclcpp::GenericClient> client);

  /// Wait until sent service requests will receive responses from service servers.
  /// \param client - Generic service client from which requests was sent.
  /// \note If client is a nullptr it will be taken into account all requests from all clients
  /// and timeout will be due per each client.
  /// \param timeout - Timeout in fraction of seconds to wait for.
  /// \return true if service requests successfully finished, otherwise false.
  bool wait_for_sent_requests_to_finish(
    std::shared_ptr<rclcpp::GenericClient> client,
    std::chrono::duration<double> timeout = std::chrono::seconds(5));

private:
  using time_point = std::chrono::steady_clock::time_point;
  using FutureAndRequestIdSharedPtr = std::shared_ptr<rclcpp::GenericClient::FutureAndRequestId>;
  using FutureAndRequestIdAndClient =
    std::pair<FutureAndRequestIdSharedPtr, std::weak_ptr<rclcpp::GenericClient>>;
  std::map<time_point, FutureAndRequestIdAndClient> request_futures_list_;
  std::mutex request_futures_list_mutex_;

  std::chrono::seconds request_future_timeout_;
  size_t maximum_request_future_queue_;

  void remove_complete_request_future();

  void remove_all_timeout_request_future();
};

}  // namespace rosbag2_transport

#endif  // ROSBAG2_TRANSPORT__PLAYER_SERVICE_CLIENT_HPP_
