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
#include <unordered_map>
#include <string>
#include <tuple>

#include "rclcpp/generic_client.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rosbag2_cpp/service_utils.hpp"

namespace rosbag2_transport
{

class PlayerServiceClientManager;

class PlayerServiceClient final
{
public:
  explicit PlayerServiceClient(
    std::shared_ptr<rclcpp::GenericClient> cli,
    std::string service_name,
    const std::string & service_event_type,
    const rclcpp::Logger logger,
    std::shared_ptr<PlayerServiceClientManager> player_service_client_manager);

  // Can call this function if check_include_request_message() return true
  void async_send_request(const rclcpp::SerializedMessage & message);

  std::shared_ptr<rclcpp::GenericClient> generic_client()
  {
    return client_;
  }

  // Check if message can be unpacked to get request message
  bool include_request_message(const rclcpp::SerializedMessage & message);

private:
  std::shared_ptr<rclcpp::GenericClient> client_;
  std::string service_name_;
  const rclcpp::Logger logger_;
  std::shared_ptr<PlayerServiceClientManager> player_service_client_manager_;
  enum class request_info_from
  {
    SERVICE = 0,
    CLIENT,
    NO_CONTENT  // Only have META info. Not send request.
  };
  bool service_set_introspection_content_ = false;

  using client_id = service_msgs::msg::ServiceEventInfo::_client_gid_type;
  // Info on request data from service or client
  std::unordered_map<client_id, request_info_from, rosbag2_cpp::client_id_hash> request_info_;

  std::shared_ptr<rcpputils::SharedLibrary> ts_lib_;
  const rosidl_message_type_support_t * service_event_type_ts_;
  const rosidl_typesupport_introspection_cpp::MessageMembers * message_members_;

  rcutils_allocator_t allocator_ = rcutils_get_default_allocator();

  std::tuple<uint8_t, client_id, int64_t>
  get_msg_event_type(const rclcpp::SerializedMessage & message);
};

class PlayerServiceClientManager final
{
public:
  PlayerServiceClientManager(
    std::chrono::seconds request_future_timeout, size_t maximum_request_future_queue = 30);

  // Timeout future will be discarded and check queue.
  bool request_future_queue_is_full();

  bool register_request_future(rclcpp::GenericClient::FutureAndRequestId & request_future);

private:
  using time_point = std::chrono::steady_clock::time_point;
  using ptr_future_and_request_id = std::unique_ptr<rclcpp::GenericClient::FutureAndRequestId>;
  std::map<time_point, ptr_future_and_request_id> request_futures_list_;
  std::mutex request_futures_list_lock_;

  std::chrono::seconds request_future_timeout_;
  size_t maximum_request_future_queue_;

  void
  remove_complete_request_future();

  void
  remove_all_timeout_request_future();
};

}  // namespace rosbag2_transport

#endif  // ROSBAG2_TRANSPORT__PLAYER_SERVICE_CLIENT_HPP_
