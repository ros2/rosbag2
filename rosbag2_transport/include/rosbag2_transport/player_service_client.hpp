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
#include <string>

#include "rclcpp/generic_client.hpp"
#include "rclcpp/rclcpp.hpp"

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
  bool is_include_request_message(const rclcpp::SerializedMessage & message);

private:
  std::shared_ptr<rclcpp::GenericClient> client_;
  std::string service_name_;
  const rclcpp::Logger logger_;
  std::shared_ptr<PlayerServiceClientManager> player_service_client_manager_;
  enum class introspection_type
  {
    UNKNOW = 0,
    METADATA,
    CONTENTS
  };
  introspection_type client_side_type_ = introspection_type::UNKNOW;
  introspection_type service_side_type_ = introspection_type::UNKNOW;

  std::shared_ptr<rcpputils::SharedLibrary> ts_lib_;
  const rosidl_message_type_support_t * ts_;
  const rosidl_typesupport_introspection_cpp::MessageMembers * message_members_;

  rcutils_allocator_t allocator_ = rcutils_get_default_allocator();

  uint8_t get_msg_event_type(const rclcpp::SerializedMessage & message);
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
};

}  // namespace rosbag2_transport

#endif  // ROSBAG2_TRANSPORT__PLAYER_SERVICE_CLIENT_HPP_
