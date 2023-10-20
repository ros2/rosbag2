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

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace rosbag2_transport
{

class PlayerServiceClient final
{
public:
  explicit PlayerServiceClient(
    std::shared_ptr<rclcpp::GenericClient> cli,
    std::string service_name,
    const std::string & service_event_type,
    const rclcpp::Logger logger);

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

}  // namespace rosbag2_transport

#endif  // ROSBAG2_TRANSPORT__PLAYER_SERVICE_CLIENT_HPP_
