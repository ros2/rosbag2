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

#include "rosbag2_transport/player_service_client.hpp"

#include "rclcpp/logger.hpp"

#include "rosbag2_cpp/service_utils.hpp"

#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"

#include "logging.hpp"

namespace rosbag2_transport
{

PlayerServiceClient::PlayerServiceClient(
  std::shared_ptr<rclcpp::GenericClient> cli,
  std::string service_name,
  const std::string & service_event_type,
  const rclcpp::Logger logger)
: client_(std::move(cli)),
  service_name_(std::move(service_name)),
  logger_(logger)
{
  ts_lib_ = rclcpp::get_typesupport_library(
    service_event_type, "rosidl_typesupport_cpp");

  ts_ = rclcpp::get_typesupport_handle(
    service_event_type,
    "rosidl_typesupport_cpp",
    *ts_lib_);

  auto message_ts_handle = get_message_typesupport_handle(
    ts_,
    rosidl_typesupport_introspection_cpp::typesupport_identifier);

  message_members_ =
    reinterpret_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(
    message_ts_handle->data);
}

bool PlayerServiceClient::is_include_request_message(
  const rclcpp::SerializedMessage & message)
{
  auto type = get_msg_event_type(message);

  // Ignore response message
  if (type == service_msgs::msg::ServiceEventInfo::RESPONSE_SENT ||
    type == service_msgs::msg::ServiceEventInfo::RESPONSE_RECEIVED)
  {
    return false;
  }

  // Ignore metadata message
  if ((type == service_msgs::msg::ServiceEventInfo::REQUEST_SENT ||
    type == service_msgs::msg::ServiceEventInfo::REQUEST_RECEIVED) &&
    client_side_type_ == introspection_type::METADATA)
  {
    return false;
  }

  if (client_side_type_ == introspection_type::UNKNOW &&
    type == service_msgs::msg::ServiceEventInfo::REQUEST_SENT)
  {
    if (message.size() <= rosbag2_cpp::get_serialization_size_for_service_metadata_event()) {
      client_side_type_ = introspection_type::METADATA;
      RCUTILS_LOG_WARN_ONCE_NAMED(
        ROSBAG2_TRANSPORT_PACKAGE_NAME,
        "The configuration of introspection for '%s' client is metadata !",
        service_name_.c_str());
      return false;
    } else {
      client_side_type_ = introspection_type::CONTENTS;
    }
  }
  if (service_side_type_ == introspection_type::UNKNOW &&
    type == service_msgs::msg::ServiceEventInfo::REQUEST_RECEIVED)
  {
    if (message.size() <= rosbag2_cpp::get_serialization_size_for_service_metadata_event()) {
      service_side_type_ = introspection_type::METADATA;
      RCUTILS_LOG_WARN_ONCE_NAMED(
        ROSBAG2_TRANSPORT_PACKAGE_NAME,
        "The configuration of introspection for '%s' service is metadata !",
        service_name_.c_str());
      return false;
    } else {
      service_side_type_ = introspection_type::CONTENTS;
    }
  }

  // If there are request send info and request receive info, only send request send info.
  if (client_side_type_ == introspection_type::CONTENTS &&
    service_side_type_ == introspection_type::CONTENTS &&
    type == service_msgs::msg::ServiceEventInfo::REQUEST_RECEIVED)
  {
    return false;
  }

  return true;
}

void PlayerServiceClient::async_send_request(const rclcpp::SerializedMessage & message)
{
  int ret = RMW_RET_OK;

  {
    auto ros_message = std::make_unique<uint8_t[]>(message_members_->size_of_);

    message_members_->init_function(
      ros_message.get(), rosidl_runtime_cpp::MessageInitialization::ZERO);

    ret = rmw_deserialize(&message.get_rcl_serialized_message(), ts_, ros_message.get());
    if (ret == RMW_RET_OK) {
      if (client_->service_is_ready()) {
        // members_[0]: info, members_[1]: request, members_[2]: response
        auto request_offset = message_members_->members_[1].offset_;
        auto request_addr = reinterpret_cast<size_t>(ros_message.get()) + request_offset;
        client_->async_send_request(
          reinterpret_cast<void *>(*reinterpret_cast<size_t *>(request_addr)));
      } else {
        RCLCPP_ERROR(
          logger_, "Service request hasn't been sent. The '%s' service isn't ready !",
          service_name_.c_str());
      }
    }

    message_members_->fini_function(ros_message.get());
  }

  if (ret != RMW_RET_OK) {
    throw std::runtime_error(
            "Failed to deserialize service event message for " + service_name_ + " !");
  }
}

uint8_t PlayerServiceClient::get_msg_event_type(
  const rclcpp::SerializedMessage & message)
{
  auto msg = service_msgs::msg::ServiceEventInfo();

  const rosidl_message_type_support_t * type_support_info =
    rosidl_typesupport_cpp::
    get_message_type_support_handle<service_msgs::msg::ServiceEventInfo>();
  if (type_support_info == nullptr) {
    throw std::runtime_error(
            "Failed to get message type support handle of service event info !");
  }

  auto ret = rmw_deserialize(
    &message.get_rcl_serialized_message(),
    type_support_info,
    reinterpret_cast<void *>(&msg));
  if (ret != RMW_RET_OK) {
    throw std::runtime_error("Failed to deserialize message !");
  }

  return msg.event_type;
}

}  // namespace rosbag2_transport
