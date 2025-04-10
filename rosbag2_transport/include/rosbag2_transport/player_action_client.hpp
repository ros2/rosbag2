// Copyright 2025 Sony Group Corporation.
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

#ifndef ROSBAG2_TRANSPORT__PLAYER_ACTION_CLIENT_HPP_
#define ROSBAG2_TRANSPORT__PLAYER_ACTION_CLIENT_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "rcl/types.h"
#include "rclcpp_action/generic_client.hpp"
#include <rclcpp_action/types.hpp>
#include "rclcpp/rclcpp.hpp"

#include "rosbag2_cpp/action_utils.hpp"

namespace rosbag2_transport
{

class PlayerActionClient final
{
public:
  using GoalID = rclcpp_action::GoalUUID;
  using IntrospectionMessageMembersPtr =
    const rosidl_typesupport_introspection_cpp::MessageMembers *;

  enum class ServiceInterfaceInAction
  {
    SEND_GOAL_SERVICE,
    CANCEL_GOAL_SERVICE,
    GET_RESULT_SERVICE
  };

  enum class ServiceEventType
  {
    REQUEST_SENT = service_msgs::msg::ServiceEventInfo::REQUEST_SENT,
    REQUEST_RECEIVED = service_msgs::msg::ServiceEventInfo::REQUEST_RECEIVED,
    RESPONSE_SENT = service_msgs::msg::ServiceEventInfo::RESPONSE_SENT,
    RESPONSE_RECEIVED = service_msgs::msg::ServiceEventInfo::RESPONSE_RECEIVED
  };

  explicit
  PlayerActionClient(
    rclcpp_action::GenericClient::SharedPtr generic_client,
    std::string & action_name,
    const std::string & action_type,
    rclcpp::Logger logger);

  ~PlayerActionClient();

  const std::string & get_action_name();

  /// \brief Deserialize message to the type erased send goal service event
  /// \param message - Serialized message
  /// \return Shared pointer to the byte array with deserialized service event if success,
  /// otherwise nullptr
  std::shared_ptr<uint8_t[]> deserialize_send_goal_service_event(
    const rcl_serialized_message_t & message);

  /// \brief Deserialize message to the type erased cancel goal service event
  /// \param message - Serialized message
  /// \return Shared pointer to the byte array with deserialized service event if success,
  /// otherwise nullptr
  std::shared_ptr<uint8_t[]> deserialize_cancel_goal_service_event(
    const rcl_serialized_message_t & message);

  /// \brief Deserialize message to the type erased get result service event
  /// \param message - Serialized message
  /// \return Shared pointer to the byte array with deserialized service event if success,
  /// otherwise nullptr
  std::shared_ptr<uint8_t[]> deserialize_get_result_service_event(
    const rcl_serialized_message_t & message);

  /// \brief Extract the request data from the send_goal event message and asynchronously
  // send the extracted request data.
  void async_send_goal_request(
    const std::shared_ptr<uint8_t[]> & type_erased_send_goal_service_event);

  /// \brief Extract the request data from the cancel_goal event message and asynchronously
  // send the extracted request data.
  void async_send_cancel_request(
    const std::shared_ptr<uint8_t[]> & type_erased_cancel_goal_service_event);

  /// \brief Extract the request data from the get_result event message and asynchronously
  // send the extracted request data.
  void async_send_result_request(
    const std::shared_ptr<uint8_t[]> & type_erased_get_result_service_event);

  ServiceEventType get_service_event_type(
    const std::shared_ptr<uint8_t[]> & type_erased_service_event,
    ServiceInterfaceInAction service_type_in_action);

  rclcpp_action::GenericClient::SharedPtr generic_client()
  {
    return client_;
  }

  bool goal_handle_in_processing(const GoalID & goal_id);

private:
  rclcpp_action::GenericClient::SharedPtr client_;
  std::string action_name_;
  std::string action_type_;
  const rclcpp::Logger logger_;

  // Warning output when exceeding this value
  const size_t maximum_goal_handle_size_ = 100;

  // Note: The action_ts_lib_ shall be a member variable to make sure that library loaded
  // during the liveliness of the instance of this class, since we have raw pointers to its members.
  std::shared_ptr<rcpputils::SharedLibrary> action_ts_lib_;

  const rosidl_message_type_support_t * goal_service_event_type_ts_;
  IntrospectionMessageMembersPtr goal_service_request_type_members_;
  IntrospectionMessageMembersPtr goal_service_event_type_members_;

  const rosidl_message_type_support_t * cancel_service_event_type_ts_;
  IntrospectionMessageMembersPtr cancel_service_request_type_members_;
  IntrospectionMessageMembersPtr cancel_service_event_type_members_;

  const rosidl_message_type_support_t * result_service_event_type_ts_;
  IntrospectionMessageMembersPtr result_service_request_type_members_;
  IntrospectionMessageMembersPtr result_service_event_type_members_;

  rcutils_allocator_t allocator_ = rcutils_get_default_allocator();

  // Map to store recorded goal ID to goal ID mapping
  // If recorded_goal_to_goal_id_id_map_ has recorded the goal ID, it means the goal request has
  // already been sent.
  std::unordered_map<
    GoalID, GoalID, std::hash<GoalID>> recorded_goal_to_goal_id_id_map_;
  // Map to store goal ID to goal handle mapping
  // If goal_id_to_goal_handle_map_ has recorded the goal handle, it means the goal has been
  // accepted by the action server.
  std::unordered_map<
    GoalID, rclcpp_action::GenericClientGoalHandle::SharedPtr, std::hash<GoalID>>
  goal_id_to_goal_handle_map_;
  std::mutex goal_id_maps_mutex_;  // This mutex is used to protect above 2 maps.

  std::unordered_set<GoalID, std::hash<GoalID>> goal_ids_to_postpone_send_cancel_;
  std::mutex goal_ids_to_postpone_send_cancel_mutex_;

  std::unordered_set<GoalID, std::hash<GoalID>> goal_ids_to_postpone_send_result_;
  std::mutex goal_ids_to_postpone_send_result_mutex_;

  std::independent_bits_engine<
    std::default_random_engine, 8, unsigned int> random_bytes_generator_;

  std::shared_ptr<uint8_t[]>
  deserialize_service_event(
    const rosidl_message_type_support_t * service_event_type_support,
    IntrospectionMessageMembersPtr service_event_members,
    const rcl_serialized_message_t & message);

  bool is_request_service_event(
    const std::shared_ptr<uint8_t[]> & type_erased_service_event,
    IntrospectionMessageMembersPtr service_event_members);

  // Must be called after check_if_service_event_type_is_request
  bool get_goal_id_from_cancel_goal_service_event(
    const std::shared_ptr<uint8_t[]> & type_erased_cancel_goal_service_event, GoalID & goal_id);

  bool get_goal_id_from_get_result_service_event(
    const std::shared_ptr<uint8_t[]> & type_erased_get_result_service_event, GoalID & goal_id);

  rclcpp_action::GoalUUID generate_goal_id();
};
}  // namespace rosbag2_transport

#endif  // ROSBAG2_TRANSPORT__PLAYER_ACTION_CLIENT_HPP_
