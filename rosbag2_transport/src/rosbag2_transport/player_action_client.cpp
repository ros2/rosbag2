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

#include <memory>
#include <utility>

#include "rosbag2_transport/player_action_client.hpp"

#include "rosbag2_cpp/action_utils.hpp"

#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"

#include "logging.hpp"

using namespace std::chrono_literals;

namespace rosbag2_transport
{

PlayerActionClient::PlayerActionClient(
  rclcpp_action::GenericClient::SharedPtr generic_client,
  std::string & action_name,
  const std::string & action_type,
  rclcpp::Logger logger)
: client_(std::move(generic_client)),
  action_name_(action_name),
  action_type_(action_type),
  logger_(std::move(logger)),
  random_bytes_generator_(std::random_device{}())
{
  action_ts_lib_ =
    rclcpp::get_typesupport_library(action_type, "rosidl_typesupport_cpp");

  auto action_typesupport_handle = rclcpp::get_action_typesupport_handle(
    action_type, "rosidl_typesupport_cpp", *action_ts_lib_);

  auto goal_service_request_type_support_intro = get_message_typesupport_handle(
    action_typesupport_handle->goal_service_type_support->request_typesupport,
    rosidl_typesupport_introspection_cpp::typesupport_identifier);
  goal_service_request_type_members_ =
    static_cast<IntrospectionMessageMembersPtr>(goal_service_request_type_support_intro->data);
  if (goal_service_request_type_members_ == nullptr) {
    throw std::invalid_argument(
      "goal_service_request_type_members_ for `" + action_name_ + "` is nullptr");
  }

  goal_service_event_type_ts_ =
    action_typesupport_handle->goal_service_type_support->event_typesupport;
  auto goal_service_event_type_support_intro = get_message_typesupport_handle(
    goal_service_event_type_ts_,
    rosidl_typesupport_introspection_cpp::typesupport_identifier);
  goal_service_event_type_members_ =
    static_cast<IntrospectionMessageMembersPtr>(goal_service_event_type_support_intro->data);
  if (goal_service_event_type_members_ == nullptr) {
    throw std::invalid_argument(
      "goal_service_event_type_members_ for `" + action_name_ + "` is nullptr");
  }
  if (goal_service_event_type_members_->member_count_ != 3) {
    // members_[0]: service_info, members_[1]: request[<=1], members_[2]: response[<=1]
    std::stringstream ss;
    ss << "Expected 3 fields in the goal service introspection message, but got " <<
      goal_service_event_type_members_->member_count_;
    throw std::invalid_argument(ss.str());
  }

  auto cancel_service_request_type_support_intro = get_message_typesupport_handle(
    action_typesupport_handle->cancel_service_type_support->request_typesupport,
    rosidl_typesupport_introspection_cpp::typesupport_identifier);
  cancel_service_request_type_members_ =
    static_cast<IntrospectionMessageMembersPtr>(cancel_service_request_type_support_intro->data);
  if (cancel_service_request_type_members_ == nullptr) {
    throw std::invalid_argument(
      "cancel_service_request_type_members_ for `" + action_name_ + "` is nullptr");
  }

  cancel_service_event_type_ts_ =
    action_typesupport_handle->cancel_service_type_support->event_typesupport;
  auto cancel_service_event_type_support_intro = get_message_typesupport_handle(
    cancel_service_event_type_ts_,
    rosidl_typesupport_introspection_cpp::typesupport_identifier);
  cancel_service_event_type_members_ =
    static_cast<IntrospectionMessageMembersPtr>(cancel_service_event_type_support_intro->data);
  if (cancel_service_event_type_members_ == nullptr) {
    throw std::invalid_argument(
      "cancel_service_event_type_members_ for `" + action_name_ + "` is nullptr");
  }
  if (cancel_service_event_type_members_->member_count_ != 3) {
    // members_[0]: service_info, members_[1]: request[<=1], members_[2]: response[<=1]
    std::stringstream ss;
    ss << "Expected 3 fields in the cancel service introspection message, but got " <<
      cancel_service_event_type_members_->member_count_;
    throw std::invalid_argument(ss.str());
  }

  auto result_service_request_type_support_intro = get_message_typesupport_handle(
    action_typesupport_handle->result_service_type_support->request_typesupport,
    rosidl_typesupport_introspection_cpp::typesupport_identifier);
  result_service_request_type_members_ =
    static_cast<IntrospectionMessageMembersPtr>(result_service_request_type_support_intro->data);
  if (result_service_request_type_members_ == nullptr) {
    throw std::invalid_argument(
      "result_service_request_type_members_ for `" + action_name_ + "` is nullptr");
  }

  result_service_event_type_ts_ =
    action_typesupport_handle->result_service_type_support->event_typesupport;
  auto result_service_event_type_support_intro = get_message_typesupport_handle(
    result_service_event_type_ts_,
    rosidl_typesupport_introspection_cpp::typesupport_identifier);
  result_service_event_type_members_ =
    static_cast<IntrospectionMessageMembersPtr>(result_service_event_type_support_intro->data);
  if (result_service_event_type_members_ == nullptr) {
    throw std::invalid_argument(
      "result_service_event_type_members_ for `" + action_name_ + "` is nullptr");
  }
  if (result_service_event_type_members_->member_count_ != 3) {
    // members_[0]: service_info, members_[1]: request[<=1], members_[2]: response[<=1]
    std::stringstream ss;
    ss << "Expected 3 fields in the result service introspection message, but got " <<
      result_service_event_type_members_->member_count_;
    throw std::invalid_argument(ss.str());
  }
}

PlayerActionClient::~PlayerActionClient()
{
  {
    std::lock_guard<std::mutex> lock(goal_id_maps_mutex_);
    recorded_goal_to_goal_id_id_map_.clear();
    goal_id_to_goal_handle_map_.clear();
  }
  client_->async_cancel_all_goals();

  {
    std::lock_guard<std::mutex> lock(goal_ids_to_postpone_send_cancel_mutex_);
    goal_ids_to_postpone_send_cancel_.clear();
  }

  {
    std::lock_guard<std::mutex> lock(goal_ids_to_postpone_send_result_mutex_);
    goal_ids_to_postpone_send_result_.clear();
  }
}

const std::string & PlayerActionClient::get_action_name()
{
  return action_name_;
}

std::shared_ptr<uint8_t[]>
PlayerActionClient::deserialize_service_event(
  const rosidl_message_type_support_t * service_event_type_support,
  IntrospectionMessageMembersPtr service_event_members,
  const rcl_serialized_message_t & message)
{
  auto type_erased_service_event = std::shared_ptr<uint8_t[]>(
    new uint8_t[service_event_members->size_of_],
    [fini_function = service_event_members->fini_function](uint8_t * msg) {
      fini_function(msg);
      delete[] msg;
    });

  service_event_members->init_function(
    type_erased_service_event.get(), rosidl_runtime_cpp::MessageInitialization::ZERO);

  rmw_ret_t ret =
    rmw_deserialize(&message, service_event_type_support, type_erased_service_event.get());
  if (ret != RMW_RET_OK) {  // Failed to deserialize service event message
    type_erased_service_event.reset();
  }
  return type_erased_service_event;
}

std::shared_ptr<uint8_t[]>
PlayerActionClient::deserialize_send_goal_service_event(const rcl_serialized_message_t & message)
{
  return deserialize_service_event(
    goal_service_event_type_ts_, goal_service_event_type_members_, message);
}

std::shared_ptr<uint8_t[]>
PlayerActionClient::deserialize_cancel_goal_service_event(const rcl_serialized_message_t & message)
{
  return deserialize_service_event(
    cancel_service_event_type_ts_, cancel_service_event_type_members_, message);
}

std::shared_ptr<uint8_t[]>
PlayerActionClient::deserialize_get_result_service_event(const rcl_serialized_message_t & message)
{
  return deserialize_service_event(
    result_service_event_type_ts_, result_service_event_type_members_, message);
}

PlayerActionClient::ServiceEventType
PlayerActionClient::get_service_event_type(
  const std::shared_ptr<uint8_t[]> & type_erased_service_event,
  ServiceInterfaceInAction service_type_in_action)
{
  IntrospectionMessageMembersPtr service_event_type_members;
  switch (service_type_in_action) {
    case ServiceInterfaceInAction::SEND_GOAL_SERVICE:
      service_event_type_members = goal_service_event_type_members_;
      break;
    case ServiceInterfaceInAction::CANCEL_GOAL_SERVICE:
      service_event_type_members = cancel_service_event_type_members_;
      break;
    case ServiceInterfaceInAction::GET_RESULT_SERVICE:
      service_event_type_members = result_service_event_type_members_;
      break;
  }
  // members_[0]: service_info, members_[1]: request[<=1], members_[2]: response[<=1]
  const auto & service_info_member = service_event_type_members->members_[0];
  auto service_event_info_ptr =
    reinterpret_cast<service_msgs::msg::ServiceEventInfo *>(
    type_erased_service_event.get() + service_info_member.offset_);

  switch (service_event_info_ptr->event_type) {
    case service_msgs::msg::ServiceEventInfo::REQUEST_SENT:
      return ServiceEventType::REQUEST_SENT;
    case service_msgs::msg::ServiceEventInfo::REQUEST_RECEIVED:
      return ServiceEventType::REQUEST_RECEIVED;
    case service_msgs::msg::ServiceEventInfo::RESPONSE_SENT:
      return ServiceEventType::RESPONSE_SENT;
    case service_msgs::msg::ServiceEventInfo::RESPONSE_RECEIVED:
      return ServiceEventType::RESPONSE_RECEIVED;
    default:
      // Never go here
      throw std::out_of_range("Invalid service event type");
  }
}

bool
PlayerActionClient::is_request_service_event(
  const std::shared_ptr<uint8_t[]> & type_erased_service_event,
  IntrospectionMessageMembersPtr service_event_members)
{
  // members_[0]: service_info, members_[1]: request[<=1], members_[2]: response[<=1]
  const auto & request_member = service_event_members->members_[0];
  auto service_event_info_ptr =
    reinterpret_cast<service_msgs::msg::ServiceEventInfo *>(
    type_erased_service_event.get() + request_member.offset_);

  if (service_event_info_ptr->event_type == service_msgs::msg::ServiceEventInfo::REQUEST_SENT ||
    service_event_info_ptr->event_type == service_msgs::msg::ServiceEventInfo::REQUEST_RECEIVED)
  {
    return true;
  }

  return false;
}

void
PlayerActionClient::async_send_goal_request(
  const std::shared_ptr<uint8_t[]> & type_erased_send_goal_service_event)
{
  // members_[0]: service_info, members_[1]: request[<=1], members_[2]: response[<=1]

  // Confirm that the service event type is REQUEST_SENT or REQUEST_RECEIVED.
  if (!is_request_service_event(
      type_erased_send_goal_service_event, goal_service_event_type_members_))
  {
    return;
  }

  const auto & request_member = goal_service_event_type_members_->members_[1];
  void * request_sequence_ptr = type_erased_send_goal_service_event.get() + request_member.offset_;
  if (request_member.size_function(request_sequence_ptr) > 0) {
    void * request_ptr = request_member.get_function(request_sequence_ptr, 0);

    // Use new gold id to replace the old one in recorded request message.
    // The first part of request is UUID (goal id)
    auto uuid = reinterpret_cast<unique_identifier_msgs::msg::UUID *>(request_ptr);
    auto recorded_goal_id = uuid->uuid;
    uuid->uuid = generate_goal_id();

    auto result_callback =
      [this, recorded_goal_id]
      (const rclcpp_action::GenericClientGoalHandle::WrappedResult & result) {
        // Regardless of the return result, this Goal ID has already been completed
        // So remove corresponding goal handle.
        auto goal_id = result.goal_id;
        std::lock_guard<std::mutex> lock(goal_id_maps_mutex_);
        recorded_goal_to_goal_id_id_map_.erase(recorded_goal_id);
        goal_id_to_goal_handle_map_.erase(goal_id);
      };

    auto send_goal_options = rclcpp_action::GenericClient::SendGoalOptions();
    auto goal_response_callback =
      [this, recorded_goal_id, result_callback]
      (rclcpp_action::GenericClientGoalHandle::SharedPtr goal_handle) {
        // Only goal is accepted, then register the goal handle
        if (goal_handle) {
          auto goal_id = goal_handle->get_goal_id();

          // Check if the Goal ID needs to get result.
          bool send_result_request = false;
          {
            std::lock_guard<std::mutex> lock(goal_ids_to_postpone_send_result_mutex_);
            if (goal_ids_to_postpone_send_result_.count(goal_id) > 0) {
              goal_ids_to_postpone_send_result_.erase(goal_id);
              send_result_request = true;
            }
          }
          if (send_result_request) {
            client_->async_get_result(goal_handle, result_callback);
          }

          // Check if the Goal ID needs to be canceled.
          bool send_cancel_request = false;
          {
            std::lock_guard<std::mutex> lock(goal_ids_to_postpone_send_cancel_mutex_);
            if (goal_ids_to_postpone_send_cancel_.count(goal_id) > 0) {
              goal_ids_to_postpone_send_cancel_.erase(goal_id);
              send_cancel_request = true;
            }
          }
          if (send_cancel_request) {
            client_->async_cancel_goal(goal_handle);
            // If the goal id also exists in goal_ids_to_postpone_send_result, remove it.
            {
              std::lock_guard<std::mutex> lock(goal_ids_to_postpone_send_result_mutex_);
              if (goal_ids_to_postpone_send_result_.count(goal_id) > 0) {
                goal_ids_to_postpone_send_result_.erase(goal_id);
              }
            }
          }

          {
            std::lock_guard<std::mutex> lock(goal_id_maps_mutex_);
            goal_id_to_goal_handle_map_[goal_id] = goal_handle;
            if (goal_id_to_goal_handle_map_.size() > maximum_goal_handle_size_) {
              RCLCPP_WARN(
                logger_,
                "For action \"%s\", the number of goal handles is over %lu, which may "
                "cause memory leak.", action_name_.c_str(), maximum_goal_handle_size_);
            }
          }
        } else {
          // Goal was rejected by server
          {
            std::lock_guard<std::mutex> lock(goal_id_maps_mutex_);
            recorded_goal_to_goal_id_id_map_.erase(recorded_goal_id);
          }
        }
      };

    send_goal_options.goal_response_callback = goal_response_callback;
    send_goal_options.result_callback = result_callback;

    {
      std::lock_guard<std::mutex> lock(goal_id_maps_mutex_);
      recorded_goal_to_goal_id_id_map_[recorded_goal_id] = uuid->uuid;
    }

    client_->async_send_goal(request_ptr, send_goal_options);
  } else {
    // No service request in the service event.
    RCLCPP_WARN(
      logger_,
      "Can't send goal request since the configuration of introspection for '%s' "
      "action was metadata !", action_name_.c_str());
  }
}

bool
PlayerActionClient::get_goal_id_from_cancel_goal_service_event(
  const std::shared_ptr<uint8_t[]> & type_erased_cancel_goal_service_event, GoalID & goal_id)
{
  // members_[0]: service_info, members_[1]: request[<=1], members_[2]: response[<=1]
  const auto & request_member = cancel_service_event_type_members_->members_[1];
  void * request_sequence_ptr =
    type_erased_cancel_goal_service_event.get() + request_member.offset_;
  if (request_member.size_function(request_sequence_ptr) > 0) {
    void * request_ptr = request_member.get_function(request_sequence_ptr, 0);
    auto cancel_request = static_cast<action_msgs::srv::CancelGoal::Request *>(request_ptr);
    goal_id = cancel_request->goal_info.goal_id.uuid;
    return true;
  } else {
    // No request data
    return false;
  }
}

bool
PlayerActionClient::get_goal_id_from_get_result_service_event(
  const std::shared_ptr<uint8_t[]> & type_erased_get_result_service_event, GoalID & goal_id)
{
  // members_[0]: service_info, members_[1]: request[<=1], members_[2]: response[<=1]
  const auto & request_member = result_service_event_type_members_->members_[1];
  void * request_sequence_ptr = type_erased_get_result_service_event.get() + request_member.offset_;
  if (request_member.size_function(request_sequence_ptr) > 0) {
    void * request_ptr = request_member.get_function(request_sequence_ptr, 0);
    auto result_request = static_cast<rclcpp_action::GoalUUID *>(request_ptr);
    goal_id = *result_request;
    return true;
  } else {
    // No request data
    return false;
  }
}

void
PlayerActionClient::async_send_cancel_request(
  const std::shared_ptr<uint8_t[]> & type_erased_cancel_goal_service_event)
{
  // Confirm that the service event type is REQUEST_SENT or REQUEST_RECEIVED.
  if (!is_request_service_event(
      type_erased_cancel_goal_service_event, cancel_service_event_type_members_))
  {
    return;
  }

  GoalID recorded_goal_id;
  if (!get_goal_id_from_cancel_goal_service_event(
    type_erased_cancel_goal_service_event, recorded_goal_id))
  {
    RCLCPP_WARN(
      logger_,
      "Can't send cancel goal request since the configuration of introspection for '%s' "
      "action was metadata !", action_name_.c_str());
    return;
  }

  rclcpp_action::GenericClientGoalHandle::SharedPtr goal_handle = nullptr;
  {
    std::lock_guard<std::mutex> lock(goal_id_maps_mutex_);

    // Check if goal request was sent
    if (recorded_goal_to_goal_id_id_map_.count(recorded_goal_id) > 0) {
      auto it = goal_id_to_goal_handle_map_.find(
        recorded_goal_to_goal_id_id_map_[recorded_goal_id]);
      // Check if goal has been accepted
      if (it != goal_id_to_goal_handle_map_.end()) {
        goal_handle = it->second;
      }
    } else {
      RCLCPP_WARN(
        logger_,
        "Can't send cancel goal request before send goal request for '%s' action !",
        action_name_.c_str());
      return;
    }
  }

  if (goal_handle) {
    client_->async_cancel_goal(goal_handle);
  } else {
    // Record this Goal ID, and process the cancel request after the goal is accepted
    {
      std::lock_guard<std::mutex> lock(goal_ids_to_postpone_send_cancel_mutex_);
      goal_ids_to_postpone_send_cancel_.insert(recorded_goal_to_goal_id_id_map_[recorded_goal_id]);
    }
    RCLCPP_DEBUG(
      logger_,
      "For action \"%s\", postpone sending cancel_goal request since the goal "
      "may not be accepted yet.", action_name_.c_str());
  }
}

void
PlayerActionClient::async_send_result_request(
  const std::shared_ptr<uint8_t[]> & type_erased_get_result_service_event)
{
  // Confirm that the service event type is REQUEST_SENT or REQUEST_RECEIVED.
  if (!is_request_service_event(
      type_erased_get_result_service_event, result_service_event_type_members_))
  {
    return;
  }

  GoalID recorded_goal_id;
  if (!get_goal_id_from_get_result_service_event(type_erased_get_result_service_event,
      recorded_goal_id))
  {
    RCLCPP_WARN(
      logger_,
      "Can't send result request since the configuration of introspection for '%s' "
      "action was metadata !", action_name_.c_str());
    return;
  }

  rclcpp_action::GenericClientGoalHandle::SharedPtr goal_handle = nullptr;
  {
    std::lock_guard<std::mutex> lock(goal_id_maps_mutex_);

    // Check if goal request was sent
    if (recorded_goal_to_goal_id_id_map_.count(recorded_goal_id) > 0) {
      auto it = goal_id_to_goal_handle_map_.find(
        recorded_goal_to_goal_id_id_map_[recorded_goal_id]);
      // Check if goal has been accepted
      if (it != goal_id_to_goal_handle_map_.end()) {
        goal_handle = it->second;
      }
    } else {
      RCLCPP_WARN(
        logger_,
        "Can't send result request before send goal request for '%s' action !",
        action_name_.c_str());
      return;
    }
  }

  if (goal_handle) {
    auto result_callback =
      [this, recorded_goal_id]
      (const rclcpp_action::GenericClientGoalHandle::WrappedResult & result)
      {
        (void) result;
        std::lock_guard<std::mutex> lock(goal_id_maps_mutex_);
        goal_id_to_goal_handle_map_.erase(recorded_goal_to_goal_id_id_map_[recorded_goal_id]);
        recorded_goal_to_goal_id_id_map_.erase(recorded_goal_id);
      };

    client_->async_get_result(goal_handle, result_callback);
  } else {
    // Record this Goal ID, and process the result request after the goal is accepted
    {
      std::lock_guard<std::mutex> lock(goal_ids_to_postpone_send_result_mutex_);
      goal_ids_to_postpone_send_result_.insert(recorded_goal_id);
    }
    RCLCPP_DEBUG(
      logger_,
      "For action \"%s\", postpone sending get result request since the goal "
      "may not be accepted yet.", action_name_.c_str());
  }
}

bool PlayerActionClient::goal_handle_in_processing(const GoalID & goal_id)
{
  std::lock_guard<std::mutex> lock(goal_id_maps_mutex_);
  return goal_id_to_goal_handle_map_.count(goal_id) > 0;
}

// This implementation comes from rclcpp_action::ClientBase::generate_goal_id()
rclcpp_action::GoalUUID PlayerActionClient::generate_goal_id()
{
  rclcpp_action::GoalUUID goal_id;
  std::generate(
    goal_id.begin(), goal_id.end(),
    std::ref(random_bytes_generator_));
  return goal_id;
}
}  // namespace rosbag2_transport
