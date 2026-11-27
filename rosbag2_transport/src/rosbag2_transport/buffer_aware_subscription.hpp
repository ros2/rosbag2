// Copyright 2026 Open Source Robotics Foundation, Inc.
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

#ifndef ROSBAG2_TRANSPORT__BUFFER_AWARE_SUBSCRIPTION_HPP_
#define ROSBAG2_TRANSPORT__BUFFER_AWARE_SUBSCRIPTION_HPP_

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/clock.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rclcpp/subscription_base.hpp"
#include "rclcpp/subscription_options.hpp"
#include "rclcpp/typesupport_helpers.hpp"
#include "rcpputils/shared_library.hpp"
#include "rmw/error_handling.h"
#include "rmw/rmw.h"
#include "rosidl_buffer/buffer.hpp"
#include "rosbag2_cpp/types/introspection_message.hpp"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"

namespace rosbag2_transport
{

/// True if a message type has a rosidl::Buffer field, directly or in a nested message. Only such
/// types can carry a buffer backend payload; for every other type a generic subscription is
/// equivalent and cheaper. Takes the introspection type support of the message.
inline bool message_type_has_buffer_field(const rosidl_message_type_support_t * introspection_ts)
{
  namespace introspection = rosidl_typesupport_introspection_cpp;
  if (!introspection_ts || !introspection_ts->data) {
    return false;
  }
  const auto * members = static_cast<const introspection::MessageMembers *>(
    introspection_ts->data);
  for (uint32_t i = 0; i < members->member_count_; ++i) {
    const introspection::MessageMember & member = members->members_[i];
    if (member.is_rosidl_buffer_) {
      return true;
    }
    if (member.type_id_ == introspection::ROS_TYPE_MESSAGE &&
      message_type_has_buffer_field(member.members_))
    {
      return true;
    }
  }
  return false;
}

/// True if a member is a nested message whose introspection members are available.
inline bool is_nested_message(
  const rosidl_typesupport_introspection_cpp::MessageMember & member)
{
  return member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE &&
         member.members_ && member.members_->data;
}

/// True if a received message holds a non-CPU rosidl::Buffer with no elements, at any depth.
///
/// A buffer backend can lose the payload of a message that a subscriber is too slow to import:
/// the CUDA backend, for example, recycles a block 100 ms after publishing, and a subscriber that
/// imports it later gets an empty buffer but the message is still delivered. A non-CPU buffer
/// of size zero never occurs for a payload that arrived intact, whereas an empty CPU buffer is a
/// legitimate empty array, so only non-CPU buffers count. Takes the message and the introspection
/// members of its type.
inline bool message_has_stale_buffer(
  const void * message,
  const rosidl_typesupport_introspection_cpp::MessageMembers * members)
{
  namespace introspection = rosidl_typesupport_introspection_cpp;
  if (!message || !members) {
    return false;
  }
  for (uint32_t i = 0; i < members->member_count_; ++i) {
    const introspection::MessageMember & member = members->members_[i];
    const void * field = static_cast<const uint8_t *>(message) + member.offset_;
    if (member.is_rosidl_buffer_) {
      const auto * buffer = static_cast<const rosidl::Buffer<uint8_t> *>(field);
      if (buffer->size() == 0 && buffer->get_backend_type() != "cpu") {
        return true;
      }
    } else if (is_nested_message(member)) {
      const auto * nested = static_cast<const introspection::MessageMembers *>(
        member.members_->data);
      if (!member.is_array_) {
        if (message_has_stale_buffer(field, nested)) {
          return true;
        }
      } else if (member.size_function && member.get_const_function) {
        const size_t count = member.size_function(field);
        for (size_t j = 0; j < count; ++j) {
          if (message_has_stale_buffer(member.get_const_function(field, j), nested)) {
            return true;
          }
        }
      }
    }
  }
  return false;
}

/// A subscription that takes messages through the typed (deserialized) path, so that messages
/// delivered through a non-CPU buffer backend (for example CUDA) are received as live backend
/// buffers, and then serializes each message itself.
///
/// rclcpp::GenericSubscription can only take serialized messages, and the RMW rejects serialized
/// takes of messages that arrived through a buffer backend because their bytes would be a buffer
/// descriptor. Taking the message as a type-erased ROS message (allocated from introspection
/// metadata) and serializing it here lets the publisher skip its CPU conversion and send, and
/// moves the device-to-host copy onto the thread running this subscription.
class BufferAwareSubscription : public rclcpp::SubscriptionBase
{
public:
  using Callback = std::function<void (
        std::shared_ptr<const rclcpp::SerializedMessage>, const rclcpp::MessageInfo &)>;
  /// Called with the running total each time a message is dropped because a buffer was stale.
  using StaleMessageCallback = std::function<void (uint64_t)>;

  BufferAwareSubscription(
    rclcpp::node_interfaces::NodeBaseInterface * node_base,
    const std::string & topic_name,
    const std::string & topic_type,
    const rclcpp::QoS & qos,
    const rclcpp::SubscriptionOptions & options,
    Callback callback,
    StaleMessageCallback on_stale_message = nullptr)
  : BufferAwareSubscription(
      node_base, load_type_supports(topic_type), topic_name, qos, options, std::move(callback),
      std::move(on_stale_message))
  {}

  /// Number of messages dropped because a backend buffer arrived empty (stale).
  uint64_t stale_messages_dropped() const
  {
    return stale_messages_dropped_.load();
  }

  ~BufferAwareSubscription() override = default;

  std::shared_ptr<void> create_message() override
  {
    auto allocator = rcutils_get_default_allocator();
    auto introspection_message = rosbag2_cpp::allocate_introspection_message(
      type_supports_->introspection_ts, &allocator);
    // Aliasing constructor: keeps the introspection message (and its deleter) alive for as long
    // as the returned pointer is.
    return std::shared_ptr<void>(introspection_message, introspection_message->message);
  }

  std::shared_ptr<rclcpp::SerializedMessage> create_serialized_message() override
  {
    return std::make_shared<rclcpp::SerializedMessage>(0);
  }

  void handle_message(
    std::shared_ptr<void> & message, const rclcpp::MessageInfo & message_info) override
  {
    // Never record a message whose backend buffer arrived empty: it would be written as a valid
    // message with its payload silently missing. Drop it and report it as lost instead.
    const auto * members =
      static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(
      type_supports_->introspection_ts->data);
    if (message_has_stale_buffer(message.get(), members)) {
      const uint64_t total = ++stale_messages_dropped_;
      RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger("rosbag2_transport"), warn_clock_, 5000,
        "Dropped messages on topic '%s' whose buffer backend payload arrived empty because the "
        "recorder was too slow to import it (%lu dropped so far). Reduce the load on the "
        "recorder or the number of buffer-aware topics.",
        get_topic_name(), static_cast<unsigned long>(total));  // NOLINT(runtime/int)
      if (on_stale_message_) {
        on_stale_message_(total);
      }
      return;
    }
    auto serialized = std::make_shared<rclcpp::SerializedMessage>(0);
    const rmw_ret_t ret = rmw_serialize(
      message.get(), type_supports_->rmw_ts, &serialized->get_rcl_serialized_message());
    if (ret != RMW_RET_OK) {
      RCLCPP_ERROR(
        rclcpp::get_logger("rosbag2_transport"),
        "Failed to serialize message received through a buffer backend: %s",
        rmw_get_error_string().str);
      rmw_reset_error();
      return;
    }
    callback_(serialized, message_info);
  }

  void handle_serialized_message(
    const std::shared_ptr<rclcpp::SerializedMessage> &, const rclcpp::MessageInfo &) override
  {
    throw rclcpp::exceptions::UnimplementedError(
            "handle_serialized_message is not implemented for BufferAwareSubscription");
  }

  void handle_loaned_message(void *, const rclcpp::MessageInfo &) override
  {
    throw rclcpp::exceptions::UnimplementedError(
            "handle_loaned_message is not implemented for BufferAwareSubscription");
  }

  void return_message(std::shared_ptr<void> & message) override
  {
    message.reset();
  }

  void return_serialized_message(std::shared_ptr<rclcpp::SerializedMessage> & message) override
  {
    message.reset();
  }

  rclcpp::dynamic_typesupport::DynamicMessageType::SharedPtr
  get_shared_dynamic_message_type() override
  {
    throw rclcpp::exceptions::UnimplementedError("dynamic types are not supported");
  }

  rclcpp::dynamic_typesupport::DynamicMessage::SharedPtr
  get_shared_dynamic_message() override
  {
    throw rclcpp::exceptions::UnimplementedError("dynamic types are not supported");
  }

  rclcpp::dynamic_typesupport::DynamicSerializationSupport::SharedPtr
  get_shared_dynamic_serialization_support() override
  {
    throw rclcpp::exceptions::UnimplementedError("dynamic types are not supported");
  }

  rclcpp::dynamic_typesupport::DynamicMessage::SharedPtr
  create_dynamic_message() override
  {
    throw rclcpp::exceptions::UnimplementedError("dynamic types are not supported");
  }

  void return_dynamic_message(rclcpp::dynamic_typesupport::DynamicMessage::SharedPtr &) override
  {
    throw rclcpp::exceptions::UnimplementedError("dynamic types are not supported");
  }

  void handle_dynamic_message(
    const rclcpp::dynamic_typesupport::DynamicMessage::SharedPtr &,
    const rclcpp::MessageInfo &) override
  {
    throw rclcpp::exceptions::UnimplementedError("dynamic types are not supported");
  }

private:
  struct TypeSupports
  {
    std::shared_ptr<rcpputils::SharedLibrary> rmw_lib;
    const rosidl_message_type_support_t * rmw_ts{nullptr};
    std::shared_ptr<rcpputils::SharedLibrary> introspection_lib;
    const rosidl_message_type_support_t * introspection_ts{nullptr};
  };

  static std::shared_ptr<TypeSupports> load_type_supports(const std::string & topic_type)
  {
    auto supports = std::make_shared<TypeSupports>();
    supports->rmw_lib = rclcpp::get_typesupport_library(topic_type, "rosidl_typesupport_cpp");
    supports->rmw_ts = rclcpp::get_message_typesupport_handle(
      topic_type, "rosidl_typesupport_cpp", *supports->rmw_lib);
    supports->introspection_lib = rclcpp::get_typesupport_library(
      topic_type, "rosidl_typesupport_introspection_cpp");
    supports->introspection_ts = rclcpp::get_message_typesupport_handle(
      topic_type, "rosidl_typesupport_introspection_cpp", *supports->introspection_lib);
    return supports;
  }

  BufferAwareSubscription(
    rclcpp::node_interfaces::NodeBaseInterface * node_base,
    std::shared_ptr<TypeSupports> type_supports,
    const std::string & topic_name,
    const rclcpp::QoS & qos,
    const rclcpp::SubscriptionOptions & options,
    Callback callback,
    StaleMessageCallback on_stale_message)
  : rclcpp::SubscriptionBase(
      node_base,
      *type_supports->rmw_ts,
      topic_name,
      options.to_rcl_subscription_options(qos),
      options.event_callbacks,
      options.use_default_callbacks,
      rclcpp::DeliveredMessageKind::ROS_MESSAGE),
    type_supports_(std::move(type_supports)),
    callback_(std::move(callback)),
    on_stale_message_(std::move(on_stale_message))
  {}

  std::shared_ptr<TypeSupports> type_supports_;
  Callback callback_;
  StaleMessageCallback on_stale_message_;
  std::atomic<uint64_t> stale_messages_dropped_{0};
  rclcpp::Clock warn_clock_{RCL_STEADY_TIME};
};

}  // namespace rosbag2_transport

#endif  // ROSBAG2_TRANSPORT__BUFFER_AWARE_SUBSCRIPTION_HPP_
