// Copyright 2018, Bosch Software Innovations GmbH.
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

#include "generic_subscription.hpp"

#include <memory>
#include <string>

#include "rclcpp/any_subscription_callback.hpp"
#include "rclcpp/subscription.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"

#include "rosbag2/typesupport_helpers.hpp"
#include "rosbag2_transport/logging.hpp"

namespace rosbag2_transport
{

static std::shared_ptr<rmw_serialized_message_t>
get_initialized_serialized_message(size_t capacity)
{
  static rcutils_allocator_t rcutils_allocator_ = rcutils_get_default_allocator();

  auto msg = std::make_unique<rmw_serialized_message_t>();
  *msg = rmw_get_zero_initialized_serialized_message();
  auto ret = rmw_serialized_message_init(msg.get(), capacity, &rcutils_allocator_);
  if (ret != RCUTILS_RET_OK) {
    throw std::runtime_error("Error allocating resources for serialized message: " +
      std::string(rcutils_get_error_string().str));
  }

  auto serialized_message = std::shared_ptr<rmw_serialized_message_t>(msg.release(),
  [](rmw_serialized_message_t * msg) {
    int error = rmw_serialized_message_fini(msg);
    delete msg;
    if (error != RCUTILS_RET_OK) {
      RCUTILS_LOG_ERROR_NAMED("rosbag2_test_common", "Leaking memory. Error: %s",
      rcutils_get_error_string().str);
    }
  });

  return serialized_message;
}

GenericSubscription::GenericSubscription(
  std::shared_ptr<rcl_node_t> node_handle,
  const rosidl_message_type_support_t & ts,
  const std::string & topic_name,
  std::function<void(std::shared_ptr<rmw_serialized_message_t>)> callback)
: SubscriptionBase(
    node_handle,
    ts,
    topic_name,
    rcl_subscription_get_default_options(),
    true),
  default_allocator_(rcutils_get_default_allocator()),
  callback_(callback)
{}

std::shared_ptr<void> GenericSubscription::create_message()
{
  return create_serialized_message();
}

std::shared_ptr<rmw_serialized_message_t> GenericSubscription::create_serialized_message()
{
  return borrow_serialized_message(0);
}

void GenericSubscription::handle_message(
  std::shared_ptr<void> & message, const rmw_message_info_t & message_info)
{
  if (matches_any_intra_process_publishers_) {
    if (matches_any_intra_process_publishers_(&message_info.publisher_gid)) {
      // In this case, the message will be delivered via intra process and
      // we should ignore this copy of the message.
      return;
    }
  }

  auto typed_message = std::static_pointer_cast<rmw_serialized_message_t>(message);
  callback_(typed_message);
}

void GenericSubscription::return_message(std::shared_ptr<void> & message)
{
  auto typed_message = std::static_pointer_cast<rmw_serialized_message_t>(message);
  return_serialized_message(typed_message);
}

void GenericSubscription::return_serialized_message(
  std::shared_ptr<rmw_serialized_message_t> & message)
{
  message.reset();
}

void GenericSubscription::handle_intra_process_message(
  rcl_interfaces::msg::IntraProcessMessage & ipm,
  const rmw_message_info_t & message_info)
{
  (void)message_info;

  if (!get_intra_process_message_callback_) {
    throw std::runtime_error("Intra process is not supported");
  }

  MessageUniquePtr message;
  get_intra_process_message_callback_(
      ipm.publisher_id,
      ipm.message_sequence,
      intra_process_subscription_id_,
      message);

  if (!message) {
    ROSBAG2_TRANSPORT_LOG_ERROR_STREAM(
      "Intra process not successfull: probalby queue run over for " << get_topic_name());
    return;
  }
  
  auto serialized_message = get_initialized_serialized_message(0);
  auto error = rmw_serialize(
    message.get(),
    &get_message_type_support_handle(),
    serialized_message.get());
  if (error != RCL_RET_OK) {
    throw std::runtime_error("Failed to serialize");
  }

  callback_(serialized_message);
}

const std::shared_ptr<rcl_subscription_t>
GenericSubscription::get_intra_process_subscription_handle() const
{
  if (!get_intra_process_message_callback_) {
    return nullptr;
  }
  return intra_process_subscription_handle_;
}

std::shared_ptr<rmw_serialized_message_t>
GenericSubscription::borrow_serialized_message(size_t capacity)
{
  auto message = new rmw_serialized_message_t;
  *message = rmw_get_zero_initialized_serialized_message();
  auto init_return = rmw_serialized_message_init(message, capacity, &default_allocator_);
  if (init_return != RCL_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(init_return);
  }

  auto serialized_msg = std::shared_ptr<rmw_serialized_message_t>(message,
      [](rmw_serialized_message_t * msg) {
        auto fini_return = rmw_serialized_message_fini(msg);
        delete msg;
        if (fini_return != RCL_RET_OK) {
          ROSBAG2_TRANSPORT_LOG_ERROR_STREAM(
            "Failed to destroy serialized message: " << rcl_get_error_string().str);
        }
      });

  return serialized_msg;
}

}  // namespace rosbag2_transport
