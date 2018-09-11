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

#include "rosbag2/logging.hpp"

namespace rosbag2
{

GenericSubscription::GenericSubscription(
  std::shared_ptr<rcl_node_t> node_handle,
  const rosidl_message_type_support_t & ts,
  const std::string & topic_name,
  std::function<void(std::shared_ptr<rcutils_char_array_t>)> callback)
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
  (void) message_info;
  auto typed_message = std::static_pointer_cast<rcutils_char_array_t>(message);
  callback_(typed_message);
}

void GenericSubscription::return_message(std::shared_ptr<void> & message)
{
  auto typed_message = std::static_pointer_cast<rcutils_char_array_t>(message);
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
  (void) ipm;
  (void) message_info;
  throw std::runtime_error("Intra process is not supported");
}

const std::shared_ptr<rcl_subscription_t>
GenericSubscription::get_intra_process_subscription_handle() const
{
  return nullptr;
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
          ROSBAG2_LOG_ERROR_STREAM(
            "Failed to destroy serialized message: " << rcl_get_error_string_safe());
        }
      });

  return serialized_msg;
}

}  // namespace rosbag2
