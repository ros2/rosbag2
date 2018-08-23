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

#ifndef ROSBAG2__RAW_SUBSCRIPTION_HPP_
#define ROSBAG2__RAW_SUBSCRIPTION_HPP_

#include <memory>
#include <string>

#include "rclcpp/subscription.hpp"
#include "rclcpp/any_subscription_callback.hpp"

namespace rosbag2
{

class RawSubscription : public rclcpp::SubscriptionBase
{
  // TODO(Martin-Idel-SI): Do we need this?
  friend class rclcpp::node_interfaces::NodeTopicsInterface;

public:
  RCLCPP_SMART_PTR_DEFINITIONS(RawSubscription)

  RawSubscription(
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
    callback_(callback)
  {}

  std::shared_ptr<void> create_message() override
  {
    return borrow_serialized_message(0);
  }

  std::shared_ptr<rcl_serialized_message_t> create_serialized_message() override
  {
    return borrow_serialized_message(0);
  }


  void handle_message(
    std::shared_ptr<void> & message, const rmw_message_info_t & message_info) override
  {
    (void) message_info;
    auto typed_message = std::static_pointer_cast<rcutils_char_array_t>(message);
    callback_(typed_message);
  }

  void return_message(std::shared_ptr<void> & message) override
  {
    auto typed_message = std::static_pointer_cast<rcutils_char_array_t>(message);
    message.reset();
  }

  void return_serialized_message(std::shared_ptr<rcl_serialized_message_t> & message) override
  {
    message.reset();
  }

  void handle_intra_process_message(
    rcl_interfaces::msg::IntraProcessMessage & ipm,
    const rmw_message_info_t & message_info) override
  {
    (void) ipm;
    (void) message_info;
    throw std::runtime_error("Intra process is not supported");
  }

  const std::shared_ptr<rcl_subscription_t>
  get_intra_process_subscription_handle() const override
  {
    return nullptr;
  }

private:
  RCLCPP_DISABLE_COPY(RawSubscription)

  std::shared_ptr<rcl_serialized_message_t> borrow_serialized_message(size_t capacity)
  {
    auto message = new rcl_serialized_message_t;
    *message = rmw_get_zero_initialized_serialized_message();
    // TODO(Martin-Idel-SI): make class member
    auto allocator = new rcutils_allocator_t;
    *allocator = rcutils_get_default_allocator();
    auto init_return = rmw_serialized_message_init(message, capacity, allocator);
    if (init_return != RCL_RET_OK) {
      rclcpp::exceptions::throw_from_rcl_error(init_return);
    }

    auto serialized_msg = std::shared_ptr<rcl_serialized_message_t>(message,
        [](rmw_serialized_message_t * msg) {
          auto fini_return = rmw_serialized_message_fini(msg);
          delete msg;
          if (fini_return != RCL_RET_OK) {
            RCUTILS_LOG_ERROR_NAMED(
              "rosbag2",
              "failed to destroy serialized message: %s", rcl_get_error_string_safe());
          }
        });

    return serialized_msg;
  }

  std::function<void(std::shared_ptr<rcutils_char_array_t>)> callback_;
};

}  // namespace rosbag2

#endif  // ROSBAG2__RAW_SUBSCRIPTION_HPP_
