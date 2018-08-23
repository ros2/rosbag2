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
public:
  RCLCPP_SMART_PTR_DEFINITIONS(RawSubscription)

  RawSubscription(
    std::shared_ptr<rcl_node_t> node_handle,
    const rosidl_message_type_support_t & ts,
    const std::string & topic_name,
    std::function<void(std::shared_ptr<rcutils_char_array_t>)> callback);

  std::shared_ptr<void> create_message() override;

  std::shared_ptr<rcl_serialized_message_t> create_serialized_message() override;


  void handle_message(
    std::shared_ptr<void> & message, const rmw_message_info_t & message_info) override;

  void return_message(std::shared_ptr<void> & message) override;

  void return_serialized_message(std::shared_ptr<rcl_serialized_message_t> & message) override;

  void handle_intra_process_message(
    rcl_interfaces::msg::IntraProcessMessage & ipm,
    const rmw_message_info_t & message_info) override;

  const std::shared_ptr<rcl_subscription_t>
  get_intra_process_subscription_handle() const override;

private:
  RCLCPP_DISABLE_COPY(RawSubscription)

  std::shared_ptr<rcl_serialized_message_t> borrow_serialized_message(size_t capacity);

  std::function<void(std::shared_ptr<rcutils_char_array_t>)> callback_;
};

}  // namespace rosbag2

#endif  // ROSBAG2__RAW_SUBSCRIPTION_HPP_
