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

#ifndef ROSBAG2_TRANSPORT__GENERIC_SUBSCRIPTION_HPP_
#define ROSBAG2_TRANSPORT__GENERIC_SUBSCRIPTION_HPP_

#include <memory>
#include <string>

#include "rclcpp/any_subscription_callback.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/subscription.hpp"

namespace rosbag2_transport
{

/**
 * This class is an implementation of an rclcpp::Subscription for serialized messages whose topic
 * is not known at compile time (hence templating does not work).
 *
 * It does not support intra-process handling
 */
class GenericSubscription : public rclcpp::SubscriptionBase
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(GenericSubscription)

  /**
   * Constructor. In order to properly subscribe to a topic, this subscription needs to be added to
   * the node_topic_interface of the node passed into this constructor.
   *
   * \param node_base NodeBaseInterface pointer used in parts of the setup.
   * \param ts Type support handle
   * \param topic_name Topic name
   * \param callback Callback for new messages of serialized form
   */
  GenericSubscription(
    rclcpp::node_interfaces::NodeBaseInterface * node_base,
    const rosidl_message_type_support_t & ts,
    const std::string & topic_name,
    std::function<void(std::shared_ptr<rmw_serialized_message_t>)> callback);

  // Same as create_serialized_message() as the subscription is to serialized_messages only
  std::shared_ptr<void> create_message() override;

  std::shared_ptr<rmw_serialized_message_t> create_serialized_message() override;

  void handle_message(
    std::shared_ptr<void> & message, const rmw_message_info_t & message_info) override;

  void handle_loaned_message(void * loaned_message, const rmw_message_info_t & message_info) override;

  // Same as return_serialized_message() as the subscription is to serialized_messages only
  void return_message(std::shared_ptr<void> & message) override;

  void return_serialized_message(std::shared_ptr<rmw_serialized_message_t> & message) override;

  // Intra-process message handling is not supported by this publisher
  void handle_intra_process_message(
    rcl_interfaces::msg::IntraProcessMessage & ipm,
    const rmw_message_info_t & message_info) override;

  // Intra-process message handling is not supported by this publisher (returns nullptr always)
  const std::shared_ptr<rcl_subscription_t>
  get_intra_process_subscription_handle() const override;

private:
  RCLCPP_DISABLE_COPY(GenericSubscription)

  std::shared_ptr<rmw_serialized_message_t> borrow_serialized_message(size_t capacity);
  rcutils_allocator_t default_allocator_;
  std::function<void(std::shared_ptr<rmw_serialized_message_t>)> callback_;
};

}  // namespace rosbag2_transport

#endif  // ROSBAG2_TRANSPORT__GENERIC_SUBSCRIPTION_HPP_
