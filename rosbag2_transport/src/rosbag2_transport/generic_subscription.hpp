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
#include "rclcpp/subscription.hpp"
#include "rclcpp/expand_topic_or_service_name.hpp"

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

  using CallbackMessageT = void*;
  using Alloc = std::allocator<void>;

  using MessageAllocTraits = rclcpp::allocator::AllocRebind<CallbackMessageT, Alloc>;
  using MessageAlloc = typename MessageAllocTraits::allocator_type;
  using MessageDeleter = rclcpp::allocator::Deleter<MessageAlloc, CallbackMessageT>;
  using MessageUniquePtr = std::unique_ptr<CallbackMessageT, MessageDeleter>;
  using GetMessageCallbackType = std::function<void (uint64_t, uint64_t, uint64_t, MessageUniquePtr &)>;
  using MatchesAnyPublishersCallbackType = std::function<bool (const rmw_gid_t *)>;

  /**
   * Constructor. In order to properly subscribe to a topic, this subscription needs to be added to
   * the node_topic_interface of the node passed into this constructor.
   *
   * \param node_handle Node handle to the node to create the subscription to
   * \param ts Type support handle
   * \param topic_name Topic name
   * \param callback Callback for new messages of serialized form
   */
  GenericSubscription(
    std::shared_ptr<rcl_node_t> node_handle,
    const rosidl_message_type_support_t & ts,
    const std::string & topic_name,
    std::function<void(std::shared_ptr<rmw_serialized_message_t>)> callback);

  // Same as create_serialized_message() as the subscription is to serialized_messages only
  std::shared_ptr<void> create_message() override;

  std::shared_ptr<rmw_serialized_message_t> create_serialized_message() override;

  void handle_message(
    std::shared_ptr<void> & message, const rmw_message_info_t & message_info) override;

  // Same as return_serialized_message() as the subscription is to serialized_messages only
  void return_message(std::shared_ptr<void> & message) override;

  void return_serialized_message(std::shared_ptr<rmw_serialized_message_t> & message) override;

  /// Returns the subscription options to duplicate the set options for the intra-process communication.
  const rcl_subscription_options_t *get_options() const
  {
    return rcl_subscription_get_options(subscription_handle_.get());
  }

  // Intra-process message handling is not supported by this publisher
  void handle_intra_process_message(
    rcl_interfaces::msg::IntraProcessMessage & ipm,
    const rmw_message_info_t & message_info) override;

  // Intra-process message handling is not supported by this publisher (returns nullptr always)
  const std::shared_ptr<rcl_subscription_t>
  get_intra_process_subscription_handle() const override;

  /// Implemenation detail.
  void setup_intra_process(
    uint64_t intra_process_subscription_id,
    GetMessageCallbackType get_message_callback,
    MatchesAnyPublishersCallbackType matches_any_publisher_callback,
    const rcl_subscription_options_t & intra_process_options)
  {
    std::string intra_process_topic_name = std::string(get_topic_name()) + "/_intra";
    std::cout<<"topic "<<intra_process_topic_name<<std::endl;
    rcl_ret_t ret = rcl_subscription_init(
      intra_process_subscription_handle_.get(),
      node_handle_.get(),
      rclcpp::type_support::get_intra_process_message_msg_type_support(),
      intra_process_topic_name.c_str(),
      &intra_process_options);
    if (ret != RCL_RET_OK) {
      if (ret == RCL_RET_TOPIC_NAME_INVALID) {
        auto rcl_node_handle = node_handle_.get();
        // this will throw on any validation problem
        rcl_reset_error();
        rclcpp::expand_topic_or_service_name(
          intra_process_topic_name,
          rcl_node_get_name(rcl_node_handle),
          rcl_node_get_namespace(rcl_node_handle));
      }

      rclcpp::exceptions::throw_from_rcl_error(ret, "could not create intra process subscription");
    }

    intra_process_subscription_id_ = intra_process_subscription_id;
    get_intra_process_message_callback_ = get_message_callback;
    matches_any_intra_process_publishers_ = matches_any_publisher_callback;
}

private:
  RCLCPP_DISABLE_COPY(GenericSubscription)

  std::shared_ptr<rmw_serialized_message_t> borrow_serialized_message(size_t capacity);
  rcutils_allocator_t default_allocator_;
  std::function<void(std::shared_ptr<rmw_serialized_message_t>)> callback_;

  // intra-process related
  GetMessageCallbackType get_intra_process_message_callback_;
  MatchesAnyPublishersCallbackType matches_any_intra_process_publishers_;
  uint64_t intra_process_subscription_id_;
};

}  // namespace rosbag2_transport

#endif  // ROSBAG2_TRANSPORT__GENERIC_SUBSCRIPTION_HPP_
