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

#include "generic_publisher.hpp"

#include <memory>
#include <string>

namespace
{
rcl_publisher_options_t rosbag2_get_publisher_options(const rclcpp::QoS & qos)
{
  auto options = rcl_publisher_get_default_options();
  options.qos = qos.get_rmw_qos_profile();
  return options;
}
}  // unnamed namespace

namespace rosbag2_transport
{

GenericPublisher::GenericPublisher(
  rclcpp::node_interfaces::NodeBaseInterface * node_base,
  const rosidl_message_type_support_t & type_support,
  const std::string & topic_name,
  const rclcpp::QoS & qos)
: rclcpp::PublisherBase(node_base, topic_name, type_support, rosbag2_get_publisher_options(qos)),
  type_support_{type_support}
{}

void GenericPublisher::publish(std::shared_ptr<rmw_serialized_message_t> message)
{
  auto return_code = rcl_publish_serialized_message(
    get_publisher_handle().get(), message.get(), NULL);

  if (return_code != RCL_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(return_code, "failed to publish serialized message");
  }
}

void GenericPublisher::publish_loaned_message(void * ros_message)
{
  auto return_code{rcl_publish_loaned_message(
      get_publisher_handle().get(), ros_message, NULL)};
  if (return_code != RCL_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(return_code, "failed to publish loaned message");
  }
}

void * GenericPublisher::borrow_loaned_message()
{
  void * loaned_message{nullptr};
  auto ret{rcl_borrow_loaned_message(
      this->get_publisher_handle().get(), &this->type_support_, &loaned_message)};
  if (ret != RMW_RET_OK) {
    throw std::runtime_error("failed to publish borrow loaned msg");
  }
  return loaned_message;
}

void GenericPublisher::deserialize_message(
  const rmw_serialized_message_t * serialized_message,
  void * deserialized_msg)
{
  auto ret{rmw_deserialize(serialized_message, &this->type_support_, deserialized_msg)};
  if (ret != RMW_RET_OK) {
    throw std::runtime_error("failed to deserialize msg");
  }
}

}  // namespace rosbag2_transport
