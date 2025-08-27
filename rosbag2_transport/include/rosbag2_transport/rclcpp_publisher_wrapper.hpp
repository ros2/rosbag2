// Copyright 2025 Apex.AI, Inc.
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

#ifndef ROSBAG2_TRANSPORT__RCLCPP_PUBLISHER_WRAPPER_HPP_
#define ROSBAG2_TRANSPORT__RCLCPP_PUBLISHER_WRAPPER_HPP_

#include <memory>
#include <utility>

#include "rclcpp/publisher.hpp"
#include "rclcpp/macros.hpp"

#include "rosbag2_transport/visibility_control.hpp"

namespace rosbag2_transport
{

/// \brief A wrapper class for rclcpp::Publisher with virtual methods for publishing and
/// borrowing loan messages.
/// \details This class is designed to be used when need to have ability to mock
/// the publisher in tests.
/// \note Due to the virtual methods and indirection this class is not as
/// efficient as using rclcpp::Publisher directly, so it should be used only when performance
/// overhead is considered and justified.
/// \tparam MessageT must be a ROS message type with its own type support
/// (e. g. std_msgs::msgs::String)
/// \tparam AllocatorT the allocator type used for loaned messages, defaults to
/// std::allocator<void>.
template<typename MessageT, typename AllocatorT = std::allocator<void>>
class ROSBAG2_TRANSPORT_PUBLIC RclcppPublisherWrapper {
public:
  explicit RclcppPublisherWrapper(typename rclcpp::Publisher<MessageT>::SharedPtr publisher)
  : publisher_(publisher) {}

  virtual ~RclcppPublisherWrapper()
  {
    // Explicitly reset the publisher_ to ensure the destructor of underlying publisher is called
    // only once.
    publisher_.reset();
  }

  RclcppPublisherWrapper & operator=(const typename rclcpp::Publisher<MessageT>::SharedPtr pub)
  {
    publisher_ = pub;
    return *this;
  }

  virtual void publish(const MessageT & msg)
  {
    publisher_->publish(msg);
  }

  virtual void publish(rclcpp::LoanedMessage<MessageT, AllocatorT> && loaned_msg)
  {
    publisher_->publish(std::move(loaned_msg));
  }

  virtual rclcpp::LoanedMessage<MessageT, AllocatorT> borrow_loaned_message()
  {
    return publisher_->borrow_loaned_message();
  }

  [[nodiscard]] virtual bool can_loan_messages() const
  {
    return publisher_->can_loan_messages();
  }

  [[nodiscard]] virtual const char * get_topic_name() const
  {
    return publisher_->get_topic_name();
  }

  RCLCPP_SMART_PTR_DEFINITIONS(RclcppPublisherWrapper<MessageT, AllocatorT>)

protected:
  typename rclcpp::Publisher<MessageT, AllocatorT>::SharedPtr publisher_;
};

}  // namespace rosbag2_transport

#endif  // ROSBAG2_TRANSPORT__RCLCPP_PUBLISHER_WRAPPER_HPP_
