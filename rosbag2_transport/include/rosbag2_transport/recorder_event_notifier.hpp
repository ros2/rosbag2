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

#ifndef ROSBAG2_TRANSPORT__RECORDER_EVENT_NOTIFIER_HPP_
#define ROSBAG2_TRANSPORT__RECORDER_EVENT_NOTIFIER_HPP_

#include <vector>
#include <memory>
#include <string>

#include "rclcpp/node.hpp"

#include "rosbag2_cpp/bag_events.hpp"
#include "rosbag2_transport/visibility_control.hpp"

#ifdef _WIN32
#  pragma warning(push)
// Suppress warning "rosbag2_transport::RecorderEventNotifier::pimpl_': class 'std::unique_ptr>'
// needs to have dll-interface to be used by clients of class
// 'rosbag2_transport::RecorderEventNotifier'"
// Justification:
// 1. We never inline code in the header that actually calls methods on RecorderEventNotifierImpl.
// 2. While the `RecorderEventNotifierImpl` is defined in the `recorder_event_notifier_impl.hpp`
// file, we include it only in the `recorder_event_notifier.cpp` file, and it does not leak into the
// external API.
// 3. The pimpl design pattern imply that implementation details are hidden and shouldn't be
// exposed with the dll-interface.
#  pragma warning(disable:4251)
#endif

namespace rosbag2_transport
{
class RecorderEventNotifierImpl;

class ROSBAG2_TRANSPORT_PUBLIC RecorderEventNotifier
{
public:
  /// \brief Constructor for the RecorderEventNotifier class.
  explicit RecorderEventNotifier(rclcpp::Node * node);

  /// \brief Destructor for the RecorderEventNotifier class.
  virtual ~RecorderEventNotifier();

  /// \brief Set the maximum update rate for messages lost statistics.
  /// \details This controls how often the statistics about messages lost are published.
  /// \param update_rate_hz Maximum publishing rate in times per second (Hz) for messages lost
  /// statistics. A value of 0.0 means that the statistics will not be published.
  /// \note Event notifier will not publish statistics if there are no messages lost since the last
  /// time it was published.
  /// \throws std::invalid_argument if the update rate is negative or if the update rate more than
  /// or equal 1000.00 Hz.
  void set_messages_lost_statistics_max_publishing_rate(float update_rate_hz);

  /// \brief Callback for when a bag split occurs in the recorder.
  void on_bag_split_in_recorder(const rosbag2_cpp::bag_events::BagSplitInfo & bag_split_info);

  /// \brief Callback for when messages are lost in transport.
  void on_messages_lost_in_transport(
    const std::string & topic_name,
    const rclcpp::QOSMessageLostInfo & qos_msgs_lost_info);

  /// \brief Getter for the total number of messages lost in transport.
  /// \return The total number of messages lost in transport.
  [[nodiscard]] uint64_t get_total_num_messages_lost_in_transport() const;

  /// \brief Reset the counters for messages lost in transport.
  void reset_total_num_messages_lost_in_transport();

private:
  std::unique_ptr<RecorderEventNotifierImpl> pimpl_;
};

}  // namespace rosbag2_transport

#ifdef _WIN32
#  pragma warning(pop)
#endif

#endif  // ROSBAG2_TRANSPORT__RECORDER_EVENT_NOTIFIER_HPP_
