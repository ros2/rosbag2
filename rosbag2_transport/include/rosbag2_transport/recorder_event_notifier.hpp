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
#include <string>

#include "rclcpp/node.hpp"

#include "rosbag2_cpp/bag_events.hpp"
#include "rclcpp/logging.hpp"

#include "rosbag2_interfaces/msg/write_split_event.hpp"
#include "rosbag2_transport/visibility_control.hpp"

namespace rosbag2_transport
{

class ROSBAG2_TRANSPORT_PUBLIC RecorderEventNotifier
{
public:
  /// \brief Constructor for the RecorderEventNotifier class.
  explicit RecorderEventNotifier(rclcpp::Node * node);

  /// \brief Destructor for the RecorderEventNotifier class.
  virtual ~RecorderEventNotifier();

  /// \brief Callback for when a bag split occurs in the recorder.
  void on_bag_split_in_recorder(const rosbag2_cpp::bag_events::BagSplitInfo & bag_split_info);

  /// \brief Callback for when messages are lost in recorder.
  void on_messages_lost_in_recorder(
    const std::vector<rosbag2_cpp::bag_events::MessagesLostInfo> & msgs_lost_info);

  /// \brief Callback for when messages are lost in transport.
  void on_messages_lost_in_transport(
    const std::string & topic_name,
    const rclcpp::QOSMessageLostInfo & qos_msgs_lost_info);

  /// \brief Get the number of messages lost on the transport layer.
  [[nodiscard]] size_t get_total_num_messages_lost_in_transport() const;

  /// \brief Get the number of messages lost in the recorder.
  [[nodiscard]] size_t get_total_num_messages_lost_in_recorder() const;

  /// \brief Reset the counters for messages lost in transport.
  void reset_total_num_messages_lost_in_transport();

  /// \brief Reset the counters for messages lost in recorder.
  void reset_total_num_messages_lost_in_recorder();

private:
  void event_publisher_thread_main();

  rclcpp::Node * node;

  // Variables for event publishing
  rclcpp::Publisher<rosbag2_interfaces::msg::WriteSplitEvent>::SharedPtr split_event_pub_;
  std::atomic<bool> event_publisher_thread_should_exit_ = false;
  std::atomic<bool> write_split_has_occurred_ = false;
  rosbag2_cpp::bag_events::BagSplitInfo bag_split_info_;
  std::mutex event_publisher_thread_mutex_;
  std::condition_variable event_publisher_thread_wake_cv_;
  std::thread event_publisher_thread_;
  std::chrono::milliseconds msgs_lost_statistics_update_period_{1000};  // 1 second

  std::mutex per_topic_messages_lost_statistics_mutex_;
  // Stores the number of messages lost per topic in the transport and recorder layers.
  std::unordered_map<std::string, std::pair<size_t, size_t>> per_topic_messages_lost_statistics_;

  std::atomic<size_t> total_num_messages_lost_in_transport_{0};
  std::atomic<size_t> total_num_messages_lost_in_recorder_{0};
};

}  // namespace rosbag2_transport

#endif  // ROSBAG2_TRANSPORT__RECORDER_EVENT_NOTIFIER_HPP_
