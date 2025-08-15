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

#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"

#include "rosbag2_transport/recorder_event_notifier.hpp"

namespace rosbag2_transport
{

RecorderEventNotifier::RecorderEventNotifier(rclcpp::Node * node)
: node(node)
{
  if (!node) {
    throw std::invalid_argument("Node pointer cannot be null");
  }
  split_event_pub_ =
    node->create_publisher<rosbag2_interfaces::msg::WriteSplitEvent>("events/write_split", 1);

  // Start the thread that will publish events
  {
    std::lock_guard<std::mutex> lock(event_publisher_thread_mutex_);
    event_publisher_thread_should_exit_ = false;
    event_publisher_thread_ = std::thread(&RecorderEventNotifier::event_publisher_thread_main,
        this);
  }
}

RecorderEventNotifier::~RecorderEventNotifier()
{
  if (event_publisher_thread_.joinable()) {
    {
      std::lock_guard<std::mutex> lock(event_publisher_thread_mutex_);
      event_publisher_thread_should_exit_ = true;
    }
    event_publisher_thread_wake_cv_.notify_all();
    event_publisher_thread_.join();
  }
}

void RecorderEventNotifier::event_publisher_thread_main()
{
  RCLCPP_INFO(node->get_logger(), "Event publisher thread: Started");
  while (!event_publisher_thread_should_exit_.load()) {
    std::unique_lock<std::mutex> pub_thread_lock(event_publisher_thread_mutex_);

    event_publisher_thread_wake_cv_.wait_for(pub_thread_lock, msgs_lost_statistics_update_period_,
      [this]() {
        return write_split_has_occurred_ || event_publisher_thread_should_exit_;
        }
    );

    if (write_split_has_occurred_) {
      write_split_has_occurred_ = false;
      auto message = rosbag2_interfaces::msg::WriteSplitEvent();
      message.closed_file = bag_split_info_.closed_file;
      message.opened_file = bag_split_info_.opened_file;
      message.node_name = node->get_fully_qualified_name();
      try {
        split_event_pub_->publish(message);
      } catch (const std::exception & e) {
        RCLCPP_ERROR_STREAM(
          node->get_logger(),
          "Failed to publish message on '/events/write_split' topic. \nError: " << e.what());
      } catch (...) {
        RCLCPP_ERROR_STREAM(
          node->get_logger(),
          "Failed to publish message on '/events/write_split' topic.");
      }
    }

//    {
//      // TODO(morlov): Check if we need to publish statistics about messages lost events
//      std::unique_lock<std::mutex> statistics_lock(per_topic_messages_lost_statistics_mutex_);
//      for (const auto &[topic, lost_stats] : per_topic_messages_lost_statistics_) {
//        const auto &[transport_lost, recorder_lost] = lost_stats;
//        // Use topic, transport_lost, and recorder_lost to publish statistics if needed
//      }
//    }
  }
  RCLCPP_INFO(node->get_logger(), "Event publisher thread: Exited");
}

void RecorderEventNotifier::on_bag_split_in_recorder(
  const rosbag2_cpp::bag_events::BagSplitInfo & bag_split_info)
{
  {
    std::lock_guard<std::mutex> lock(event_publisher_thread_mutex_);
    bag_split_info_ = bag_split_info;
    write_split_has_occurred_ = true;
  }
  event_publisher_thread_wake_cv_.notify_all();
}

void RecorderEventNotifier::on_messages_lost_in_recorder(
  const std::vector<rosbag2_cpp::bag_events::MessagesLostInfo> & msgs_lost_info)
{
  if (!msgs_lost_info.empty()) {
    // Log lost messages in recorder
    std::string log_text("Recorder lost messages per topic: ");
    {
      std::unique_lock<std::mutex> lock(per_topic_messages_lost_statistics_mutex_);
      for (const auto & info : msgs_lost_info) {
        total_num_messages_lost_in_recorder_.fetch_add(info.num_messages_lost);
        per_topic_messages_lost_statistics_[info.topic_name].second += info.num_messages_lost;
        log_text += "\n\t" + info.topic_name + ": " + std::to_string(info.num_messages_lost);
      }
    }
    RCLCPP_DEBUG(node->get_logger(), "%s", log_text.c_str());
  }
}

void RecorderEventNotifier::on_messages_lost_in_transport(
  const std::string & topic_name,
  const rclcpp::QOSMessageLostInfo & qos_msgs_lost_info)
{
  total_num_messages_lost_in_transport_.fetch_add(qos_msgs_lost_info.total_count_change);
  RCLCPP_DEBUG(
    node->get_logger(),
    "Messages lost on transport layer for topic '%s'. Total lost: %lu",
    topic_name.c_str(), qos_msgs_lost_info.total_count);

  {
    std::unique_lock<std::mutex> lock(per_topic_messages_lost_statistics_mutex_);
    per_topic_messages_lost_statistics_[topic_name].first += qos_msgs_lost_info.total_count_change;
  }
}

size_t RecorderEventNotifier::get_total_num_messages_lost_in_transport() const
{
  return total_num_messages_lost_in_transport_.load();
}

size_t RecorderEventNotifier::get_total_num_messages_lost_in_recorder() const
{
  return total_num_messages_lost_in_recorder_.load();
}

void RecorderEventNotifier::reset_total_num_messages_lost_in_transport()
{
  total_num_messages_lost_in_transport_.store(0);
}

void RecorderEventNotifier::reset_total_num_messages_lost_in_recorder()
{
  total_num_messages_lost_in_recorder_.store(0);
}


}  // namespace rosbag2_transport
