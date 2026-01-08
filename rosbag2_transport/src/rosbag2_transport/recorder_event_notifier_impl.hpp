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


#ifndef ROSBAG2_TRANSPORT__RECORDER_EVENT_NOTIFIER_IMPL_HPP_
#define ROSBAG2_TRANSPORT__RECORDER_EVENT_NOTIFIER_IMPL_HPP_

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/qos.hpp"

#include "rosbag2_interfaces/msg/messages_lost_event.hpp"
#include "rosbag2_interfaces/msg/write_split_event.hpp"
#include "rosbag2_cpp/bag_events.hpp"
#include "rosbag2_storage/qos.hpp"
#include "rosbag2_transport/rclcpp_publisher_wrapper.hpp"
#include "rosbag2_transport/recorder_event_notifier.hpp"

namespace rosbag2_transport
{
class RecorderEventNotifierImpl
{
public:
  using WriteSplitEvent = rosbag2_interfaces::msg::WriteSplitEvent;
  using MessagesLostEvent = rosbag2_interfaces::msg::MessagesLostEvent;
  static constexpr const char * kDefaultWriteSplitTopicName = "events/write_split";
  static constexpr const char * kDefaultMessagesLostTopicName = "events/rosbag2_messages_lost";

  explicit RecorderEventNotifierImpl(
    rclcpp::Node * node,
    const rosbag2_transport::RecordOptions & record_options,
    RclcppPublisherWrapper<WriteSplitEvent>::SharedPtr split_event_pub = nullptr,
    RclcppPublisherWrapper<MessagesLostEvent>::SharedPtr msgs_lost_event_pub = nullptr)
  : node_(node)
  {
    if (!node) {
      throw std::invalid_argument("Node pointer cannot be null");
    }

    rosbag2_storage::Rosbag2QoS split_event_qos = rosbag2_storage::Rosbag2QoS::EventQoS();
    rosbag2_storage::Rosbag2QoS msgs_lost_event_qos = rosbag2_storage::Rosbag2QoS::EventQoS();

    // Need to expand the default relative topic name to check for QoS overrides
    auto write_split_topic_name = rclcpp::expand_topic_or_service_name(
      kDefaultWriteSplitTopicName, node->get_name(), node->get_namespace(), false);

    if (record_options.topic_qos_profile_overrides.find(write_split_topic_name) !=
      record_options.topic_qos_profile_overrides.end())
    {
      const auto & override_qos =
        record_options.topic_qos_profile_overrides.at(write_split_topic_name);
      split_event_qos = rosbag2_storage::Rosbag2QoS(override_qos);
      RCLCPP_DEBUG(node_->get_logger(),
                   "Using overridden QoS profile: \n%s\nfor '%s' topic.",
                   split_event_qos.to_string().c_str(),
                   write_split_topic_name.c_str());
    }

    // Need to expand the default relative topic name to check for QoS overrides
    auto messages_lost_topic_name = rclcpp::expand_topic_or_service_name(
      kDefaultMessagesLostTopicName, node->get_name(), node->get_namespace(), false);

    if (record_options.topic_qos_profile_overrides.find(messages_lost_topic_name) !=
      record_options.topic_qos_profile_overrides.end())
    {
      const auto & override_qos =
        record_options.topic_qos_profile_overrides.at(messages_lost_topic_name);
      msgs_lost_event_qos = rosbag2_storage::Rosbag2QoS(override_qos);
      RCLCPP_DEBUG(node_->get_logger(),
                   "Using overridden QoS profile: \n%s\nfor '%s' topic.",
                   msgs_lost_event_qos.to_string().c_str(),
                   messages_lost_topic_name.c_str());
    }

    // Store QoS profiles for getter methods
    split_event_qos_ = split_event_qos;
    msgs_lost_event_qos_ = msgs_lost_event_qos;

    if (split_event_pub) {
      split_event_pub_ = std::move(split_event_pub);
    } else {
      split_event_pub_ = RclcppPublisherWrapper<WriteSplitEvent>::make_shared(
        node_->create_publisher<WriteSplitEvent>(kDefaultWriteSplitTopicName,
                                                 split_event_qos));
    }

    if (msgs_lost_event_pub) {
      msgs_lost_event_pub_ = std::move(msgs_lost_event_pub);
    } else {
      msgs_lost_event_pub_ = RclcppPublisherWrapper<MessagesLostEvent>::make_shared(
        node_->create_publisher<MessagesLostEvent>(kDefaultMessagesLostTopicName,
                                                   msgs_lost_event_qos));
    }

    // Start the thread that will publish events
    {
      std::lock_guard<std::mutex> lock(event_publisher_thread_mutex_);
      event_publisher_thread_should_exit_ = false;
      event_publisher_thread_ = std::thread(&RecorderEventNotifierImpl::event_publisher_thread_main,
                                            this);
    }
  }

  virtual ~RecorderEventNotifierImpl()
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

  [[nodiscard]] std::string_view get_write_split_topic_name() const
  {
    if (split_event_pub_) {
      return split_event_pub_->get_topic_name();
    } else {
      return std::string_view{""};
    }
  }

  [[nodiscard]] std::string_view get_messages_lost_topic_name() const
  {
    if (msgs_lost_event_pub_) {
      return msgs_lost_event_pub_->get_topic_name();
    } else {
      return std::string_view{""};
    }
  }

  [[nodiscard]] rclcpp::QoS get_write_split_qos() const
  {
    return split_event_qos_;
  }

  [[nodiscard]] rclcpp::QoS get_messages_lost_qos() const
  {
    return msgs_lost_event_qos_;
  }

  /// \brief Set the maximum publishing rate for messages lost statistics.
  void set_messages_lost_statistics_max_publishing_rate(float update_rate_hz)
  {
    {
      std::unique_lock<std::mutex> pub_thread_lock(event_publisher_thread_mutex_);
      if (update_rate_hz == 0.0f) {
        disable_publishing_msgs_lost_statistics_ = true;
        RCLCPP_DEBUG(node_->get_logger(), "Messages lost statistics publishing is disabled");
      } else if (update_rate_hz > 0.0f) {
        if (update_rate_hz >= 1000.0f) {
          throw std::invalid_argument("Update rate must be less than 1000 Hz");
        }
        disable_publishing_msgs_lost_statistics_ = false;
        msgs_lost_stats_max_publishing_period_ =
          std::chrono::milliseconds(static_cast<int>(1000 / update_rate_hz));
        RCLCPP_DEBUG(node_->get_logger(),
                     "Messages lost statistics publishing update rate set to %ld ms",
                     msgs_lost_stats_max_publishing_period_.count());
      } else {
        throw std::invalid_argument("Update rate must be non-negative");
      }
    }
    event_publisher_thread_wake_cv_.notify_all();
  }

  void on_bag_split_in_recorder(const rosbag2_cpp::bag_events::BagSplitInfo & bag_split_info)
  {
    {
      std::lock_guard<std::mutex> lock(event_publisher_thread_mutex_);
      bag_split_info_queue_.push(bag_split_info);
    }
    event_publisher_thread_wake_cv_.notify_all();
  }

  void on_messages_lost_in_recorder(
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
      RCLCPP_DEBUG(node_->get_logger(), "%s", log_text.c_str());
    }
  }

  void on_messages_lost_in_transport(
    const std::string & topic_name,
    const rclcpp::QOSMessageLostInfo & qos_msgs_lost_info)
  {
    total_num_messages_lost_in_transport_.fetch_add(qos_msgs_lost_info.total_count_change);
    RCLCPP_DEBUG(
      node_->get_logger(),
      "Messages lost on transport layer for topic '%s'. Total lost: %lu",
      topic_name.c_str(), qos_msgs_lost_info.total_count);

    {
      std::unique_lock<std::mutex> lock(per_topic_messages_lost_statistics_mutex_);
      per_topic_messages_lost_statistics_[topic_name].first +=
        qos_msgs_lost_info.total_count_change;
    }
  }

  [[nodiscard]] uint64_t get_total_num_messages_lost_in_transport() const
  {
    return total_num_messages_lost_in_transport_.load();
  }

  [[nodiscard]] uint64_t get_total_num_messages_lost_in_recorder() const
  {
    return total_num_messages_lost_in_recorder_.load();
  }

  void reset_total_num_messages_lost_in_transport()
  {
    total_num_messages_lost_in_transport_.store(0);
  }

  void reset_total_num_messages_lost_in_recorder()
  {
    total_num_messages_lost_in_recorder_.store(0);
  }

  void event_publisher_thread_main()
  {
    RCLCPP_INFO(node_->get_logger(), "Event publisher thread: Started");
    while (!event_publisher_thread_should_exit_.load()) {
      std::unique_lock<std::mutex> pub_thread_lock(event_publisher_thread_mutex_);
      if (disable_publishing_msgs_lost_statistics_) {
        // If publishing of messages lost statistics is disabled, wait indefinitely
        event_publisher_thread_wake_cv_.wait(pub_thread_lock,
          [this]() {
            return !bag_split_info_queue_.empty() || event_publisher_thread_should_exit_ ||
                   !disable_publishing_msgs_lost_statistics_;
          });
      } else {
        // Wait for either a write split event or the specified period for messages lost statistics
        event_publisher_thread_wake_cv_.wait_for(
          pub_thread_lock,
          msgs_lost_stats_max_publishing_period_,
          [this]() {
            return !bag_split_info_queue_.empty() || event_publisher_thread_should_exit_ ||
                   disable_publishing_msgs_lost_statistics_;
          }
        );
      }

      while (!bag_split_info_queue_.empty()) {
        try {
          const auto & bag_split_info = bag_split_info_queue_.front();
          auto message = rosbag2_interfaces::msg::WriteSplitEvent();
          message.closed_file = bag_split_info.closed_file;
          message.opened_file = bag_split_info.opened_file;
          message.node_name = node_->get_fully_qualified_name();
          split_event_pub_->publish(message);
        } catch (const std::exception & e) {
          RCLCPP_ERROR_STREAM(node_->get_logger(),
            "Failed to publish message on '" << get_write_split_topic_name() <<
            "' topic. \nError: " << e.what());
        } catch (...) {
          RCLCPP_ERROR_STREAM(node_->get_logger(),
            "Failed to publish message on '" << get_write_split_topic_name() << "' topic.");
        }
        bag_split_info_queue_.pop();
      }

      if (!disable_publishing_msgs_lost_statistics_) {
        std::unique_lock<std::mutex> statistics_lock(per_topic_messages_lost_statistics_mutex_);
        if (!per_topic_messages_lost_statistics_.empty()) {
          try {
            auto message = rosbag2_interfaces::msg::MessagesLostEvent();
            message.node_name = node_->get_fully_qualified_name();
            for (const auto &[topic, lost_stats] : per_topic_messages_lost_statistics_) {
              const auto &[transport_lost, recorder_lost] = lost_stats;
              message.messages_lost_statistics.emplace_back();
              message.messages_lost_statistics.back().topic_name = topic;
              message.messages_lost_statistics.back().messages_lost_in_transport = transport_lost;
              message.messages_lost_statistics.back().messages_lost_in_recorder = recorder_lost;
            }
            // Reset statistics
            per_topic_messages_lost_statistics_.clear();
            statistics_lock.unlock();
            msgs_lost_event_pub_->publish(message);
          } catch (const std::exception & e) {
            RCLCPP_ERROR_STREAM(node_->get_logger(),
              "Failed to publish message on '" << get_messages_lost_topic_name() <<
              "' topic. \nError: " << e.what());
          } catch (...) {
            RCLCPP_ERROR_STREAM(node_->get_logger(),
              "Failed to publish message on '" << get_messages_lost_topic_name() << "' topic.");
          }
        }
      }
    }
    RCLCPP_INFO(node_->get_logger(), "Event publisher thread: Exited");
  }

private:
  rclcpp::Node * node_;
  RclcppPublisherWrapper<WriteSplitEvent>::SharedPtr split_event_pub_;
  RclcppPublisherWrapper<MessagesLostEvent>::SharedPtr msgs_lost_event_pub_;
  rclcpp::QoS split_event_qos_{1};
  rclcpp::QoS msgs_lost_event_qos_{1};
  std::atomic<bool> event_publisher_thread_should_exit_ = false;
  std::queue<rosbag2_cpp::bag_events::BagSplitInfo> bag_split_info_queue_;
  std::mutex event_publisher_thread_mutex_;
  std::condition_variable event_publisher_thread_wake_cv_;
  std::thread event_publisher_thread_;
  bool disable_publishing_msgs_lost_statistics_{true};
  std::chrono::milliseconds msgs_lost_stats_max_publishing_period_{1000};  // 1 second

  std::mutex per_topic_messages_lost_statistics_mutex_;
  // Stores the number of messages lost per topic in the transport and recorder layers.
  std::unordered_map<std::string, std::pair<uint64_t, uint64_t>>
  per_topic_messages_lost_statistics_;

  std::atomic<uint64_t> total_num_messages_lost_in_transport_{0};
  std::atomic<uint64_t> total_num_messages_lost_in_recorder_{0};
};

}  // namespace rosbag2_transport

#endif  // ROSBAG2_TRANSPORT__RECORDER_EVENT_NOTIFIER_IMPL_HPP_
