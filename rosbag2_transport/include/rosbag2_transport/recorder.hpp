// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#ifndef ROSBAG2_TRANSPORT__RECORDER_HPP_
#define ROSBAG2_TRANSPORT__RECORDER_HPP_

#include <future>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "keyboard_handler/keyboard_handler.hpp"

#include "example_interfaces/msg/empty.hpp"

#include "rclcpp/node.hpp"
#include "rclcpp/qos.hpp"

#include "rosbag2_cpp/writer.hpp"

#include "rosbag2_interfaces/srv/snapshot.hpp"

#include "rosbag2_interfaces/msg/write_split_event.hpp"

#include "rosbag2_storage/topic_metadata.hpp"

#include "rosbag2_transport/record_options.hpp"
#include "rosbag2_transport/visibility_control.hpp"
#include "rosbag2_transport/topic_filter.hpp"

namespace rosbag2_cpp
{
class Writer;
}

namespace rosbag2_transport
{

class Recorder : public rclcpp::Node
{
public:
  ROSBAG2_TRANSPORT_PUBLIC
  explicit Recorder(
    const std::string & node_name = "rosbag2_recorder",
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());

  ROSBAG2_TRANSPORT_PUBLIC
  Recorder(
    std::shared_ptr<rosbag2_cpp::Writer> writer,
    const rosbag2_storage::StorageOptions & storage_options,
    const rosbag2_transport::RecordOptions & record_options,
    const std::string & node_name = "rosbag2_recorder",
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());

  ROSBAG2_TRANSPORT_PUBLIC
  Recorder(
    std::shared_ptr<rosbag2_cpp::Writer> writer,
    std::shared_ptr<KeyboardHandler> keyboard_handler,
    const rosbag2_storage::StorageOptions & storage_options,
    const rosbag2_transport::RecordOptions & record_options,
    const std::string & node_name = "rosbag2_recorder",
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());

  ROSBAG2_TRANSPORT_PUBLIC
  virtual ~Recorder();

  ROSBAG2_TRANSPORT_PUBLIC
  void record();

  const std::unordered_set<std::string> &
  topics_using_fallback_qos() const
  {
    return topics_warned_about_incompatibility_;
  }

  const std::unordered_map<std::string, std::shared_ptr<rclcpp::GenericSubscription>> &
  subscriptions() const
  {
    return subscriptions_;
  }

  ROSBAG2_TRANSPORT_PUBLIC
  const rosbag2_cpp::Writer & get_writer_handle();

  /// Pause the recording.
  ROSBAG2_TRANSPORT_PUBLIC
  void pause();

  /// Resume recording.
  ROSBAG2_TRANSPORT_PUBLIC
  void resume();

  /// Pause if it was recording, continue recording if paused.
  ROSBAG2_TRANSPORT_PUBLIC
  void toggle_paused();

  /// Return the current paused state.
  ROSBAG2_TRANSPORT_PUBLIC
  bool is_paused();

  inline constexpr static const auto kPauseResumeToggleKey = KeyboardHandler::KeyCode::SPACE;

protected:
  ROSBAG2_TRANSPORT_EXPORT
  std::unordered_map<std::string, std::string> get_requested_or_available_topics();

private:
  void topics_discovery();

  std::unordered_map<std::string, std::string>
  get_missing_topics(const std::unordered_map<std::string, std::string> & all_topics);

  void subscribe_topics(
    const std::unordered_map<std::string, std::string> & topics_and_types);

  void subscribe_topic(const rosbag2_storage::TopicMetadata & topic);

  std::shared_ptr<rclcpp::GenericSubscription> create_subscription(
    const std::string & topic_name, const std::string & topic_type, const rclcpp::QoS & qos);

  /**
   * Find the QoS profile that should be used for subscribing.
   *
   * Uses the override from record_options, if it is specified for this topic.
   * Otherwise, falls back to Rosbag2QoS::adapt_request_to_offers
   *
   *   \param topic_name The full name of the topic, with namespace (ex. /arm/joint_status).
   *   \return The QoS profile to be used for subscribing.
   */
  rclcpp::QoS subscription_qos_for_topic(const std::string & topic_name) const;

  // Serialize all currently offered QoS profiles for a topic into a YAML list.
  std::string serialized_offered_qos_profiles_for_topic(const std::string & topic_name);

  void warn_if_new_qos_for_subscribed_topic(const std::string & topic_name);

  std::unique_ptr<TopicFilter> topic_filter_;
  std::shared_ptr<rosbag2_cpp::Writer> writer_;
  rosbag2_storage::StorageOptions storage_options_;
  rosbag2_transport::RecordOptions record_options_;
  std::atomic<bool> stop_discovery_;
  std::future<void> discovery_future_;
  std::unordered_map<std::string, std::shared_ptr<rclcpp::GenericSubscription>> subscriptions_;
  std::unordered_set<std::string> topics_warned_about_incompatibility_;
  std::string serialization_format_;
  std::unordered_map<std::string, rclcpp::QoS> topic_qos_profile_overrides_;
  std::unordered_set<std::string> topic_unknown_types_;
  rclcpp::Service<rosbag2_interfaces::srv::Snapshot>::SharedPtr srv_snapshot_;
  std::atomic<bool> paused_ = false;
  // Keyboard handler
  std::shared_ptr<KeyboardHandler> keyboard_handler_;
  // Toogle paused key callback handle
  KeyboardHandler::callback_handle_t toggle_paused_key_callback_handle_ =
    KeyboardHandler::invalid_handle;
  rclcpp::Publisher<rosbag2_interfaces::msg::WriteSplitEvent>::SharedPtr split_event_pub_;
};

}  // namespace rosbag2_transport

#endif  // ROSBAG2_TRANSPORT__RECORDER_HPP_
