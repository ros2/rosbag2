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

#include "rosbag2_transport/recorder.hpp"

#include <algorithm>
#include <future>
#include <memory>
#include <regex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rclcpp/logging.hpp"
#include "rclcpp/clock.hpp"

#include "rosbag2_cpp/bag_events.hpp"
#include "rosbag2_cpp/writer.hpp"

#include "rosbag2_interfaces/srv/snapshot.hpp"

#include "rosbag2_storage/yaml.hpp"
#include "rosbag2_transport/qos.hpp"

#include "rosbag2_transport/topic_filter.hpp"

namespace rosbag2_transport
{

Recorder::Recorder(
  const std::string & node_name,
  const rclcpp::NodeOptions & node_options)
: rclcpp::Node(node_name, node_options)
{
  // TODO(karsten1987): Use this constructor later with parameter parsing.
  // The reader, storage_options as well as record_options can be loaded via parameter.
  // That way, the recorder can be used as a simple component in a component manager.
  throw rclcpp::exceptions::UnimplementedError();
}

Recorder::Recorder(
  std::shared_ptr<rosbag2_cpp::Writer> writer,
  const rosbag2_storage::StorageOptions & storage_options,
  const rosbag2_transport::RecordOptions & record_options,
  const std::string & node_name,
  const rclcpp::NodeOptions & node_options)
: Recorder(
    std::move(writer),
#ifndef _WIN32
    std::make_shared<KeyboardHandler>(false),
#else
    // We don't have signal handler option in constructor for windows version
    std::shared_ptr<KeyboardHandler>(new KeyboardHandler()),
#endif
    storage_options,
    record_options,
    node_name,
    node_options)
{}

Recorder::Recorder(
  std::shared_ptr<rosbag2_cpp::Writer> writer,
  std::shared_ptr<KeyboardHandler> keyboard_handler,
  const rosbag2_storage::StorageOptions & storage_options,
  const rosbag2_transport::RecordOptions & record_options,
  const std::string & node_name,
  const rclcpp::NodeOptions & node_options)
: rclcpp::Node(node_name, rclcpp::NodeOptions(node_options)
    .start_parameter_event_publisher(false)
    .parameter_overrides({rclcpp::Parameter("use_sim_time", record_options.use_sim_time)})),
  writer_(std::move(writer)),
  storage_options_(storage_options),
  record_options_(record_options),
  stop_discovery_(record_options_.is_discovery_disabled),
  paused_(record_options.start_paused),
  keyboard_handler_(std::move(keyboard_handler))
{
  std::string key_str = enum_key_code_to_str(Recorder::kPauseResumeToggleKey);
  toggle_paused_key_callback_handle_ =
    keyboard_handler_->add_key_press_callback(
    [this](KeyboardHandler::KeyCode /*key_code*/,
    KeyboardHandler::KeyModifiers /*key_modifiers*/) {this->toggle_paused();},
    Recorder::kPauseResumeToggleKey);
  topic_filter_ = std::make_unique<TopicFilter>(record_options, this->get_node_graph_interface());
  // show instructions
  RCLCPP_INFO_STREAM(
    get_logger(),
    "Press " << key_str << " for pausing/resuming");

  for (auto & topic : record_options_.topics) {
    topic = rclcpp::expand_topic_or_service_name(topic, get_name(), get_namespace(), false);
  }
}

Recorder::~Recorder()
{
  keyboard_handler_->delete_key_press_callback(toggle_paused_key_callback_handle_);
  stop_discovery_ = true;
  if (discovery_future_.valid()) {
    discovery_future_.wait();
  }

  subscriptions_.clear();

  {
    std::lock_guard<std::mutex> lock(event_publisher_thread_mutex_);
    event_publisher_thread_should_exit_ = true;
  }
  event_publisher_thread_wake_cv_.notify_all();
  if (event_publisher_thread_.joinable()) {
    event_publisher_thread_.join();
  }
}

void Recorder::record()
{
  topic_qos_profile_overrides_ = record_options_.topic_qos_profile_overrides;
  if (record_options_.rmw_serialization_format.empty()) {
    throw std::runtime_error("No serialization format specified!");
  }

  writer_->open(
    storage_options_,
    {rmw_get_serialization_format(), record_options_.rmw_serialization_format});

  // Only expose snapshot service when mode is enabled
  if (storage_options_.snapshot_mode) {
    srv_snapshot_ = create_service<rosbag2_interfaces::srv::Snapshot>(
      "~/snapshot",
      [this](
        const std::shared_ptr<rmw_request_id_t>/* request_header */,
        const std::shared_ptr<rosbag2_interfaces::srv::Snapshot::Request>/* request */,
        const std::shared_ptr<rosbag2_interfaces::srv::Snapshot::Response> response)
      {
        response->success = writer_->take_snapshot();
      });
  }

  // Start the thread that will publish events
  event_publisher_thread_ = std::thread(&Recorder::event_publisher_thread_main, this);

  split_event_pub_ = create_publisher<rosbag2_interfaces::msg::WriteSplitEvent>(
    "events/write_split",
    1);
  rosbag2_cpp::bag_events::WriterEventCallbacks callbacks;
  callbacks.write_split_callback =
    [this](rosbag2_cpp::bag_events::BagSplitInfo & info) {
      {
        std::lock_guard<std::mutex> lock(event_publisher_thread_mutex_);
        bag_split_info_ = info;
        write_split_has_occurred_ = true;
      }
      event_publisher_thread_wake_cv_.notify_all();
    };
  writer_->add_event_callbacks(callbacks);

  serialization_format_ = record_options_.rmw_serialization_format;
  RCLCPP_INFO(this->get_logger(), "Listening for topics...");
  subscribe_topics(get_requested_or_available_topics());

  if (!record_options_.is_discovery_disabled) {
    discovery_future_ =
      std::async(std::launch::async, std::bind(&Recorder::topics_discovery, this));
  }
}

void Recorder::event_publisher_thread_main()
{
  RCLCPP_INFO(get_logger(), "Event publisher thread: Starting");

  bool should_exit = false;

  while (!should_exit) {
    std::unique_lock<std::mutex> lock(event_publisher_thread_mutex_);
    event_publisher_thread_wake_cv_.wait(
      lock,
      [this] {return event_publisher_thread_should_wake();});

    if (write_split_has_occurred_) {
      write_split_has_occurred_ = false;

      auto message = rosbag2_interfaces::msg::WriteSplitEvent();
      message.closed_file = bag_split_info_.closed_file;
      message.opened_file = bag_split_info_.opened_file;
      split_event_pub_->publish(message);
    }

    should_exit = event_publisher_thread_should_exit_;
  }

  RCLCPP_INFO(get_logger(), "Event publisher thread: Exiting");
}

bool Recorder::event_publisher_thread_should_wake()
{
  return write_split_has_occurred_ || event_publisher_thread_should_exit_;
}

const rosbag2_cpp::Writer & Recorder::get_writer_handle()
{
  return *writer_;
}

void Recorder::pause()
{
  paused_.store(true);
  RCLCPP_INFO_STREAM(get_logger(), "Pausing recording.");
}

void Recorder::resume()
{
  paused_.store(false);
  RCLCPP_INFO_STREAM(get_logger(), "Resuming recording.");
}

void Recorder::toggle_paused()
{
  if (paused_.load()) {
    this->resume();
  } else {
    this->pause();
  }
}

bool Recorder::is_paused()
{
  return paused_.load();
}

void Recorder::topics_discovery()
{
  while (rclcpp::ok() && stop_discovery_ == false) {
    auto topics_to_subscribe =
      get_requested_or_available_topics();
    for (const auto & topic_and_type : topics_to_subscribe) {
      warn_if_new_qos_for_subscribed_topic(topic_and_type.first);
    }
    auto missing_topics = get_missing_topics(topics_to_subscribe);
    subscribe_topics(missing_topics);

    if (!record_options_.topics.empty() && subscriptions_.size() == record_options_.topics.size()) {
      RCLCPP_INFO(
        this->get_logger(),
        "All requested topics are subscribed. Stopping discovery...");
      return;
    }
    std::this_thread::sleep_for(record_options_.topic_polling_interval);
  }
}

std::unordered_map<std::string, std::string>
Recorder::get_requested_or_available_topics()
{
  auto all_topics_and_types = this->get_topic_names_and_types();
  return topic_filter_->filter_topics(all_topics_and_types);
}

std::unordered_map<std::string, std::string>
Recorder::get_missing_topics(const std::unordered_map<std::string, std::string> & all_topics)
{
  std::unordered_map<std::string, std::string> missing_topics;
  for (const auto & i : all_topics) {
    if (subscriptions_.find(i.first) == subscriptions_.end()) {
      missing_topics.emplace(i.first, i.second);
    }
  }
  return missing_topics;
}


void Recorder::subscribe_topics(
  const std::unordered_map<std::string, std::string> & topics_and_types)
{
  for (const auto & topic_with_type : topics_and_types) {
    subscribe_topic(
      {
        topic_with_type.first,
        topic_with_type.second,
        serialization_format_,
        serialized_offered_qos_profiles_for_topic(topic_with_type.first)
      });
  }
}

void Recorder::subscribe_topic(const rosbag2_storage::TopicMetadata & topic)
{
  // Need to create topic in writer before we are trying to create subscription. Since in
  // callback for subscription we are calling writer_->write(bag_message); and it could happened
  // that callback called before we reached out the line: writer_->create_topic(topic)
  writer_->create_topic(topic);

  Rosbag2QoS subscription_qos{subscription_qos_for_topic(topic.name)};
  auto subscription = create_subscription(topic.name, topic.type, subscription_qos);
  if (subscription) {
    subscriptions_.insert({topic.name, subscription});
    RCLCPP_INFO_STREAM(
      this->get_logger(),
      "Subscribed to topic '" << topic.name << "'");
  } else {
    writer_->remove_topic(topic);
    subscriptions_.erase(topic.name);
  }
}

std::shared_ptr<rclcpp::GenericSubscription>
Recorder::create_subscription(
  const std::string & topic_name, const std::string & topic_type, const rclcpp::QoS & qos)
{
  auto subscription = this->create_generic_subscription(
    topic_name,
    topic_type,
    qos,
    [this, topic_name, topic_type](std::shared_ptr<const rclcpp::SerializedMessage> message) {
      if (!paused_.load()) {
        writer_->write(message, topic_name, topic_type, this->get_clock()->now());
      }
    });
  return subscription;
}

std::string Recorder::serialized_offered_qos_profiles_for_topic(const std::string & topic_name)
{
  YAML::Node offered_qos_profiles;
  auto endpoints = this->get_publishers_info_by_topic(topic_name);
  for (const auto & info : endpoints) {
    offered_qos_profiles.push_back(Rosbag2QoS(info.qos_profile()));
  }
  return YAML::Dump(offered_qos_profiles);
}

rclcpp::QoS Recorder::subscription_qos_for_topic(const std::string & topic_name) const
{
  if (topic_qos_profile_overrides_.count(topic_name)) {
    RCLCPP_INFO_STREAM(
      this->get_logger(),
      "Overriding subscription profile for " << topic_name);
    return topic_qos_profile_overrides_.at(topic_name);
  }
  return Rosbag2QoS::adapt_request_to_offers(
    topic_name, this->get_publishers_info_by_topic(topic_name));
}

void Recorder::warn_if_new_qos_for_subscribed_topic(const std::string & topic_name)
{
  auto existing_subscription = subscriptions_.find(topic_name);
  if (existing_subscription == subscriptions_.end()) {
    // Not subscribed yet
    return;
  }
  if (topics_warned_about_incompatibility_.count(topic_name) > 0) {
    // Already warned about this topic
    return;
  }
  const auto actual_qos = existing_subscription->second->get_actual_qos();
  const auto & used_profile = actual_qos.get_rmw_qos_profile();
  auto publishers_info = this->get_publishers_info_by_topic(topic_name);
  for (const auto & info : publishers_info) {
    auto new_profile = info.qos_profile().get_rmw_qos_profile();
    bool incompatible_reliability =
      new_profile.reliability == RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT &&
      used_profile.reliability != RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    bool incompatible_durability =
      new_profile.durability == RMW_QOS_POLICY_DURABILITY_VOLATILE &&
      used_profile.durability != RMW_QOS_POLICY_DURABILITY_VOLATILE;

    if (incompatible_reliability) {
      RCLCPP_WARN_STREAM(
        this->get_logger(),
        "A new publisher for subscribed topic " << topic_name << " "
          "was found offering RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT, "
          "but rosbag already subscribed requesting RMW_QOS_POLICY_RELIABILITY_RELIABLE. "
          "Messages from this new publisher will not be recorded.");
      topics_warned_about_incompatibility_.insert(topic_name);
    } else if (incompatible_durability) {
      RCLCPP_WARN_STREAM(
        this->get_logger(),
        "A new publisher for subscribed topic " << topic_name << " "
          "was found offering RMW_QOS_POLICY_DURABILITY_VOLATILE, "
          "but rosbag2 already subscribed requesting RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL. "
          "Messages from this new publisher will not be recorded.");
      topics_warned_about_incompatibility_.insert(topic_name);
    }
  }
}

}  // namespace rosbag2_transport
