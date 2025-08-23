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
#include <unordered_set>
#include <utility>
#include <vector>

#include "rcutils/allocator.h"

#include "rclcpp/logging.hpp"
#include "rclcpp/clock.hpp"

#include "rmw/types.h"

#include "rosbag2_cpp/bag_events.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_cpp/service_utils.hpp"

#include "rosbag2_interfaces/srv/snapshot.hpp"

#include "rosbag2_storage/yaml.hpp"
#include "rosbag2_storage/qos.hpp"

#include "logging.hpp"
#include "rosbag2_transport/config_options_from_node_params.hpp"
#include "rosbag2_transport/reader_writer_factory.hpp"
#include "rosbag2_transport/topic_filter.hpp"
#include "rosbag2_transport/recorder_event_notifier.hpp"

namespace rosbag2_transport
{

class RecorderImpl
{
public:
  RecorderImpl(
    rclcpp::Node * owner,
    std::shared_ptr<rosbag2_cpp::Writer> writer,
    std::shared_ptr<KeyboardHandler> keyboard_handler,
    const rosbag2_storage::StorageOptions & storage_options,
    const rosbag2_transport::RecordOptions & record_options);

  ~RecorderImpl();

  void record();

  /// @brief Stopping recording and closing writer.
  /// The record() can be called again after stop().
  void stop();

  const rosbag2_cpp::Writer & get_writer_handle();

  /// Pause the recording.
  void pause();

  /// Resume recording.
  void resume();

  /// Pause if it was recording, continue recording if paused.
  void toggle_paused();

  /// Return the current paused state.
  bool is_paused();

  /// Start discovery
  void start_discovery();

  /// Stop discovery
  void stop_discovery();

  std::unordered_map<std::string, std::string> get_requested_or_available_topics();

  /// Public members for access by wrapper
  std::unordered_set<std::string> topics_warned_about_incompatibility_;
  std::shared_ptr<rosbag2_cpp::Writer> writer_;
  rosbag2_storage::StorageOptions storage_options_;
  rosbag2_transport::RecordOptions record_options_;
  std::unordered_map<std::string, std::shared_ptr<rclcpp::SubscriptionBase>> subscriptions_;

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

  // Get all currently offered QoS profiles for a topic.
  std::vector<rclcpp::QoS> offered_qos_profiles_for_topic(
    const std::vector<rclcpp::TopicEndpointInfo> & topics_endpoint_info) const;

  void warn_if_new_qos_for_subscribed_topic(const std::string & topic_name);

  rclcpp::Node * node;
  std::unique_ptr<TopicFilter> topic_filter_;
  std::future<void> discovery_future_;
  std::string serialization_format_;
  std::unordered_map<std::string, rclcpp::QoS> topic_qos_profile_overrides_;
  std::unordered_set<std::string> topic_unknown_types_;
  rclcpp::Service<rosbag2_interfaces::srv::IsPaused>::SharedPtr srv_is_paused_;
  rclcpp::Service<rosbag2_interfaces::srv::Pause>::SharedPtr srv_pause_;
  rclcpp::Service<rosbag2_interfaces::srv::Resume>::SharedPtr srv_resume_;
  rclcpp::Service<rosbag2_interfaces::srv::Snapshot>::SharedPtr srv_snapshot_;
  rclcpp::Service<rosbag2_interfaces::srv::SplitBagfile>::SharedPtr srv_split_bagfile_;

  std::mutex start_stop_transition_mutex_;
  std::mutex discovery_mutex_;
  std::atomic<bool> discovery_running_ = false;
  std::atomic_uchar paused_ = 0;
  std::atomic<bool> in_recording_ = false;
  std::shared_ptr<KeyboardHandler> keyboard_handler_;
  KeyboardHandler::callback_handle_t toggle_paused_key_callback_handle_ =
    KeyboardHandler::invalid_handle;

  std::unique_ptr<RecorderEventNotifier> event_notifier_;
};

RecorderImpl::RecorderImpl(
  rclcpp::Node * owner,
  std::shared_ptr<rosbag2_cpp::Writer> writer,
  std::shared_ptr<KeyboardHandler> keyboard_handler,
  const rosbag2_storage::StorageOptions & storage_options,
  const rosbag2_transport::RecordOptions & record_options)
: writer_(std::move(writer)),
  storage_options_(storage_options),
  record_options_(record_options),
  node(owner),
  paused_(record_options.start_paused),
  keyboard_handler_(std::move(keyboard_handler)),
  event_notifier_(std::make_unique<RecorderEventNotifier>(node))
{
  event_notifier_->set_messages_lost_statistics_max_publishing_rate(0.0f);  // Disable by default

  if (record_options_.use_sim_time && record_options_.is_discovery_disabled) {
    throw std::runtime_error(
            "use_sim_time and is_discovery_disabled both set, but are incompatible settings. "
            "The /clock topic needs to be discovered to record with sim time.");
  }
  if (!record_options_.disable_keyboard_controls) {
    std::string key_str = enum_key_code_to_str(Recorder::kPauseResumeToggleKey);
    toggle_paused_key_callback_handle_ =
      keyboard_handler_->add_key_press_callback(
      [this](KeyboardHandler::KeyCode /*key_code*/,
      KeyboardHandler::KeyModifiers /*key_modifiers*/) {this->toggle_paused();},
      Recorder::kPauseResumeToggleKey);
    // show instructions
    RCLCPP_INFO_STREAM(
      node->get_logger(),
      "Press " << key_str << " for pausing/resuming");
  }

  for (auto & topic : record_options_.topics) {
    topic = rclcpp::expand_topic_or_service_name(
      topic, node->get_name(),
      node->get_namespace(), false);
  }

  for (auto & exclude_topic : record_options_.exclude_topics) {
    exclude_topic = rclcpp::expand_topic_or_service_name(
      exclude_topic, node->get_name(),
      node->get_namespace(), false);
  }

  for (auto & service : record_options_.services) {
    service = rclcpp::expand_topic_or_service_name(
      service, node->get_name(),
      node->get_namespace(), false);
  }

  for (auto & exclude_service_event_topic : record_options_.exclude_service_events) {
    exclude_service_event_topic = rclcpp::expand_topic_or_service_name(
      exclude_service_event_topic, node->get_name(),
      node->get_namespace(), false);
  }

  topic_filter_ = std::make_unique<TopicFilter>(record_options_, node->get_node_graph_interface());
}

RecorderImpl::~RecorderImpl()
{
  if (keyboard_handler_ &&
    (toggle_paused_key_callback_handle_ != KeyboardHandler::invalid_handle))
  {
    keyboard_handler_->delete_key_press_callback(toggle_paused_key_callback_handle_);
  }
  stop();
}

void RecorderImpl::stop()
{
  std::lock_guard<std::mutex> state_lock(start_stop_transition_mutex_);
  if (!in_recording_) {
    RCLCPP_DEBUG(node->get_logger(), "Recording has already been stopped or not running.");
    return;
  }

  stop_discovery();
  pause();
  subscriptions_.clear();
  writer_->close();  // Call writer->close() to finalize current bag file and write metadata

  in_recording_ = false;
  RCLCPP_INFO(node->get_logger(), "Recording stopped");

  auto num_messages_lost_on_transport = event_notifier_->get_total_num_messages_lost_in_transport();

  if (num_messages_lost_on_transport > 0) {
    RCLCPP_WARN(node->get_logger(),
                "Number of messages lost on the transport layer: %lu",
                num_messages_lost_on_transport);
  }
}

void RecorderImpl::record()
{
  std::lock_guard<std::mutex> state_lock(start_stop_transition_mutex_);
  if (in_recording_.exchange(true)) {
    RCLCPP_WARN_STREAM(
      node->get_logger(),
      "Called Recorder::record() while already in recording, dismissing request.");
    return;
  }
  paused_ = record_options_.start_paused;
  topic_qos_profile_overrides_ = record_options_.topic_qos_profile_overrides;
  if (record_options_.rmw_serialization_format.empty()) {
    throw std::runtime_error("No serialization format specified!");
  }

  subscriptions_.clear();
  event_notifier_->reset_total_num_messages_lost_in_transport();
  writer_->open(
    storage_options_,
    {rmw_get_serialization_format(), record_options_.rmw_serialization_format});

  // Only expose snapshot service when mode is enabled
  if (storage_options_.snapshot_mode) {
    srv_snapshot_ = node->create_service<rosbag2_interfaces::srv::Snapshot>(
      "~/snapshot",
      [this](
        const std::shared_ptr<rmw_request_id_t>/* request_header */,
        const std::shared_ptr<rosbag2_interfaces::srv::Snapshot::Request>/* request */,
        const std::shared_ptr<rosbag2_interfaces::srv::Snapshot::Response> response)
      {
        response->success = writer_->take_snapshot();
      });
  }

  srv_split_bagfile_ = node->create_service<rosbag2_interfaces::srv::SplitBagfile>(
    "~/split_bagfile",
    [this](
      const std::shared_ptr<rmw_request_id_t>/* request_header */,
      const std::shared_ptr<rosbag2_interfaces::srv::SplitBagfile::Request>/* request */,
      const std::shared_ptr<rosbag2_interfaces::srv::SplitBagfile::Response>/* response */)
    {
      writer_->split_bagfile();
    });

  srv_pause_ = node->create_service<rosbag2_interfaces::srv::Pause>(
    "~/pause",
    [this](
      const std::shared_ptr<rmw_request_id_t>/* request_header */,
      const std::shared_ptr<rosbag2_interfaces::srv::Pause::Request>/* request */,
      const std::shared_ptr<rosbag2_interfaces::srv::Pause::Response>/* response */)
    {
      pause();
    });

  srv_resume_ = node->create_service<rosbag2_interfaces::srv::Resume>(
    "~/resume",
    [this](
      const std::shared_ptr<rmw_request_id_t>/* request_header */,
      const std::shared_ptr<rosbag2_interfaces::srv::Resume::Request>/* request */,
      const std::shared_ptr<rosbag2_interfaces::srv::Resume::Response>/* response */)
    {
      resume();
    });

  srv_is_paused_ = node->create_service<rosbag2_interfaces::srv::IsPaused>(
    "~/is_paused",
    [this](
      const std::shared_ptr<rmw_request_id_t>/* request_header */,
      const std::shared_ptr<rosbag2_interfaces::srv::IsPaused::Request>/* request */,
      const std::shared_ptr<rosbag2_interfaces::srv::IsPaused::Response> response)
    {
      response->paused = is_paused();
    });

  rosbag2_cpp::bag_events::WriterEventCallbacks callbacks;
  callbacks.write_split_callback =
    [this](rosbag2_cpp::bag_events::BagSplitInfo & info) {
      event_notifier_->on_bag_split_in_recorder(info);
    };
  writer_->add_event_callbacks(callbacks);

  serialization_format_ = record_options_.rmw_serialization_format;
  RCLCPP_INFO(node->get_logger(), "Listening for topics...");
  if (!record_options_.use_sim_time) {
    subscribe_topics(get_requested_or_available_topics());
  }
  if (!record_options_.is_discovery_disabled) {
    start_discovery();
  }
  if (record_options_.start_paused) {
    RCLCPP_INFO(
      node->get_logger(), "Wait for recording: Press %s to start.",
      enum_key_code_to_str(Recorder::kPauseResumeToggleKey).c_str());
  } else {
    RCLCPP_INFO(node->get_logger(), "Recording...");
  }
}

const rosbag2_cpp::Writer & RecorderImpl::get_writer_handle()
{
  return *writer_;
}

void RecorderImpl::pause()
{
  if (paused_.exchange(true)) {
    RCLCPP_DEBUG(node->get_logger(), "Recorder is already in pause state.");
  } else {
    RCLCPP_INFO_STREAM(node->get_logger(), "Pausing recording.");
  }
}

void RecorderImpl::resume()
{
  if (paused_.exchange(false)) {
    RCLCPP_DEBUG(node->get_logger(), "Already in the recording.");
  } else {
    RCLCPP_INFO_STREAM(node->get_logger(), "Resuming recording.");
  }
}

void RecorderImpl::toggle_paused()
{
  if (atomic_fetch_xor(&paused_, 1)) {
    RCLCPP_INFO_STREAM(node->get_logger(), "Resuming recording.");
  } else {
    RCLCPP_INFO_STREAM(node->get_logger(), "Pausing recording.");
  }
}

bool RecorderImpl::is_paused()
{
  return paused_.load();
}

void RecorderImpl::start_discovery()
{
  std::lock_guard<std::mutex> state_lock(discovery_mutex_);
  if (discovery_running_.exchange(true)) {
    RCLCPP_DEBUG(node->get_logger(), "Recorder topic discovery is already running.");
  } else {
    discovery_future_ =
      std::async(std::launch::async, std::bind(&RecorderImpl::topics_discovery, this));
  }
}

void RecorderImpl::stop_discovery()
{
  std::lock_guard<std::mutex> state_lock(discovery_mutex_);
  if (discovery_running_.exchange(false)) {
    if (discovery_future_.valid()) {
      auto status = discovery_future_.wait_for(
        std::chrono::milliseconds(500) + record_options_.topic_polling_interval);
      if (status != std::future_status::ready) {
        RCLCPP_ERROR_STREAM(
          node->get_logger(),
          "discovery_future_.wait_for(" << record_options_.topic_polling_interval.count() <<
            ") return status: " <<
            (status == std::future_status::timeout ? "timeout" : "deferred"));
      } else {
        discovery_future_.get();
      }
    }
  } else {
    RCLCPP_DEBUG(
      node->get_logger(), "Recorder topic discovery has already been stopped or not running.");
  }
}

void RecorderImpl::topics_discovery()
{
  // If using sim time - wait until /clock topic received before even creating subscriptions
  if (record_options_.use_sim_time) {
    RCLCPP_INFO(
      node->get_logger(),
      "use_sim_time set, waiting for /clock before starting recording...");
    while (rclcpp::ok() && discovery_running_) {
      if (node->get_clock()->wait_until_started(record_options_.topic_polling_interval)) {
        break;
      }
    }
    if (node->get_clock()->started()) {
      RCLCPP_INFO(node->get_logger(), "Sim time /clock found, starting recording.");
    }
  }
  while (rclcpp::ok() && discovery_running_) {
    try {
      if (!record_options_.topics.empty() &&
        subscriptions_.size() == record_options_.topics.size())
      {
        RCLCPP_INFO(
          node->get_logger(), "All requested topics are subscribed. Stopping discovery...");
        return;
      }

      auto topics_to_subscribe = get_requested_or_available_topics();
      for (const auto & topic_and_type : topics_to_subscribe) {
        warn_if_new_qos_for_subscribed_topic(topic_and_type.first);
      }
      auto missing_topics = get_missing_topics(topics_to_subscribe);
      subscribe_topics(missing_topics);
    } catch (const std::exception & e) {
      RCLCPP_ERROR_STREAM(node->get_logger(), "Failure in topics discovery.\nError: " << e.what());
    } catch (...) {
      RCLCPP_ERROR_STREAM(node->get_logger(), "Failure in topics discovery.");
    }
    std::this_thread::sleep_for(record_options_.topic_polling_interval);
  }
}

std::unordered_map<std::string, std::string>
RecorderImpl::get_requested_or_available_topics()
{
  auto all_topics_and_types = node->get_topic_names_and_types();
  return topic_filter_->filter_topics(all_topics_and_types);
}

std::unordered_map<std::string, std::string>
RecorderImpl::get_missing_topics(const std::unordered_map<std::string, std::string> & all_topics)
{
  std::unordered_map<std::string, std::string> missing_topics;
  for (const auto & [topic_name, topic_type] : all_topics) {
    if (subscriptions_.find(topic_name) == subscriptions_.end()) {
      missing_topics.emplace(topic_name, topic_type);
    }
  }
  return missing_topics;
}


void RecorderImpl::subscribe_topics(
  const std::unordered_map<std::string, std::string> & topics_and_types)
{
  for (const auto & topic_with_type : topics_and_types) {
    auto endpoint_infos = node->get_publishers_info_by_topic(topic_with_type.first);
    subscribe_topic(
      {
        0u,
        topic_with_type.first,
        topic_with_type.second,
        serialization_format_,
        offered_qos_profiles_for_topic(endpoint_infos),
        type_description_hash_for_topic(endpoint_infos),
      });
  }
}

void RecorderImpl::subscribe_topic(const rosbag2_storage::TopicMetadata & topic)
{
  if (subscriptions_.find(topic.name) != subscriptions_.end()) {
    return;
  }
  // Need to create topic in writer before we are trying to create subscription. Since in
  // callback for subscription we are calling writer_->write(bag_message); and it could happened
  // that callback called before we reached out the line: writer_->create_topic(topic)
  writer_->create_topic(topic);

  rosbag2_storage::Rosbag2QoS subscription_qos{subscription_qos_for_topic(topic.name)};

  auto subscription = create_subscription(topic.name, topic.type, subscription_qos);
  if (subscription) {
    subscriptions_.insert({topic.name, subscription});
    if (node->get_logger().get_effective_level() == rclcpp::Logger::Level::Debug) {
      RCLCPP_DEBUG_STREAM(node->get_logger(),
        "Subscribed to topic '" << topic.name << "' with QoS:\n" << subscription_qos.to_string());
    } else {
      RCLCPP_INFO_STREAM(node->get_logger(), "Subscribed to topic '" << topic.name << "'");
    }

  } else {
    writer_->remove_topic(topic);
  }
}

std::shared_ptr<rclcpp::GenericSubscription>
RecorderImpl::create_subscription(
  const std::string & topic_name, const std::string & topic_type, const rclcpp::QoS & qos)
{
  rclcpp::SubscriptionOptions sub_options;
  sub_options.event_callbacks.message_lost_callback =
    [this, topic_name](const rclcpp::QOSMessageLostInfo & msgs_lost_info) {
      this->event_notifier_->on_messages_lost_in_transport(topic_name, msgs_lost_info);
    };

#ifdef _WIN32
  if (std::string(rmw_get_implementation_identifier()).find("rmw_connextdds") !=
    std::string::npos)
  {
    return node->create_generic_subscription(
      topic_name,
      topic_type,
      qos,
      [this, topic_name, topic_type](std::shared_ptr<const rclcpp::SerializedMessage> message,
      const rclcpp::MessageInfo &) {
        if (!paused_.load()) {
          writer_->write(
            std::move(message), topic_name, topic_type, node->now().nanoseconds(),
            0);
        }
      },
      sub_options);
  }
#endif

  if (record_options_.use_sim_time) {
    return node->create_generic_subscription(
      topic_name,
      topic_type,
      qos,
      [this, topic_name, topic_type](std::shared_ptr<const rclcpp::SerializedMessage> message,
      const rclcpp::MessageInfo & mi) {
        if (!paused_.load()) {
          writer_->write(
            std::move(message), topic_name, topic_type, node->now().nanoseconds(),
            mi.get_rmw_message_info().source_timestamp);
        }
      },
      sub_options);
  } else {
    return node->create_generic_subscription(
      topic_name,
      topic_type,
      qos,
      [this, topic_name, topic_type](std::shared_ptr<const rclcpp::SerializedMessage> message,
      const rclcpp::MessageInfo & mi) {
        if (!paused_.load()) {
          writer_->write(
            std::move(message), topic_name, topic_type,
            mi.get_rmw_message_info().received_timestamp,
            mi.get_rmw_message_info().source_timestamp);
        }
      },
      sub_options);
  }
}

std::vector<rclcpp::QoS> RecorderImpl::offered_qos_profiles_for_topic(
  const std::vector<rclcpp::TopicEndpointInfo> & topics_endpoint_info) const
{
  std::vector<rclcpp::QoS> offered_qos_profiles;
  for (const auto & info : topics_endpoint_info) {
    offered_qos_profiles.push_back(info.qos_profile());
  }
  return offered_qos_profiles;
}

std::string type_hash_to_string(const rosidl_type_hash_t & type_hash)
{
  if (type_hash.version == 0) {
    // version is unset, this is an empty type hash.
    return "";
  }
  if (type_hash.version > 1) {
    // this is a version we don't know how to serialize
    ROSBAG2_TRANSPORT_LOG_WARN_STREAM(
      "attempted to stringify type hash with unknown version " << type_hash.version);
    return "";
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  char * stringified_type_hash = nullptr;
  rcutils_ret_t status = rosidl_stringify_type_hash(&type_hash, allocator, &stringified_type_hash);
  std::string result = "";
  if (status == RCUTILS_RET_OK) {
    result = stringified_type_hash;
  }
  if (stringified_type_hash != nullptr) {
    allocator.deallocate(stringified_type_hash, allocator.state);
  }
  return result;
}

std::string type_description_hash_for_topic(
  const std::vector<rclcpp::TopicEndpointInfo> & topics_endpoint_info)
{
  rosidl_type_hash_t result_hash = rosidl_get_zero_initialized_type_hash();
  for (const auto & info : topics_endpoint_info) {
    // If all endpoint infos provide the same type hash, return it. Otherwise return an empty
    // string to signal that the type description hash for this topic cannot be determined.
    rosidl_type_hash_t endpoint_hash = info.topic_type_hash();
    if (endpoint_hash.version == 0) {
      continue;
    }
    if (result_hash.version == 0) {
      result_hash = endpoint_hash;
      continue;
    }
    bool difference_detected = (endpoint_hash.version != result_hash.version);
    difference_detected |= (
      0 != memcmp(endpoint_hash.value, result_hash.value, ROSIDL_TYPE_HASH_SIZE));
    if (difference_detected) {
      std::string result_string = type_hash_to_string(result_hash);
      std::string endpoint_string = type_hash_to_string(endpoint_hash);
      ROSBAG2_TRANSPORT_LOG_WARN_STREAM(
        "type description hashes for topic type '" << info.topic_type() << "' conflict: '" <<
          result_string << "' != '" << endpoint_string << "'");
      return "";
    }
  }
  return type_hash_to_string(result_hash);
}

rclcpp::QoS RecorderImpl::subscription_qos_for_topic(const std::string & topic_name) const
{
  if (topic_qos_profile_overrides_.count(topic_name)) {
    RCLCPP_INFO_STREAM(
      node->get_logger(),
      "Overriding subscription profile for " << topic_name);
    return topic_qos_profile_overrides_.at(topic_name);
  }
  return rosbag2_storage::Rosbag2QoS::adapt_request_to_offers(
    topic_name, node->get_publishers_info_by_topic(topic_name));
}

void RecorderImpl::warn_if_new_qos_for_subscribed_topic(const std::string & topic_name)
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
  auto publishers_info = node->get_publishers_info_by_topic(topic_name);
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
        node->get_logger(),
        "A new publisher for subscribed topic " << topic_name << " "
          "was found offering RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT, "
          "but rosbag already subscribed requesting RMW_QOS_POLICY_RELIABILITY_RELIABLE. "
          "Messages from this new publisher will not be recorded.");
      topics_warned_about_incompatibility_.insert(topic_name);
    } else if (incompatible_durability) {
      RCLCPP_WARN_STREAM(
        node->get_logger(),
        "A new publisher for subscribed topic " << topic_name << " "
          "was found offering RMW_QOS_POLICY_DURABILITY_VOLATILE, "
          "but rosbag2 already subscribed requesting RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL. "
          "Messages from this new publisher will not be recorded.");
      topics_warned_about_incompatibility_.insert(topic_name);
    }
  }
}

///////////////////////////////
// Recorder public interface

Recorder::Recorder(const rclcpp::NodeOptions & node_options)
: Recorder("rosbag2_recorder", node_options) {}

Recorder::Recorder(
  const std::string & node_name,
  const rclcpp::NodeOptions & node_options)
: rclcpp::Node(node_name, node_options)
{
  rosbag2_storage::StorageOptions storage_options = get_storage_options_from_node_params(*this);

  RecordOptions record_options = get_record_options_from_node_params(*this);

  std::shared_ptr<KeyboardHandler> keyboard_handler;
  if (!record_options.disable_keyboard_controls) {
    keyboard_handler = std::make_shared<KeyboardHandler>();
  }

  auto writer = ReaderWriterFactory::make_writer(record_options);

  pimpl_ = std::make_unique<RecorderImpl>(
    this, std::move(writer), keyboard_handler,
    storage_options, record_options);
  pimpl_->record();
}

Recorder::Recorder(
  std::shared_ptr<rosbag2_cpp::Writer> writer,
  const rosbag2_storage::StorageOptions & storage_options,
  const rosbag2_transport::RecordOptions & record_options,
  const std::string & node_name,
  const rclcpp::NodeOptions & node_options)
: Recorder(
    std::move(writer),
    record_options.disable_keyboard_controls ? nullptr : std::make_shared<KeyboardHandler>(),
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
    .append_parameter_override("use_sim_time", record_options.use_sim_time)),
  pimpl_(std::make_unique<RecorderImpl>(
      this, std::move(writer), std::move(keyboard_handler),
      storage_options, record_options))
{}

Recorder::~Recorder() = default;

void Recorder::record()
{
  pimpl_->record();
}

void Recorder::stop()
{
  pimpl_->stop();
}

const std::unordered_set<std::string> &
Recorder::topics_using_fallback_qos() const
{
  return pimpl_->topics_warned_about_incompatibility_;
}

const std::unordered_map<std::string, std::shared_ptr<rclcpp::SubscriptionBase>> &
Recorder::subscriptions() const
{
  return pimpl_->subscriptions_;
}

const rosbag2_cpp::Writer &
Recorder::get_writer_handle()
{
  return *pimpl_->writer_;
}

void
Recorder::pause()
{
  pimpl_->pause();
}

void
Recorder::resume()
{
  pimpl_->resume();
}

void
Recorder::toggle_paused()
{
  pimpl_->toggle_paused();
}

bool
Recorder::is_paused()
{
  return pimpl_->is_paused();
}

std::unordered_map<std::string, std::string>
Recorder::get_requested_or_available_topics()
{
  return pimpl_->get_requested_or_available_topics();
}

rosbag2_cpp::Writer &
Recorder::get_writer()
{
  return *pimpl_->writer_;
}

rosbag2_storage::StorageOptions &
Recorder::get_storage_options()
{
  return pimpl_->storage_options_;
}

rosbag2_transport::RecordOptions &
Recorder::get_record_options()
{
  return pimpl_->record_options_;
}

void Recorder::start_discovery()
{
  pimpl_->start_discovery();
}

void Recorder::stop_discovery()
{
  pimpl_->stop_discovery();
}

}  // namespace rosbag2_transport

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be
// discoverable when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rosbag2_transport::Recorder)
