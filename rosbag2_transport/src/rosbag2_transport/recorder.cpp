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

#include "rcutils/allocator.h"

#include "rclcpp/logging.hpp"
#include "rclcpp/clock.hpp"

#include "rosbag2_cpp/bag_events.hpp"
#include "rosbag2_cpp/writer.hpp"

#include "rosbag2_interfaces/srv/snapshot.hpp"

#include "rosbag2_storage/yaml.hpp"
#include "rosbag2_transport/qos.hpp"

#include "logging.hpp"
#include "rosbag2_transport/topic_filter.hpp"

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

  std::unordered_map<std::string, std::string> get_requested_or_available_topics();

  /// Public members for access by wrapper
  std::unordered_set<std::string> topics_warned_about_incompatibility_;
  std::shared_ptr<rosbag2_cpp::Writer> writer_;
  rosbag2_storage::StorageOptions storage_options_;
  rosbag2_transport::RecordOptions record_options_;
  std::atomic<bool> stop_discovery_ = false;
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

  // Serialize all currently offered QoS profiles for a topic into a YAML list.
  std::string serialized_offered_qos_profiles_for_topic(
    const std::vector<rclcpp::TopicEndpointInfo> & topics_endpoint_info) const;

  void warn_if_new_qos_for_subscribed_topic(const std::string & topic_name);

  void event_publisher_thread_main();
  bool event_publisher_thread_should_wake();

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

  std::atomic<bool> paused_ = false;
  std::atomic<bool> in_recording_ = false;
  std::shared_ptr<KeyboardHandler> keyboard_handler_;
  KeyboardHandler::callback_handle_t toggle_paused_key_callback_handle_ =
    KeyboardHandler::invalid_handle;

  // Variables for event publishing
  rclcpp::Publisher<rosbag2_interfaces::msg::WriteSplitEvent>::SharedPtr split_event_pub_;
  std::atomic<bool> event_publisher_thread_should_exit_ = false;
  std::atomic<bool> write_split_has_occurred_ = false;
  rosbag2_cpp::bag_events::BagSplitInfo bag_split_info_;
  std::mutex event_publisher_thread_mutex_;
  std::condition_variable event_publisher_thread_wake_cv_;
  std::thread event_publisher_thread_;
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
  stop_discovery_(record_options_.is_discovery_disabled),
  node(owner),
  paused_(record_options.start_paused),
  keyboard_handler_(std::move(keyboard_handler))
{
  if (record_options_.use_sim_time && record_options_.is_discovery_disabled) {
    throw std::runtime_error(
            "use_sim_time and is_discovery_disabled both set, but are incompatible settings. "
            "The /clock topic needs to be discovered to record with sim time.");
  }

  std::string key_str = enum_key_code_to_str(Recorder::kPauseResumeToggleKey);
  toggle_paused_key_callback_handle_ =
    keyboard_handler_->add_key_press_callback(
    [this](KeyboardHandler::KeyCode /*key_code*/,
    KeyboardHandler::KeyModifiers /*key_modifiers*/) {this->toggle_paused();},
    Recorder::kPauseResumeToggleKey);
  topic_filter_ = std::make_unique<TopicFilter>(record_options, node->get_node_graph_interface());
  // show instructions
  RCLCPP_INFO_STREAM(
    node->get_logger(),
    "Press " << key_str << " for pausing/resuming");

  for (auto & topic : record_options_.topics) {
    topic = rclcpp::expand_topic_or_service_name(
      topic, node->get_name(),
      node->get_namespace(), false);
  }
}

RecorderImpl::~RecorderImpl()
{
  keyboard_handler_->delete_key_press_callback(toggle_paused_key_callback_handle_);
  stop();
}

void RecorderImpl::stop()
{
  stop_discovery_ = true;
  if (discovery_future_.valid()) {
    auto status = discovery_future_.wait_for(2 * record_options_.topic_polling_interval);
    if (status != std::future_status::ready) {
      RCLCPP_ERROR_STREAM(
        node->get_logger(),
        "discovery_future_.wait_for(" << record_options_.topic_polling_interval.count() <<
          ") return status: " << (status == std::future_status::timeout ? "timeout" : "deferred"));
    }
  }
  paused_ = true;
  subscriptions_.clear();
  writer_->close();  // Call writer->close() to finalize current bag file and write metadata

  {
    std::lock_guard<std::mutex> lock(event_publisher_thread_mutex_);
    event_publisher_thread_should_exit_ = true;
  }
  event_publisher_thread_wake_cv_.notify_all();
  if (event_publisher_thread_.joinable()) {
    event_publisher_thread_.join();
  }
  in_recording_ = false;
  RCLCPP_INFO(node->get_logger(), "Recording stopped");
}

void RecorderImpl::record()
{
  if (in_recording_.exchange(true)) {
    RCLCPP_WARN_STREAM(
      node->get_logger(),
      "Called Recorder::record() while already in recording, dismissing request.");
    return;
  }
  paused_ = record_options_.start_paused;
  stop_discovery_ = record_options_.is_discovery_disabled;
  topic_qos_profile_overrides_ = record_options_.topic_qos_profile_overrides;
  if (record_options_.rmw_serialization_format.empty()) {
    throw std::runtime_error("No serialization format specified!");
  }

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

  // Start the thread that will publish events
  event_publisher_thread_ = std::thread(&RecorderImpl::event_publisher_thread_main, this);

  split_event_pub_ =
    node->create_publisher<rosbag2_interfaces::msg::WriteSplitEvent>("events/write_split", 1);
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
  RCLCPP_INFO(node->get_logger(), "Listening for topics...");
  if (!record_options_.use_sim_time) {
    subscribe_topics(get_requested_or_available_topics());
  }
  if (!record_options_.is_discovery_disabled) {
    discovery_future_ =
      std::async(std::launch::async, std::bind(&RecorderImpl::topics_discovery, this));
  }
  RCLCPP_INFO(node->get_logger(), "Recording...");
}

void RecorderImpl::event_publisher_thread_main()
{
  RCLCPP_INFO(node->get_logger(), "Event publisher thread: Starting");
  while (!event_publisher_thread_should_exit_.load()) {
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
  }
  RCLCPP_INFO(node->get_logger(), "Event publisher thread: Exiting");
}

bool RecorderImpl::event_publisher_thread_should_wake()
{
  return write_split_has_occurred_ || event_publisher_thread_should_exit_;
}

const rosbag2_cpp::Writer & RecorderImpl::get_writer_handle()
{
  return *writer_;
}

void RecorderImpl::pause()
{
  paused_.store(true);
  RCLCPP_INFO_STREAM(node->get_logger(), "Pausing recording.");
}

void RecorderImpl::resume()
{
  paused_.store(false);
  RCLCPP_INFO_STREAM(node->get_logger(), "Resuming recording.");
}

void RecorderImpl::toggle_paused()
{
  if (paused_.load()) {
    this->resume();
  } else {
    this->pause();
  }
}

bool RecorderImpl::is_paused()
{
  return paused_.load();
}

void RecorderImpl::topics_discovery()
{
  // If using sim time - wait until /clock topic received before even creating subscriptions
  if (record_options_.use_sim_time) {
    RCLCPP_INFO(
      node->get_logger(),
      "use_sim_time set, waiting for /clock before starting recording...");
    while (rclcpp::ok() && stop_discovery_ == false) {
      if (node->get_clock()->wait_until_started(record_options_.topic_polling_interval)) {
        break;
      }
    }
    if (node->get_clock()->started()) {
      RCLCPP_INFO(node->get_logger(), "Sim time /clock found, starting recording.");
    }
  }
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
        node->get_logger(),
        "All requested topics are subscribed. Stopping discovery...");
      return;
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
  for (const auto & i : all_topics) {
    if (subscriptions_.find(i.first) == subscriptions_.end()) {
      missing_topics.emplace(i.first, i.second);
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
        topic_with_type.first,
        topic_with_type.second,
        serialization_format_,
        serialized_offered_qos_profiles_for_topic(endpoint_infos),
        type_description_hash_for_topic(endpoint_infos),
      });
  }
}

void RecorderImpl::subscribe_topic(const rosbag2_storage::TopicMetadata & topic)
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
      node->get_logger(),
      "Subscribed to topic '" << topic.name << "'");
  } else {
    writer_->remove_topic(topic);
    subscriptions_.erase(topic.name);
  }
}

std::shared_ptr<rclcpp::GenericSubscription>
RecorderImpl::create_subscription(
  const std::string & topic_name, const std::string & topic_type, const rclcpp::QoS & qos)
{
  auto subscription = node->create_generic_subscription(
    topic_name,
    topic_type,
    qos,
    [this, topic_name, topic_type](std::shared_ptr<const rclcpp::SerializedMessage> message) {
      if (!paused_.load()) {
        writer_->write(message, topic_name, topic_type, node->get_clock()->now());
      }
    });
  return subscription;
}

std::string RecorderImpl::serialized_offered_qos_profiles_for_topic(
  const std::vector<rclcpp::TopicEndpointInfo> & topics_endpoint_info) const
{
  YAML::Node offered_qos_profiles;
  for (const auto & info : topics_endpoint_info) {
    offered_qos_profiles.push_back(Rosbag2QoS(info.qos_profile()));
  }
  return YAML::Dump(offered_qos_profiles);
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
  return Rosbag2QoS::adapt_request_to_offers(
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
    .append_parameter_override("use_sim_time", record_options.use_sim_time)),
  pimpl_(std::make_unique<RecorderImpl>(
      this, std::move(writer), keyboard_handler,
      storage_options, record_options))
{}

Recorder::~Recorder()
{}

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

void
Recorder::stop_discovery()
{
  pimpl_->stop_discovery_ = true;
}

}  // namespace rosbag2_transport
