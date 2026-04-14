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
#include <chrono>
#include <functional>
#include <future>
#include <map>
#include <memory>
#include <optional>
#include <regex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "rcutils/allocator.h"
#include "rclcpp/clock.hpp"
#include "rclcpp/event.hpp"
#include "rclcpp/logging.hpp"
#include "rmw/types.h"
#include "rosbag2_cpp/bag_events.hpp"
#include "rosbag2_cpp/service_utils.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_interfaces/srv/snapshot.hpp"
#include "rosbag2_storage/qos.hpp"
#include "logging.hpp"
#include "rosbag2_transport/config_options_from_node_params.hpp"
#include "rosbag2_transport/reader_writer_factory.hpp"
#include "rosbag2_transport/delayed_action_task_runner.hpp"
#include "rosbag2_transport/recorder_event_notifier.hpp"
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

  /// \brief Start recording.
  /// \details The record(uri) method will return almost immediately and recording will happen in
  /// background.
  /// \param uri If provided, it will override the storage_options.uri provided during construction.
  /// \return true if recording started successfully, false if recorder is already running.
  /// \throws std::exception if recording could not be started.
  bool record(const std::string & uri = "");

  /// @brief Add a new channel (topic) to the rosbag2 writer to be recorded.
  /// \details This is a direct Recorder API equivalent to the rosbag2_cpp::Writer::add_topic().
  /// \note This method does not require the message definition. The recorder will try to find the
  /// corresponding message definition by the given topic name and type.
  /// @param topic_name The name of the topic.
  /// @param topic_type The type of the topic.
  /// @param serialization_format The serialization format of the topic.
  /// @param type_description_hash REP-2011 type description hash of the topic.
  /// @param offered_qos_profiles The list of offered QoS profiles for the topic.
  void add_channel(
    const std::string & topic_name,
    const std::string & topic_type,
    const std::string & serialization_format = "memory_view",
    const std::string & type_description_hash = "",
    const std::vector<rclcpp::QoS> & offered_qos_profiles = {});

  /// \brief Add a new channel (topic) to the rosbag2 writer to be recorded.
  /// \details This is a direct Recorder API equivalent to the rosbag2_cpp::Writer::add_topic().
  /// \param topic_name The name of the topic.
  /// \param topic_type The type of the topic.
  /// \param message_definition_encoding The encoding technique used in
  /// the `encoded_message_definition` e.g. "ros2idl", "ros2msg", "apex_json" or "unknown" if
  /// encoded_message_definition is empty.
  /// \param encoded_message_definition The fully encoded message definition for this type.
  /// \param serialization_format The serialization format of the topic.
  /// \param type_description_hash REP-2011 type description hash of the topic.
  /// \param offered_qos_profiles The list of offered QoS profiles for the topic.
  void add_channel(
    const std::string & topic_name,
    const std::string & topic_type,
    const std::string & message_definition_encoding,
    const std::string & encoded_message_definition,
    const std::string & serialization_format = "memory_view",
    const std::string & type_description_hash = "",
    const std::vector<rclcpp::QoS> & offered_qos_profiles = {});

  /// @brief Write a serialized message to the bag file.
  /// \details This is a direct Recorder API equivalent to the rosbag2_cpp::Writer::write().
  /// \note This method assumes that the topic has already been created via add_channel().
  /// \note If recorder is in pause mode, this method will return without writing anything.
  /// \note This overload uses only publication timestamp. The received timestamp will be taken
  /// from the underlying node's clock at the time of writing.
  /// @param serialized_data The serialized message data to write.
  /// @param topic_name The name of the topic the message belongs to.
  /// @param pub_timestamp The original or publication timestamp of the message in nanoseconds.
  /// @param sequence_number An optional sequence number of the message. If non-zero, sequence
  /// numbers should be unique per channel and increasing over time.
  /// \throws std::runtime_error if topic has not been added via add_channel().
  void write_message(
    std::shared_ptr<rcutils_uint8_array_t> serialized_data,
    const std::string & topic_name,
    const rcutils_time_point_value_t & pub_timestamp,
    uint32_t sequence_number = 0);

  /// @brief Write a serialized message to the bag file with receive timestamp.
  /// \details This is a direct Recorder API equivalent to the rosbag2_cpp::Writer::write().
  /// \note This method assumes that the topic has already been created via add_channel().
  /// \note If recorder is in pause mode, this method will return without writing anything.
  /// @param serialized_data The serialized message data to write.
  /// @param topic_name The name of the topic the message belongs to.
  /// @param pub_timestamp The original or publication timestamp of the message in nanoseconds.
  /// @param recv_timestamp The timestamp of the message in nanoseconds when message was received.
  /// @param sequence_number An optional sequence number of the message. If non-zero, sequence
  /// numbers should be unique per channel and increasing over time.
  /// \throws std::runtime_error if topic has not been added via add_channel().
  void write_message(
    std::shared_ptr<rcutils_uint8_array_t> serialized_data,
    const std::string & topic_name,
    const rcutils_time_point_value_t & pub_timestamp,
    const rcutils_time_point_value_t & recv_timestamp,
    uint32_t sequence_number = 0);

  /// @brief Updates recorder about lost messages on transport layer.
  /// @details This a direct recorder API and this method is expected to be called when messages
  /// are lost in the transport layer.
  /// The recorder may use this information for logging or metrics.
  /// @param topic_name The name of the topic.
  /// @param qos_msgs_lost_info Information about lost messages.
  void on_messages_lost_in_transport(
    const std::string & topic_name,
    const rclcpp::QOSMessageLostInfo & qos_msgs_lost_info);

  /// @brief Get total number of messages lost in transport layer.
  /// @return Total number of messages lost in transport layer.
  [[nodiscard]]
  uint64_t get_total_num_messages_lost_in_transport() const;

  /// @brief Stopping recording and closing writer.
  /// The record(uri) can be called again after stop().
  void stop();

  //// @brief Split the current bagfile and open a new one.
  /// @return true if split was successful, false if recording is not active.
  /// \throws std::exception if underlying writer fails to split the bagfile.
  bool split_bagfile();

  /// Get a const reference to the underlying rosbag2 writer.
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

  /// Return the current discovery state.
  bool is_discovery_running() const;

  std::unordered_map<std::string, std::string> get_requested_or_available_topics();

  void read_static_topics() noexcept;

  /// Public members for access by wrapper
  std::unordered_set<std::string> topics_warned_about_incompatibility_;
  std::shared_ptr<rosbag2_cpp::Writer> writer_;
  rosbag2_storage::StorageOptions storage_options_;
  rosbag2_transport::RecordOptions record_options_;
  std::unordered_map<std::string, std::shared_ptr<rclcpp::SubscriptionBase>> subscriptions_;

  std::vector<std::pair<std::string, std::string>> static_topics_{};  // topic_name, topic_type
  Recorder::OnStartRecordingCallback on_start_recording_callback_{};

private:
  using ResumeRequest = rosbag2_interfaces::srv::Resume::Request;
  using ResumeResponse = rosbag2_interfaces::srv::Resume::Response;
  using ResumeCallbackResponse = ResumeResponse::SharedPtr;

  using SplitBagFileResponse = rosbag2_interfaces::srv::SplitBagfile::Response;
  using SplitBagFileRequest = rosbag2_interfaces::srv::SplitBagfile::Request;
  using SplitBagFileCallbackResponse = SplitBagFileResponse::SharedPtr;

  enum class SplitMode : int32_t
  {
    NodeTime = SplitBagFileRequest::SPLIT_MODE_NODE_TIME,
    PublishTime = SplitBagFileRequest::SPLIT_MODE_PUBLISH_TIME,
    ReceiveTime = SplitBagFileRequest::SPLIT_MODE_RECEIVE_TIME,
  };

  enum class ResumeMode : int32_t
  {
    NodeTime = ResumeRequest::RESUME_MODE_NODE_TIME,
    PublishTime = ResumeRequest::RESUME_MODE_PUBLISH_TIME,
    ReceiveTime = ResumeRequest::RESUME_MODE_RECEIVE_TIME,
  };

  /// \brief Return codes for split bag file operation.
  enum class SplitBagFileReturnCode : int32_t
  {
    Success = SplitBagFileResponse::RETURN_CODE_SUCCESS,
    NotRecording = SplitBagFileResponse::RETURN_CODE_NOT_RECORDING,
    InvalidSplitMode = SplitBagFileResponse::RETURN_CODE_INVALID_SPLIT_MODE,
    InvalidTrackingTopic = SplitBagFileResponse::RETURN_CODE_INVALID_TRACKING_TOPIC,
    SplitFailed = SplitBagFileResponse::RETURN_CODE_SPLIT_FAILED
  };

  /// \brief Return codes for resume operation.
  enum class ResumeReturnCode : int32_t
  {
    Success = ResumeResponse::RETURN_CODE_SUCCESS,
    InvalidResumeMode = ResumeResponse::RETURN_CODE_INVALID_RESUME_MODE,
    InvalidTrackingTopic = ResumeResponse::RETURN_CODE_INVALID_TRACKING_TOPIC,
    ResumeFailed = ResumeResponse::RETURN_CODE_RESUME_FAILED
  };

  /// \brief Pending bag split state for timestamp-based split requests.
  struct PendingBagSplitState
  {
    std::string tracking_topic_name{};
    int64_t time_ns = kNoPendingPublishSplit;
    SplitMode mode = SplitMode::NodeTime;
  };

  /// \brief Pending resume state for timestamp-based resume requests.
  struct PendingResumeState
  {
    std::string tracking_topic_name{};
    int64_t time_ns = kNoPendingResumeRequest;
    ResumeMode mode = ResumeMode::NodeTime;
  };

  /// \brief Class to track last seen publish and receive timestamps for topics.
  /// Used for handling pending split requests based on message timestamps.
  class LastSeenTimestamps
  {
    // *INDENT-OFF* (prevent uncrustify from making unnecessary indents here)
    public:
      using PubLogTimestamps = std::pair<rcutils_time_point_value_t, rcutils_time_point_value_t>;

      /// \brief Get last seen timestamps for a topic.
      /// \param topic_name The name of the topic. If empty, gets global last seen timestamps.
      /// \return The last seen publish and receive timestamps for the topic. If no timestamps
      /// are found, returns zeros.
      PubLogTimestamps get(const std::string & topic_name)
      {
        std::lock_guard<std::mutex> lock(last_seen_timestamps_mutex_);
        PubLogTimestamps timestamps{0, 0};
        auto it = last_seen_timestamps_by_topic_.find(topic_name);
        if (it != last_seen_timestamps_by_topic_.end()) {
          timestamps.first = it->second.first;
          timestamps.second = it->second.second;
        }
        return timestamps;
      }

      /// \brief Update last seen timestamps for a topic.
      /// \param topic_name The name of the topic. If empty, updates global last seen timestamps.
      void update(const std::string & topic_name,
                  const rcutils_time_point_value_t & pub_time,
                  const rcutils_time_point_value_t & log_time)
      {
        std::lock_guard<std::mutex> lock(last_seen_timestamps_mutex_);
        // Update global last seen timestamps
        auto & [last_seen_global_pub_time, last_seen_global_log_time] =
          last_seen_timestamps_by_topic_[""];
        last_seen_global_pub_time = pub_time;
        last_seen_global_log_time = log_time;
        // Update per-topic last seen timestamps
        if (!topic_name.empty()) {
          auto & [last_seen_pub_time, last_seen_log_time] =
            last_seen_timestamps_by_topic_[topic_name];
          last_seen_pub_time = pub_time;
          last_seen_log_time = log_time;
        }
      }

      /// \brief Reset all last seen timestamps to zero.
      void reset()
      {
        std::lock_guard<std::mutex> lock(last_seen_timestamps_mutex_);
        last_seen_timestamps_by_topic_.clear();
      }

    private:
      /// \brief Mutex to protect access to last seen timestamps.
      std::mutex last_seen_timestamps_mutex_;
      /// \brief Map of topic name to last seen publish and receive timestamps.
      /// Note: The empty string key is used to track last seen timestamps for unfiltered split
      /// requests.
      std::unordered_map<std::string, PubLogTimestamps> last_seen_timestamps_by_topic_;
    // *INDENT-ON*
  };

  /// \brief Convert an integer split mode to the corresponding SplitMode enum.
  /// \param split_mode The integer split mode from the service request.
  /// \return An optional SplitMode enum. If the integer is invalid, returns std::nullopt.
  static std::optional<SplitMode> get_split_mode(int32_t split_mode);

  /// \brief Convert an integer resume mode to the corresponding ResumeMode enum.
  /// \param resume_mode The integer resume mode from the service request.
  /// \return An optional ResumeMode enum. If the integer is invalid, returns std::nullopt.
  static std::optional<ResumeMode> get_resume_mode(int32_t resume_mode);

  /// \brief Convert a SplitMode enum to a human-readable string.
  /// \param split_mode The SplitMode enum.
  /// \return A string representation of the SplitMode.
  static const char * to_string(SplitMode split_mode);

  /// \brief Convert a ResumeMode enum to a human-readable string.
  /// \param resume_mode The ResumeMode enum.
  /// \return A string representation of the ResumeMode.
  static const char * to_string(ResumeMode resume_mode);

  /// \brief Convert enum class to its underlying type.
  template<typename E>
  static constexpr std::underlying_type_t<E> to_underlying_type(E e) noexcept
  {
    static_assert(std::is_enum<E>::value, "E must be an enum type");
    return static_cast<std::underlying_type_t<E>>(e);
  }

  void create_control_services();

  void topics_discovery() noexcept;

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

  /// \brief Helper wrapper function to set a service response as success.
  template<typename ResponseT>
  void set_service_success(
    ResponseT & response,
    int32_t return_code = kServiceReturnCodeSuccess) const
  {
    response->return_code = return_code;
    response->error_string.clear();
  }

  /// \brief Helper wrapper function to set a service response as error.
  template<typename ResponseT>
  void set_service_error(
    ResponseT & response, const std::string & error_string,
    int32_t error_code = kServiceReturnCodeError) const
  {
    response->return_code = error_code;
    response->error_string = error_string;
  }

  /// \brief Convert a builtin_interfaces::msg::Time to an optional rclcpp::Time.
  /// \param time_msg The time message to convert.
  /// \return An optional rclcpp::Time. If time_msg is zero, returns std::nullopt.
  std::optional<rclcpp::Time> optional_time_from_request(
    const builtin_interfaces::msg::Time & time_msg) const;

  /// \brief Determine if an action task should be executed immediately based on the target time.
  /// \param target_time The time at which the action task should be executed.
  /// \return true if the action task should be executed immediately, false otherwise.
  bool should_execute_immediately(const std::optional<rclcpp::Time> & target_time) const;

  // *INDENT-OFF*
  /// \brief Handle a pending resume request based on publish/receive timestamps.
  void handle_pending_resume_request(const std::string & topic_name,
                                     const rcutils_time_point_value_t & publish_time,
                                     const rcutils_time_point_value_t & receive_time) noexcept;

  /// \brief Handle a timer-based resume request.
  /// \param resume_time The time at which to resume recording. If std::nullopt, resume
  /// immediately.
  /// \param response The service response to populate.
  void handle_timer_resume_request(std::optional<rclcpp::Time> resume_time,
                                   ResumeCallbackResponse & response);

  /// \brief Handle a timestamp-based resume request.
  /// \param resume_time The time at which to resume recording. If std::nullopt, resume
  /// immediately.
  /// \param resume_mode The mode to use for the resume (publish time, receive time).
  /// \param topic_name The topic to track for the resume. If empty, track all topics.
  /// \param response The service response to populate.
  void handle_timestamp_resume_request(const std::optional<rclcpp::Time> & resume_time,
                                       ResumeMode resume_mode,
                                       const std::string & topic_name,
                                       ResumeCallbackResponse & response);

  /// \brief Handle a pending bag split request based on publish/receive timestamps.
  void handle_pending_bag_split_request(const std::string & topic_name,
                                        const rcutils_time_point_value_t & publish_time,
                                        const rcutils_time_point_value_t & receive_time) noexcept;

  /// \brief Handle a timer-based bag split request.
  /// \param split_time The time at which to split the bag file. If std::nullopt, split immediately.
  /// \param response The service response to populate.
  void handle_timer_bag_split_request(std::optional<rclcpp::Time> split_time,
                                      SplitBagFileCallbackResponse & response);

  /// \brief Handle a timestamp-based bag split request.
  /// \param split_time The time at which to split the bag file. If std::nullopt, split immediately.
  /// \param split_mode The mode to use for the split (publish time, receive time).
  /// \param topic_name The topic to track for the split. If empty, track all topics.
  /// \param response The service response to populate.
  void handle_timestamp_bag_split_request(const std::optional<rclcpp::Time> & split_time,
                                          SplitMode split_mode,
                                          const std::string & topic_name,
                                          SplitBagFileCallbackResponse & response);
  // *INDENT-ON*

  /// \brief Attempt an immediate split of the bag file.
  void attempt_immediate_bag_split(SplitBagFileCallbackResponse & response);

  rclcpp::Node * node;
  std::unique_ptr<TopicFilter> topic_filter_;
  rclcpp::Event::SharedPtr discovery_graph_event_;
  std::future<void> discovery_future_;
  std::unordered_map<std::string, rclcpp::QoS> topic_qos_profile_overrides_;
  std::unordered_set<std::string> topic_unknown_types_;
  rclcpp::Service<rosbag2_interfaces::srv::IsDiscoveryRunning>::SharedPtr srv_is_discovery_running_;
  rclcpp::Service<rosbag2_interfaces::srv::IsPaused>::SharedPtr srv_is_paused_;
  rclcpp::Service<rosbag2_interfaces::srv::Pause>::SharedPtr srv_pause_;
  rclcpp::Service<rosbag2_interfaces::srv::Record>::SharedPtr srv_record_;
  rclcpp::Service<rosbag2_interfaces::srv::Resume>::SharedPtr srv_resume_;
  rclcpp::Service<rosbag2_interfaces::srv::Snapshot>::SharedPtr srv_snapshot_;
  rclcpp::Service<rosbag2_interfaces::srv::SplitBagfile>::SharedPtr srv_split_bagfile_;
  rclcpp::Service<rosbag2_interfaces::srv::StartDiscovery>::SharedPtr srv_start_discovery_;
  rclcpp::Service<rosbag2_interfaces::srv::Stop>::SharedPtr srv_stop_;
  rclcpp::Service<rosbag2_interfaces::srv::StopDiscovery>::SharedPtr srv_stop_discovery_;

  std::mutex start_stop_transition_mutex_;
  std::mutex discovery_mutex_;
  std::atomic<bool> discovery_running_ = false;
  std::atomic_uchar paused_ = 0;
  std::atomic<bool> in_recording_ = false;
  std::shared_ptr<KeyboardHandler> keyboard_handler_;
  KeyboardHandler::callback_handle_t toggle_paused_key_callback_handle_ =
    KeyboardHandler::invalid_handle;

  std::unique_ptr<RecorderEventNotifier> event_notifier_;
  DelayedActionTaskRunner action_task_runner_;
  static constexpr int64_t kNoPendingPublishSplit = -1;
  static constexpr int64_t kNoPendingResumeRequest = -1;

  /// \brief Mutex to protect access to the pending bag split request.
  std::mutex pending_bag_split_request_mutex_;

  /// \brief Pending bag split request.
  std::unique_ptr<PendingBagSplitState> pending_bag_split_request_;

  /// \brief Mutex to protect access to the pending resume request.
  std::mutex pending_resume_request_mutex_;

  /// \brief Pending resume request.
  std::unique_ptr<PendingResumeState> pending_resume_request_;

  /// \brief Keeps last seen publish and receive timestamps for each topic.
  /// Used for handling pending split requests based on message timestamps.
  LastSeenTimestamps last_seen_timestamps_;

  static constexpr int32_t kServiceReturnCodeSuccess = 0;
  // Ensure service return code mapping stays consistent with `SplitBagFileReturnCode`
  static_assert(static_cast<int32_t>(SplitBagFileReturnCode::Success) == kServiceReturnCodeSuccess,
                "SplitBagFileReturnCode::Success expected to be equal kServiceReturnCodeSuccess");
  // Ensure service return code mapping stays consistent with `ResumeReturnCode`
  static_assert(static_cast<int32_t>(ResumeReturnCode::Success) == kServiceReturnCodeSuccess,
                "ResumeReturnCode::Success expected to be equal kServiceReturnCodeSuccess");
  static constexpr int32_t kServiceReturnCodeError = 1;
};  // class RecorderImpl

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
  event_notifier_(std::make_unique<RecorderEventNotifier>(node, record_options)),
  action_task_runner_(node)
{
  event_notifier_->set_messages_lost_statistics_max_publishing_rate(
    record_options.statistics_max_publishing_rate);

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

  read_static_topics();
  for (auto & [topic_name, _] : static_topics_) {
    topic_name = rclcpp::expand_topic_or_service_name(
      topic_name, node->get_name(),
      node->get_namespace(), false);
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

  std::unordered_map<std::string, size_t> expanded_repeat_transient_local_messages;
  for (const auto & [topic_name, depth] : record_options_.repeat_transient_local_messages) {
    auto expanded_topic_name = rclcpp::expand_topic_or_service_name(
      topic_name, node->get_name(),
      node->get_namespace(), false);
    expanded_repeat_transient_local_messages[expanded_topic_name] = depth;
  }
  record_options_.repeat_transient_local_messages =
    std::move(expanded_repeat_transient_local_messages);

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

  topic_filter_ = std::make_unique<TopicFilter>(record_options, node->get_node_graph_interface(),
      false, static_topics_);

  action_task_runner_.start();
  create_control_services();
}

RecorderImpl::~RecorderImpl()
{
  if (keyboard_handler_ &&
    (toggle_paused_key_callback_handle_ != KeyboardHandler::invalid_handle))
  {
    keyboard_handler_->delete_key_press_callback(toggle_paused_key_callback_handle_);
  }
  action_task_runner_.stop();
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
  // Explicitly disable all subscription's callbacks to avoid UB and receiving new messages on
  // deleted subscriptions. Note: The callbacks propagated to the executor and may still be in the
  // executor's queue, but they will no longer be called after this point.
  for (auto & [_, subscription] : subscriptions_) {
    subscription->disable_callbacks();
  }
  subscriptions_.clear();
  writer_->close();  // Call writer->close() to finalize current bag file and write metadata
  {  // Clear pending split request if any
    std::lock_guard<std::mutex> lock(pending_bag_split_request_mutex_);
    pending_bag_split_request_.reset();
  }
  {  // Clear pending resume request if any
    std::lock_guard<std::mutex> lock(pending_resume_request_mutex_);
    pending_resume_request_.reset();
  }

  last_seen_timestamps_.reset();  // Clear last seen timestamps

  in_recording_ = false;
  RCLCPP_INFO(node->get_logger(), "Recording stopped");

  auto num_messages_lost_in_recorder = event_notifier_->get_total_num_messages_lost_in_recorder();
  auto num_messages_lost_on_transport = event_notifier_->get_total_num_messages_lost_in_transport();

  if (num_messages_lost_on_transport > 0) {
    RCLCPP_WARN(node->get_logger(),
                "Number of messages lost on the transport layer: %lu",
                num_messages_lost_on_transport);
  }

  if (num_messages_lost_in_recorder > 0) {
    RCLCPP_WARN(node->get_logger(),
                "Number of messages lost in the recorder: %lu",
                num_messages_lost_in_recorder);
  }
}

bool RecorderImpl::record(const std::string & uri)
{
  std::lock_guard<std::mutex> state_lock(start_stop_transition_mutex_);
  if (in_recording_) {
    RCLCPP_WARN_STREAM(node->get_logger(),
      "Called Recorder::record(uri) while already in recording, dismissing request.");
    return false;
  }
  if (!uri.empty()) {
    storage_options_.uri = uri;
  }
  RCLCPP_INFO(node->get_logger(), "Starting recording to '%s'", storage_options_.uri.c_str());

  topic_qos_profile_overrides_ = record_options_.topic_qos_profile_overrides;
  // Check serialization format options
  if (!record_options_.rmw_serialization_format.empty() &&
    record_options_.output_serialization_format.empty())
  {
    RCLCPP_WARN(node->get_logger(),
      "The rmw_serialization_format option is deprecated and will be removed in a future release.\n"
      "Please use output_serialization_format instead.");
    record_options_.output_serialization_format = record_options_.rmw_serialization_format;
  }
  if (record_options_.input_serialization_format.empty()) {
    record_options_.input_serialization_format = rmw_get_serialization_format();
    RCLCPP_WARN(node->get_logger(),
      "No input serialization format specified, using default rmw serialization format: '%s'.",
      record_options_.input_serialization_format.c_str());
  }
  if (record_options_.output_serialization_format.empty()) {
    record_options_.output_serialization_format = rmw_get_serialization_format();
    RCLCPP_WARN(node->get_logger(),
      "No output serialization format specified, using rmw serialization format. '%s'.",
      record_options_.output_serialization_format.c_str());
  }

  event_notifier_->reset_total_num_messages_lost_in_transport();
  event_notifier_->reset_total_num_messages_lost_in_recorder();

  // Check if storage_options.uri already exists and try to add '(n)' postfix
  namespace fs = std::filesystem;
  fs::path storage_path(storage_options_.uri);
  if (fs::is_directory(storage_path)) {
    RCLCPP_WARN_STREAM(node->get_logger(),
                       "Bag directory '" << storage_path.c_str() << "' already exists.");
    for (size_t i = 1U; i < std::numeric_limits<size_t>::max(); i++) {
      fs::path new_path = storage_path;
      new_path += "(" + std::to_string(i) + ")";
      if (!fs::exists(new_path)) {
        RCLCPP_WARN_STREAM(node->get_logger(),
                           "Changing bag directory to '" << new_path.c_str() << "'.");
        storage_options_.uri = new_path.generic_string();
        storage_path = new_path;
        break;
      }
    }
  }
  if (fs::is_directory(storage_path)) {
    throw std::runtime_error{
            "Failed to derive non-existent directory for the new Rosbag2 recording. "
            "Please specify non existent uri explicitly."};
  }

  writer_->open(
    storage_options_,
    {record_options_.input_serialization_format, record_options_.output_serialization_format});

  rosbag2_cpp::bag_events::WriterEventCallbacks callbacks;
  callbacks.write_split_callback =
    [this](rosbag2_cpp::bag_events::BagSplitInfo & info) {
      event_notifier_->on_bag_split_in_recorder(info);
    };
  callbacks.messages_lost_callback =
    [this](const std::vector<rosbag2_cpp::bag_events::MessagesLostInfo> & msgs_lost_info) {
      event_notifier_->on_messages_lost_in_recorder(msgs_lost_info);
    };
  writer_->add_event_callbacks(callbacks);

  if (!record_options_.use_sim_time) {
    subscribe_topics(get_requested_or_available_topics());
  }

  // Disable discovery if only static topics defined
  if (!static_topics_.empty() &&
    !(record_options_.all_topics || !record_options_.topics.empty() ||
    record_options_.all_services || !record_options_.services.empty() ||
    record_options_.all_actions || !record_options_.actions.empty() ||
    !record_options_.regex.empty()))
  {
    record_options_.is_discovery_disabled = true;
  }

  if (!record_options_.is_discovery_disabled) {
    start_discovery();
    RCLCPP_INFO(node->get_logger(), "Listening for topics...");
  }
  if (paused_.load()) {
    if (!record_options_.disable_keyboard_controls) {
      RCLCPP_INFO(
        node->get_logger(), "Wait for recording: Press %s to start.",
        enum_key_code_to_str(Recorder::kPauseResumeToggleKey).c_str());
    }
  } else {
    RCLCPP_INFO(node->get_logger(), "Recording...");
  }
  in_recording_ = true;

  if (on_start_recording_callback_) {
    on_start_recording_callback_();
  }
  return true;
}

bool RecorderImpl::split_bagfile()
{
  std::lock_guard<std::mutex> state_lock(start_stop_transition_mutex_);
  if (!in_recording_.load()) {
    RCLCPP_WARN(node->get_logger(),
    "Received SplitBagfile request while not in recording. Ignoring request.");
    return false;
  }

  writer_->split_bagfile();
  return true;
}

void RecorderImpl::create_control_services()
{
  // Only expose snapshot service when mode is enabled
  if (storage_options_.snapshot_mode) {
    srv_snapshot_ = node->create_service<rosbag2_interfaces::srv::Snapshot>(
      "~/snapshot",
      [this](
        const std::shared_ptr<rmw_request_id_t>/* request_header */,
        const std::shared_ptr<rosbag2_interfaces::srv::Snapshot::Request>/* request */,
        const std::shared_ptr<rosbag2_interfaces::srv::Snapshot::Response> response)
      {
        std::lock_guard<std::mutex> state_lock(start_stop_transition_mutex_);
        if (!in_recording_.load()) {
          RCLCPP_WARN(node->get_logger(),
            "Received Snapshot request while not in recording. Ignoring request.");
          response->success = false;
        } else {
          try {
            response->success = writer_->take_snapshot();
          } catch (std::exception & e) {
            RCLCPP_ERROR(node->get_logger(), "Error during Snapshot request: %s", e.what());
            response->success = false;
          }
        }
      }
    );
  }

  srv_split_bagfile_ = node->create_service<rosbag2_interfaces::srv::SplitBagfile>(
    "~/split_bagfile",
    [this](
      const std::shared_ptr<rmw_request_id_t>/* request_header */,
      const std::shared_ptr<rosbag2_interfaces::srv::SplitBagfile::Request> request,
      std::shared_ptr<rosbag2_interfaces::srv::SplitBagfile::Response> response)
    {
      auto split_time = optional_time_from_request(request->split_time);
      const auto split_mode = get_split_mode(request->split_mode);
      if (!split_mode.has_value()) {
        RCLCPP_ERROR(node->get_logger(),
                     "Invalid split_mode %d for SplitBagfile request.", request->split_mode);
        set_service_error(response,
                          "Invalid split_mode for SplitBagfile request.",
                          to_underlying_type(SplitBagFileReturnCode::InvalidSplitMode));
        return;
      }
      if (split_mode.value() == SplitMode::NodeTime) {
        handle_timer_bag_split_request(split_time, response);
        return;
      }
      std::string split_tracking_topic_name;
      if (!request->tracking_topic_name.empty()) {
        try {
          split_tracking_topic_name = rclcpp::expand_topic_or_service_name(
            request->tracking_topic_name, node->get_name(), node->get_namespace(), false);
        } catch (const std::exception & e) {
          RCLCPP_ERROR(node->get_logger(),
                       "Invalid split topic name '%s' for SplitBagfile request: %s",
                       request->tracking_topic_name.c_str(), e.what());
          set_service_error(response,
                            "Invalid tracking_topic_name for SplitBagfile request.",
                            to_underlying_type(SplitBagFileReturnCode::InvalidTrackingTopic));
          return;
        }
      }
      handle_timestamp_bag_split_request(split_time,
                                         split_mode.value(),
                                         split_tracking_topic_name,
                                         response);
    }
  );

  srv_start_discovery_ = node->create_service<rosbag2_interfaces::srv::StartDiscovery>(
    "~/start_discovery",
    [this](
      const std::shared_ptr<rmw_request_id_t>/* request_header */,
      const std::shared_ptr<rosbag2_interfaces::srv::StartDiscovery::Request>/* request */,
      const std::shared_ptr<rosbag2_interfaces::srv::StartDiscovery::Response> response)
    {
      std::lock_guard<std::mutex> state_lock(start_stop_transition_mutex_);
      if (!in_recording_.load()) {
        RCLCPP_WARN(node->get_logger(),
          "Received StartDiscovery request while not in recording. Ignoring request.");
        set_service_error(response, "Recorder is not currently recording.");
      } else if (discovery_running_.load()) {
        RCLCPP_WARN(node->get_logger(),
          "Received StartDiscovery request while discovery is already running. Ignoring request.");
        set_service_error(response, "Discovery is already running.");
      } else {
        try {
          this->start_discovery();
          set_service_success(response);
        } catch (const std::exception & e) {
          RCLCPP_ERROR(node->get_logger(), "Error during StartDiscovery request: %s", e.what());
          set_service_error(response, e.what());
        }
      }
    }
  );

  srv_stop_discovery_ = node->create_service<rosbag2_interfaces::srv::StopDiscovery>(
    "~/stop_discovery",
    [this](
      const std::shared_ptr<rmw_request_id_t>/* request_header */,
      const std::shared_ptr<rosbag2_interfaces::srv::StopDiscovery::Request>/* request */,
      const std::shared_ptr<rosbag2_interfaces::srv::StopDiscovery::Response> response)
    {
      if (!discovery_running_.load()) {
        RCLCPP_WARN(node->get_logger(),
          "Received StopDiscovery request while discovery is not running. Ignoring request.");
        set_service_error(response, "Discovery is not running.");
      } else {
        try {
          this->stop_discovery();
          set_service_success(response);
        } catch (const std::exception & e) {
          RCLCPP_ERROR(node->get_logger(), "Error during StopDiscovery request: %s", e.what());
          set_service_error(response, e.what());
        }
      }
    }
  );

  srv_is_discovery_running_ = node->create_service<rosbag2_interfaces::srv::IsDiscoveryRunning>(
    "~/is_discovery_running",
    [this](
      const std::shared_ptr<rmw_request_id_t>/* request_header */,
      const std::shared_ptr<rosbag2_interfaces::srv::IsDiscoveryRunning::Request>/* request */,
      const std::shared_ptr<rosbag2_interfaces::srv::IsDiscoveryRunning::Response> response)
    {
      response->running = is_discovery_running();
    });

  srv_record_ = node->create_service<rosbag2_interfaces::srv::Record>(
    "~/record",
    [this](
      const std::shared_ptr<rmw_request_id_t>/* request_header */,
      const std::shared_ptr<rosbag2_interfaces::srv::Record::Request> request,
      const std::shared_ptr<rosbag2_interfaces::srv::Record::Response> response)
    {
      auto start_time = optional_time_from_request(request->start_time);
      auto uri = request->uri;
      if (should_execute_immediately(start_time)) {
        try {
          if (this->record(uri)) {
            set_service_success(response);
          } else {
            set_service_error(response, "Called record(uri) while already in recording.");
          }
        } catch (const std::exception & e) {
          RCLCPP_ERROR(node->get_logger(), "Error starting on 'Record' request: %s", e.what());
          set_service_error(response, e.what());
        }
      } else {
        auto action_task = [this, uri]() {
          try {
            (void)this->record(uri);
          } catch (const std::exception & e) {
            RCLCPP_ERROR(node->get_logger(), "Error starting on Record request: %s", e.what());
          }
        };
        action_task_runner_.schedule(start_time.value(), std::move(action_task), "Start recording");
        set_service_success(response);
      }
    }
  );

  srv_stop_ = node->create_service<rosbag2_interfaces::srv::Stop>(
    "~/stop",
    [this](
      const std::shared_ptr<rmw_request_id_t>/* request_header */,
      const std::shared_ptr<rosbag2_interfaces::srv::Stop::Request>/* request */,
      const std::shared_ptr<rosbag2_interfaces::srv::Stop::Response> response)
    {
      if (!in_recording_) {
        RCLCPP_WARN(node->get_logger(),
          "Received Stop request while not in recording. Ignoring request.");
        set_service_error(response, "Recorder is already stopped.");
      } else {
        try {
          this->stop();
          set_service_success(response);
        } catch (const std::exception & e) {
          RCLCPP_ERROR(node->get_logger(), "Error during Stop request: %s", e.what());
          set_service_error(response, e.what());
        }
      }
    }
  );

  srv_pause_ = node->create_service<rosbag2_interfaces::srv::Pause>(
    "~/pause",
    [this](
      const std::shared_ptr<rmw_request_id_t>/* request_header */,
      const std::shared_ptr<rosbag2_interfaces::srv::Pause::Request>/* request */,
      const std::shared_ptr<rosbag2_interfaces::srv::Pause::Response>/* response */)
    {
      std::lock_guard<std::mutex> state_lock(start_stop_transition_mutex_);
      // Note: We don't check if we are in recording here, as pausing when not recording is a valid
      // operation and can be used to set the initial state before starting recording.
      this->pause();
    });

  srv_resume_ = node->create_service<rosbag2_interfaces::srv::Resume>(
    "~/resume",
    [this](
      const std::shared_ptr<rmw_request_id_t>/* request_header */,
      const std::shared_ptr<rosbag2_interfaces::srv::Resume::Request> request,
      std::shared_ptr<rosbag2_interfaces::srv::Resume::Response> response)
    {
      auto resume_time = optional_time_from_request(request->resume_time);
      const auto resume_mode = get_resume_mode(request->resume_mode);
      if (!resume_mode.has_value()) {
        RCLCPP_ERROR(node->get_logger(),
                     "Invalid resume_mode %d for Resume request.", request->resume_mode);
        set_service_error(response,
                          "Invalid resume_mode for Resume request.",
                          to_underlying_type(ResumeReturnCode::InvalidResumeMode));
        return;
      }
      if (resume_mode.value() == ResumeMode::NodeTime) {
        handle_timer_resume_request(resume_time, response);
        return;
      }
      std::string resume_tracking_topic_name;
      if (!request->tracking_topic_name.empty()) {
        try {
          resume_tracking_topic_name = rclcpp::expand_topic_or_service_name(
            request->tracking_topic_name, node->get_name(), node->get_namespace(), false);
        } catch (const std::exception & e) {
          RCLCPP_ERROR(node->get_logger(),
                       "Invalid tracking topic name '%s' for Resume request: %s",
                       request->tracking_topic_name.c_str(), e.what());
          set_service_error(response,
                            "Invalid tracking_topic_name for Resume request.",
                            to_underlying_type(ResumeReturnCode::InvalidTrackingTopic));
          return;
        }
      }
      handle_timestamp_resume_request(resume_time,
                                      resume_mode.value(),
                                      resume_tracking_topic_name,
                                      response);
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
}

std::optional<rclcpp::Time> RecorderImpl::optional_time_from_request(
  const builtin_interfaces::msg::Time & time_msg) const
{
  if (time_msg.sec == 0 && time_msg.nanosec == 0) {
    return std::nullopt;
  }
  return rclcpp::Time(time_msg, node->get_clock()->get_clock_type());
}

std::optional<RecorderImpl::SplitMode> RecorderImpl::get_split_mode(int32_t split_mode)
{
  switch (split_mode) {
    case SplitBagFileRequest::SPLIT_MODE_NODE_TIME:
      return SplitMode::NodeTime;
    case SplitBagFileRequest::SPLIT_MODE_PUBLISH_TIME:
      return SplitMode::PublishTime;
    case SplitBagFileRequest::SPLIT_MODE_RECEIVE_TIME:
      return SplitMode::ReceiveTime;
    default:
      return std::nullopt;
  }
}

std::optional<RecorderImpl::ResumeMode> RecorderImpl::get_resume_mode(int32_t resume_mode)
{
  switch (resume_mode) {
    case ResumeRequest::RESUME_MODE_NODE_TIME:
      return ResumeMode::NodeTime;
    case ResumeRequest::RESUME_MODE_PUBLISH_TIME:
      return ResumeMode::PublishTime;
    case ResumeRequest::RESUME_MODE_RECEIVE_TIME:
      return ResumeMode::ReceiveTime;
    default:
      return std::nullopt;
  }
}

const char * RecorderImpl::to_string(SplitMode split_mode)
{
  switch (split_mode) {
    case SplitMode::NodeTime:
      return "node";
    case SplitMode::PublishTime:
      return "publish";
    case SplitMode::ReceiveTime:
      return "receive";
    default:
      return "unknown";
  }
}

const char * RecorderImpl::to_string(ResumeMode resume_mode)
{
  switch (resume_mode) {
    case ResumeMode::NodeTime:
      return "node";
    case ResumeMode::PublishTime:
      return "publish";
    case ResumeMode::ReceiveTime:
      return "receive";
    default:
      return "unknown";
  }
}

bool RecorderImpl::should_execute_immediately(const std::optional<rclcpp::Time> & target_time) const
{
  if (!target_time.has_value()) {
    return true;
  }
  return *target_time <= node->now();
}

void RecorderImpl::handle_timer_resume_request(
  std::optional<rclcpp::Time> resume_time,
  ResumeCallbackResponse & response)
{
  if (should_execute_immediately(resume_time)) {
    this->resume();
    set_service_success(response);
  } else {
    auto action_task = [this]() {
        this->resume();
      };
    action_task_runner_.schedule(resume_time.value(), std::move(action_task), "Resume recording");
    set_service_success(response);
  }
}

void RecorderImpl::handle_timestamp_resume_request(
  const std::optional<rclcpp::Time> & resume_time,
  ResumeMode resume_mode,
  const std::string & topic_name,
  ResumeCallbackResponse & response)
{
  if (!resume_time.has_value()) {
    this->resume();
    set_service_success(response);
    return;
  }

  if (resume_mode == ResumeMode::NodeTime) {  // Sanity check
    // Should not happen; handled separately
    RCLCPP_ERROR(node->get_logger(),
                 "Internal error: NodeTime resume mode should be handled separately.");
    set_service_error(response,
                      "Internal error: NodeTime resume mode should be handled separately.",
                      to_underlying_type(ResumeReturnCode::InvalidResumeMode));
    return;
  }

  const auto resume_req_ns = resume_time->nanoseconds();
  const bool use_receive_timestamp_for_resume = resume_mode == ResumeMode::ReceiveTime;
  const std::string on_topic_str = topic_name.empty() ? "" : " on '" + topic_name + "' topic";

  // Check if resume is already due
  // Get last seen timestamps for the requested topic (or global if no topic specified)
  auto [last_pub_timestamp, last_recv_timestamp] = last_seen_timestamps_.get(topic_name);
  if ((!use_receive_timestamp_for_resume && last_pub_timestamp >= resume_req_ns) ||
    (use_receive_timestamp_for_resume && last_recv_timestamp >= resume_req_ns))
  {
    // Resume is already due
    RCLCPP_INFO(node->get_logger(),
                "Timestamp-based resume request%s already due (req=%.9f s). Resuming now.",
                on_topic_str.c_str(), resume_time->seconds());
    this->resume();
    set_service_success(response);
    return;
  }

  {  // Schedule pending resume request
    std::lock_guard<std::mutex> lock(pending_resume_request_mutex_);
    if (pending_resume_request_) {
      RCLCPP_WARN(node->get_logger(),
                  "Overriding pending resume request (%.9f s) with newer request (%.9f s).",
                  RCUTILS_NS_TO_S(static_cast<double>(pending_resume_request_->time_ns)),
                  RCUTILS_NS_TO_S(static_cast<double>(resume_req_ns)));
    }
    pending_resume_request_ = std::make_unique<PendingResumeState>();
    pending_resume_request_->tracking_topic_name = topic_name;
    pending_resume_request_->mode = resume_mode;
    pending_resume_request_->time_ns = resume_req_ns;
  }

  RCLCPP_INFO(node->get_logger(),
              "Scheduled timestamp-based resume at %.9f seconds using %s timestamps%s.",
              resume_time->seconds(), to_string(resume_mode), on_topic_str.c_str());

  RCLCPP_DEBUG(node->get_logger(), "use_recv_for_resume = %s, last publish time: %ld ns, last "
               "receive time: %ld ns, requested resume time: %ld ns",
               use_receive_timestamp_for_resume ? "true" : "false",
               last_pub_timestamp, last_recv_timestamp, resume_req_ns);
  set_service_success(response);
}

void RecorderImpl::attempt_immediate_bag_split(SplitBagFileCallbackResponse & response)
{
  try {
    if (this->split_bagfile()) {
      set_service_success(response);
    } else {
      set_service_error(response,
                        "Called 'SplitBagfile' request while not in recording. Request ignored.",
                        to_underlying_type(SplitBagFileReturnCode::NotRecording));
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "Error during 'SplitBagfile' request: %s", e.what());
    set_service_error(response, e.what(), to_underlying_type(SplitBagFileReturnCode::SplitFailed));
  }
}

void RecorderImpl::handle_timer_bag_split_request(
  std::optional<rclcpp::Time> split_time,
  SplitBagFileCallbackResponse & response)
{
  if (should_execute_immediately(split_time)) {
    attempt_immediate_bag_split(response);
  } else {
    auto action_task = [this]() {
        try {
          (void)this->split_bagfile();
        } catch (const std::exception & e) {
          RCLCPP_ERROR(node->get_logger(), "Error during 'SplitBagfile' request: %s", e.what());
        }
      };
    action_task_runner_.schedule(split_time.value(), std::move(action_task), "Split bag file");
    set_service_success(response);
  }
}

void RecorderImpl::handle_timestamp_bag_split_request(
  const std::optional<rclcpp::Time> & split_time,
  SplitMode split_mode,
  const std::string & topic_name,
  SplitBagFileCallbackResponse & response)
{
  if (!split_time.has_value()) {
    attempt_immediate_bag_split(response);
    return;
  }

  if (split_mode == SplitMode::NodeTime) {  // Sanity check
    // Should not happen; handled separately
    RCLCPP_ERROR(node->get_logger(),
                 "Internal error: NodeTime split mode should be handled separately.");
    set_service_error(response,
                      "Internal error: NodeTime split mode should be handled separately.",
                      to_underlying_type(SplitBagFileReturnCode::InvalidSplitMode));
    return;
  }

  const auto split_req_ns = split_time->nanoseconds();
  const bool use_receive_timestamp_for_split = split_mode == SplitMode::ReceiveTime;
  const std::string on_topic_str = topic_name.empty() ? "" : " on '" + topic_name + "' topic";

  // Check if split is already due
  // Get last seen timestamps for the requested topic (or global if no topic specified)
  auto [last_pub_timestamp, last_recv_timestamp] = last_seen_timestamps_.get(topic_name);
  if ((!use_receive_timestamp_for_split && last_pub_timestamp >= split_req_ns) ||
    (use_receive_timestamp_for_split && last_recv_timestamp >= split_req_ns))
  {
    // Split is already due
    RCLCPP_INFO(node->get_logger(),
                "Timestamp-based split request%s already due (req=%.9f s). Splitting immediately.",
                on_topic_str.c_str(), split_time->seconds());
    attempt_immediate_bag_split(response);
    return;
  }

  {  // Schedule pending split request
    std::lock_guard<std::mutex> lock(pending_bag_split_request_mutex_);
    if (pending_bag_split_request_) {
      RCLCPP_WARN(node->get_logger(),
                  "Overriding pending split request (%.9f s) with newer request (%.9f s).",
                  RCUTILS_NS_TO_S(static_cast<double>(pending_bag_split_request_->time_ns)),
                  RCUTILS_NS_TO_S(static_cast<double>(split_req_ns)));
    }
    pending_bag_split_request_ = std::make_unique<PendingBagSplitState>();
    pending_bag_split_request_->tracking_topic_name = topic_name;
    pending_bag_split_request_->mode = split_mode;
    pending_bag_split_request_->time_ns = split_req_ns;
  }

  RCLCPP_INFO(node->get_logger(),
              "Scheduled timestamp-based split at %.9f seconds using %s timestamps%s.",
              split_time->seconds(), to_string(split_mode), on_topic_str.c_str());

  RCLCPP_DEBUG(node->get_logger(), "use_recv_for_split = %s, last publish time: %ld ns, last "
               "receive time: %ld ns , requested split time: %ld ns",
               use_receive_timestamp_for_split ? "true" : "false",
               last_pub_timestamp, last_recv_timestamp, split_req_ns);
  set_service_success(response);
}

void RecorderImpl::handle_pending_resume_request(
  const std::string & topic_name,
  const rcutils_time_point_value_t & publish_time,
  const rcutils_time_point_value_t & receive_time) noexcept
{
  std::lock_guard<std::mutex> pending_resume_state_lock(pending_resume_request_mutex_);
  // if we have a valid pending resume request, check if it applies to this message
  if (pending_resume_request_ && pending_resume_request_->time_ns != kNoPendingResumeRequest) {
    const bool matches_pending_topic =
      pending_resume_request_->tracking_topic_name.empty() ||
      pending_resume_request_->tracking_topic_name == topic_name;
    if (!matches_pending_topic) {
      return;
    }
    const bool use_pub_time = pending_resume_request_->mode == ResumeMode::PublishTime;
    const rcutils_time_point_value_t & message_time = use_pub_time ? publish_time : receive_time;

    if (message_time < 0) {
      RCLCPP_WARN_ONCE(node->get_logger(),
                       "Message timestamp is invalid; cannot evaluate resume request.");
      return;
    }

    if (message_time >= pending_resume_request_->time_ns) {
      RCLCPP_DEBUG(node->get_logger(),
                   "Performing %s-time resume at message time %ld ns (threshold %ld ns).",
                   to_string(pending_resume_request_->mode),
                   message_time,
                   pending_resume_request_->time_ns);
      this->resume();
      pending_resume_request_.reset();
    } else {
      // Not yet time to resume
      RCLCPP_DEBUG(node->get_logger(),
                   "Pending resume at %ld ns not yet reached (message time %ld ns).",
                   pending_resume_request_->time_ns, message_time);
    }
  }
}

void RecorderImpl::handle_pending_bag_split_request(
  const std::string & topic_name,
  const rcutils_time_point_value_t & publish_time,
  const rcutils_time_point_value_t & receive_time) noexcept
{
  std::lock_guard<std::mutex> pending_split_state_lock(pending_bag_split_request_mutex_);
  // if we have a valid pending split request, check if it applies to this message
  if (pending_bag_split_request_ && pending_bag_split_request_->time_ns != kNoPendingPublishSplit) {
    const bool matches_pending_topic =
      pending_bag_split_request_->tracking_topic_name.empty() ||
      pending_bag_split_request_->tracking_topic_name == topic_name;
    if (!matches_pending_topic) {
      return;
    }
    const bool use_pub_time = pending_bag_split_request_->mode == SplitMode::PublishTime;
    const rcutils_time_point_value_t & message_time = use_pub_time ? publish_time : receive_time;

    if (message_time < 0) {
      RCLCPP_WARN_ONCE(node->get_logger(),
                       "Message timestamp is invalid; cannot evaluate split request.");
      return;
    }

    if (message_time >= pending_bag_split_request_->time_ns) {
      RCLCPP_DEBUG(node->get_logger(),
                   "Performing %s-time split at message time %ld ns (threshold %ld ns).",
                   to_string(pending_bag_split_request_->mode),
                   message_time, pending_bag_split_request_->time_ns);
      auto action_task =
        [this]() {
          try {
            (void)this->split_bagfile();
          } catch (const std::exception & e) {
            RCLCPP_ERROR(node->get_logger(), "Error during bag file split request: %s", e.what());
          }
        };
      // Schedule split immediately via the task runner to avoid blocking further message processing
      action_task_runner_.schedule(node->now(), std::move(action_task), "Split bag file");
      // clear pending split
      pending_bag_split_request_.reset();
    } else {
      // Not yet time to split
      RCLCPP_DEBUG(node->get_logger(),
                   "Pending split at %ld ns not yet reached (message time %ld ns).",
                   pending_bag_split_request_->time_ns, message_time);
    }
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
    // Get graph event to ensure to start GrapListener if not already started and register event,
    // before we're starting discovery thread.
    // This is a workaround to split initialization and runtime phases to avoid race condition when
    // a new publisher appeared after we finish start_discovery() and/or Recorder::record(uri),
    // but before we really start topics_discovery() thread.
    discovery_graph_event_ = node->get_graph_event();
    discovery_future_ =
      std::async(std::launch::async, std::bind(&RecorderImpl::topics_discovery, this));
  }
}

void RecorderImpl::stop_discovery()
{
  std::lock_guard<std::mutex> state_lock(discovery_mutex_);
  if (discovery_running_.exchange(false)) {
    try {
      // Notify graph change to wake up discovery thread if it is sleeping on the graph event
      node->get_node_graph_interface()->notify_graph_change();
    } catch (const std::exception & e) {
      RCLCPP_WARN_STREAM(node->get_logger(),
        "Failed to notify graph change while stopping discovery: " << e.what());
    }
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

bool RecorderImpl::is_discovery_running() const
{
  return discovery_running_.load();
}

void RecorderImpl::topics_discovery() noexcept
{
  try {
    RCLCPP_INFO(node->get_logger(), "Topics discovery started.");
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
    bool should_update_subscriptions = true;
    while (rclcpp::ok() && discovery_running_) {
      if (!record_options_.topics.empty() &&
        subscriptions_.size() == record_options_.topics.size())
      {
        RCLCPP_INFO(
          node->get_logger(), "All requested topics are subscribed. Stopping discovery...");
        break;
      }

      if (should_update_subscriptions) {
        auto topics_to_subscribe = get_requested_or_available_topics();
        for (const auto & topic_and_type : topics_to_subscribe) {
          warn_if_new_qos_for_subscribed_topic(topic_and_type.first);
        }
        auto missing_topics = get_missing_topics(topics_to_subscribe);
        subscribe_topics(missing_topics);
      }
      node->wait_for_graph_change(discovery_graph_event_, record_options_.topic_polling_interval);
      should_update_subscriptions = discovery_graph_event_->check_and_clear();
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(node->get_logger(), "Failure in topics discovery.\nError: " << e.what());
  } catch (...) {
    RCLCPP_ERROR_STREAM(node->get_logger(), "Failure in topics discovery.");
  }
  discovery_running_ = false;
  RCLCPP_INFO(node->get_logger(), "Topics discovery stopped.");
}

std::unordered_map<std::string, std::string>
RecorderImpl::get_requested_or_available_topics()
{
  std::map<std::string, std::vector<std::string>> all_topics_and_types;
  // Take topic names and types from graph only if inclusive filters among static topics defined
  if (record_options_.all_topics || !record_options_.topics.empty() ||
    !record_options_.topic_types.empty() ||
    record_options_.all_services || !record_options_.services.empty() ||
    record_options_.all_actions || !record_options_.actions.empty() ||
    !record_options_.regex.empty())
  {
    all_topics_and_types = node->get_topic_names_and_types();
  }

  for (const auto & [static_topic_name, static_topic_type] : static_topics_) {
    if (all_topics_and_types.find(static_topic_name) == all_topics_and_types.cend()) {
      all_topics_and_types[static_topic_name] = {static_topic_type};
    }
  }
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
        record_options_.input_serialization_format,
        offered_qos_profiles_for_topic(endpoint_infos),
        type_description_hash_for_topic(endpoint_infos),
      });
  }
}

void RecorderImpl::add_channel(
  const std::string & topic_name,
  const std::string & topic_type,
  const std::string & serialization_format,
  const std::string & type_description_hash,
  const std::vector<rclcpp::QoS> & offered_qos_profiles)
{
  rosbag2_storage::TopicMetadata topic_with_type{
    0u,
    topic_name,
    topic_type,
    serialization_format,
    offered_qos_profiles,
    type_description_hash,
  };
  writer_->create_topic(topic_with_type);
}

void RecorderImpl::add_channel(
  const std::string & topic_name,
  const std::string & topic_type,
  const std::string & message_definition_encoding,
  const std::string & encoded_message_definition,
  const std::string & serialization_format,
  const std::string & type_description_hash,
  const std::vector<rclcpp::QoS> & offered_qos_profiles)
{
  rosbag2_storage::TopicMetadata topic_with_type{
    0u,
    topic_name,
    topic_type,
    serialization_format,
    offered_qos_profiles,
    type_description_hash,
  };
  rosbag2_storage::MessageDefinition message_definition{
    topic_type,
    message_definition_encoding,
    encoded_message_definition,
    type_description_hash
  };
  writer_->create_topic(topic_with_type, message_definition);
}

void RecorderImpl::write_message(
  std::shared_ptr<rcutils_uint8_array_t> serialized_data,
  const std::string & topic_name,
  const rcutils_time_point_value_t & pub_timestamp,
  uint32_t sequence_number)
{
  write_message(
    std::move(serialized_data),
    topic_name,
    pub_timestamp,
    node->now().nanoseconds(),
    sequence_number);
}

void RecorderImpl::write_message(
  std::shared_ptr<rcutils_uint8_array_t> serialized_data,
  const std::string & topic_name,
  const rcutils_time_point_value_t & pub_timestamp,
  const rcutils_time_point_value_t & recv_timestamp,
  uint32_t sequence_number)
{
  last_seen_timestamps_.update(topic_name, pub_timestamp, recv_timestamp);
  handle_pending_resume_request(topic_name, pub_timestamp, recv_timestamp);

  if (!paused_.load()) {
    auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
    bag_message->serialized_data = std::move(serialized_data);
    bag_message->topic_name = topic_name;
    bag_message->recv_timestamp = recv_timestamp;
    bag_message->send_timestamp = pub_timestamp;
    bag_message->sequence_number = sequence_number;
    writer_->write(bag_message);
    // Handle pending bag split request if it is existing
    handle_pending_bag_split_request(bag_message->topic_name,
                                     bag_message->send_timestamp,
                                     bag_message->recv_timestamp);
  }
}

void RecorderImpl::on_messages_lost_in_transport(
  const std::string & topic_name,
  const rclcpp::QOSMessageLostInfo & qos_msgs_lost_info)
{
  event_notifier_->on_messages_lost_in_transport(topic_name, qos_msgs_lost_info);
}

uint64_t RecorderImpl::get_total_num_messages_lost_in_transport() const
{
  return event_notifier_->get_total_num_messages_lost_in_transport();
}

void RecorderImpl::subscribe_topic(const rosbag2_storage::TopicMetadata & topic)
{
  if (subscriptions_.find(topic.name) != subscriptions_.end()) {
    return;
  }
  // Auto-detect transient-local topics when repeat_all_transient_local_depth is set
  if (record_options_.repeat_all_transient_local_depth > 0 &&
    record_options_.repeat_transient_local_messages.count(topic.name) == 0)
  {
    auto endpoint_infos = node->get_publishers_info_by_topic(topic.name);
    if (endpoint_infos.empty()) {
      RCLCPP_WARN_STREAM(node->get_logger(),
        "No publishers found for topic '" << topic.name <<
        "', cannot determine if it is transient-local. "
        "Use --repeat-transient-local to explicitly specify this topic.");
    } else {
      bool any_transient_local =
        std::any_of(
          endpoint_infos.begin(), endpoint_infos.end(),
        [](const rclcpp::TopicEndpointInfo & info) {
          return info.qos_profile().get_rmw_qos_profile().durability ==
                 RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
          }
        );
      if (any_transient_local) {
        record_options_.repeat_transient_local_messages[topic.name] =
          record_options_.repeat_all_transient_local_depth;
      }
    }
  }
  // Need to create topic in writer before we are trying to create subscription. Since in
  // callback for subscription we are calling writer_->write(bag_message); and it could happened
  // that callback called before we reached out the line: writer_->create_topic(topic)
  if (record_options_.repeat_transient_local_messages.count(topic.name) > 0) {
    writer_->create_transient_local_topic(
      topic, record_options_.repeat_transient_local_messages.at(topic.name));
  } else {
    writer_->create_topic(topic);
  }

  rosbag2_storage::Rosbag2QoS subscription_qos{subscription_qos_for_topic(topic.name)};

  auto subscription = create_subscription(topic.name, topic.type, subscription_qos);
  if (subscription) {
    subscriptions_.insert({topic.name, subscription});
    std::optional<size_t> repeat_tl_depth =
      record_options_.repeat_transient_local_messages.count(topic.name) > 0 ?
      std::make_optional(record_options_.repeat_transient_local_messages.at(topic.name)) :
      std::nullopt;
    if (node->get_logger().get_effective_level() == rclcpp::Logger::Level::Debug) {
      RCLCPP_DEBUG_STREAM(node->get_logger(),
        "Subscribed to topic '" << topic.name << "'" <<
        (repeat_tl_depth.has_value() ?
        " (repeat-transient-local, depth=" + std::to_string(repeat_tl_depth.value()) + ")" : "") <<
        " with QoS:\n" << subscription_qos.to_string());
    } else {
      RCLCPP_INFO_STREAM(node->get_logger(),
        "Subscribed to topic '" << topic.name << "'" <<
        (repeat_tl_depth.has_value() ?
        " (repeat-transient-local, depth=" + std::to_string(repeat_tl_depth.value()) + ")" : ""));
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
      on_messages_lost_in_transport(topic_name, msgs_lost_info);
    };

  auto subscription_callback =
    [this, topic_name, topic_type](std::shared_ptr<const rclcpp::SerializedMessage> message,
    const rclcpp::MessageInfo & mi)
    {
      rcutils_time_point_value_t recv_timestamp{0};
      rcutils_time_point_value_t send_timestamp{0};
#ifdef _WIN32
      if (std::string(rmw_get_implementation_identifier()).find("rmw_connextdds") !=
        std::string::npos)
      {
        recv_timestamp = node->now().nanoseconds();
        send_timestamp = 0;
      } else if (record_options_.use_sim_time) {
        recv_timestamp = node->now().nanoseconds();
        send_timestamp = mi.get_rmw_message_info().source_timestamp;
      } else {
        recv_timestamp = mi.get_rmw_message_info().received_timestamp;
        send_timestamp = mi.get_rmw_message_info().source_timestamp;
      }
#else
      if (record_options_.use_sim_time) {
        recv_timestamp = node->now().nanoseconds();
      } else {
        recv_timestamp = mi.get_rmw_message_info().received_timestamp;
      }
      send_timestamp = mi.get_rmw_message_info().source_timestamp;
#endif
      last_seen_timestamps_.update(topic_name, send_timestamp, recv_timestamp);
      handle_pending_resume_request(topic_name, send_timestamp, recv_timestamp);

      if (!paused_.load()) {
        writer_->write(std::move(message), topic_name, topic_type, recv_timestamp, send_timestamp);
        // Handle pending bag split request if it is existing
        handle_pending_bag_split_request(topic_name, send_timestamp, recv_timestamp);
      }
    };

  return node->create_generic_subscription(
    topic_name, topic_type, qos, subscription_callback, sub_options);
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
    if (record_options_.repeat_transient_local_messages.count(topic_name) > 0 &&
      topic_qos_profile_overrides_.at(topic_name).get_rmw_qos_profile().durability !=
      RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
    {
      RCLCPP_WARN_STREAM(
        node->get_logger(),
        "Topic '" << topic_name << "' has a QoS profile override without transient_local "
          "durability, but repeat-transient-local is enabled. The QoS override takes precedence; "
          "repeat-transient-local will not work unless the override includes transient_local "
          "durability.");
    }
    return topic_qos_profile_overrides_.at(topic_name);
  }

  auto qos = rosbag2_storage::Rosbag2QoS::adapt_request_to_offers(
    topic_name, node->get_publishers_info_by_topic(topic_name));

  if (record_options_.repeat_transient_local_messages.count(topic_name) > 0) {
    if (qos.get_rmw_qos_profile().durability != RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL) {
      RCLCPP_WARN_STREAM(
        node->get_logger(),
        "Overriding QoS durability to transient_local for topic '" << topic_name <<
          "' because repeat-transient-local is enabled.");
    }
    qos.transient_local();
  }

  return qos;
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

void RecorderImpl::read_static_topics() noexcept
{
  if (!record_options_.static_topics_uri.empty()) {
    try {
      RCLCPP_INFO_STREAM(node->get_logger(),
        "Reading static topics from " << record_options_.static_topics_uri);
      YAML::Node yaml_file = YAML::LoadFile(record_options_.static_topics_uri);
      auto list_nodes = yaml_file["static_topics_and_types_list"];
      if (!list_nodes) {
        throw std::runtime_error(
                "Static topics YAML file must have top-level key 'static_topics_and_types_list'");
      }
      if (!list_nodes.IsSequence()) {
        throw std::runtime_error(
                "Top-level key 'static_topics_and_types_list' must contain a list of "
                "'topic_name, topic_type' pairs");
      }
      for (const auto & bag_node : list_nodes) {
        const auto topic_name = bag_node[0].as<std::string>();
        const auto topic_type = bag_node[1].as<std::string>();
        if (topic_type.empty()) {
          RCLCPP_ERROR_STREAM(node->get_logger(),
            "Static topic " << topic_name << " has no corresponding type");
        }
        static_topics_.emplace_back(topic_name, topic_type);
      }
    } catch (std::exception & e) {
      RCLCPP_ERROR_STREAM(node->get_logger(), "Failed to read static topics list: " << e.what());
      // Print current static topics read so far
      RCLCPP_INFO_STREAM(node->get_logger(),
                         "Read " << static_topics_.size() << " static topics before failure");
      for (const auto & [topic_name, topic_type] : static_topics_) {
        RCLCPP_INFO_STREAM(node->get_logger(),
                           " \ttopic: " << topic_name << " \t\t type: " << topic_type);
      }
      // Clear any partially read topics
      static_topics_.clear();
      RCLCPP_INFO_STREAM(node->get_logger(), "Cleared static topics list due to read failure.");
      return;
    }
    RCLCPP_INFO_STREAM(node->get_logger(), "Read " << static_topics_.size() << " static topics");
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

void Recorder::record(const std::string & uri)
{
  (void)pimpl_->record(uri);
}

void Recorder::add_channel(
  const std::string & topic_name,
  const std::string & topic_type,
  const std::string & serialization_format,
  const std::string & type_description_hash,
  const std::vector<rclcpp::QoS> & offered_qos_profiles)
{
  pimpl_->add_channel(
    topic_name, topic_type, serialization_format, type_description_hash, offered_qos_profiles);
}

void Recorder::add_channel(
  const std::string & topic_name,
  const std::string & topic_type,
  const std::string & message_definition_encoding,
  const std::string & encoded_message_definition,
  const std::string & serialization_format,
  const std::string & type_description_hash,
  const std::vector<rclcpp::QoS> & offered_qos_profiles)
{
  pimpl_->add_channel(
    topic_name, topic_type, message_definition_encoding, encoded_message_definition,
    serialization_format, type_description_hash, offered_qos_profiles);
}

void Recorder::write_message(
  std::shared_ptr<rcutils_uint8_array_t> serialized_data,
  const std::string & topic_name,
  const rcutils_time_point_value_t & pub_timestamp,
  uint32_t sequence_number)
{
  pimpl_->write_message(
    std::move(serialized_data), topic_name, pub_timestamp, sequence_number);
}

void Recorder::write_message(
  std::shared_ptr<rcutils_uint8_array_t> serialized_data,
  const std::string & topic_name,
  const rcutils_time_point_value_t & pub_timestamp,
  const rcutils_time_point_value_t & recv_timestamp,
  uint32_t sequence_number)
{
  pimpl_->write_message(
    std::move(serialized_data), topic_name, pub_timestamp, recv_timestamp, sequence_number);
}

bool Recorder::split_bagfile()
{
  return pimpl_->split_bagfile();
}

void Recorder::on_messages_lost_in_transport(
  const std::string & topic_name, const rclcpp::QOSMessageLostInfo & qos_msgs_lost_info)
{
  pimpl_->on_messages_lost_in_transport(topic_name, qos_msgs_lost_info);
}

uint64_t Recorder::get_total_num_messages_lost_in_transport() const
{
  return pimpl_->get_total_num_messages_lost_in_transport();
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

bool
Recorder::is_discovery_running() const
{
  return pimpl_->is_discovery_running();
}

void Recorder::set_on_start_recording_callback(OnStartRecordingCallback callback) const
{
  pimpl_->on_start_recording_callback_ = std::move(callback);
}

void Recorder::read_static_topics() noexcept
{
  return pimpl_->read_static_topics();
}

const std::vector<std::pair<std::string, std::string>> & Recorder::get_static_topics() noexcept
{
  return pimpl_->static_topics_;
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
