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

#include "rclcpp/node.hpp"
#include "rclcpp/qos.hpp"

#include "rosbag2_cpp/writer.hpp"

#include "rosbag2_interfaces/srv/is_discovery_running.hpp"
#include "rosbag2_interfaces/srv/is_paused.hpp"
#include "rosbag2_interfaces/srv/pause.hpp"
#include "rosbag2_interfaces/srv/record.hpp"
#include "rosbag2_interfaces/srv/resume.hpp"
#include "rosbag2_interfaces/srv/snapshot.hpp"
#include "rosbag2_interfaces/srv/split_bagfile.hpp"
#include "rosbag2_interfaces/srv/start_discovery.hpp"
#include "rosbag2_interfaces/srv/stop_discovery.hpp"
#include "rosbag2_interfaces/srv/stop.hpp"

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

class RecorderImpl;

class Recorder : public rclcpp::Node
{
public:
  /// \brief Constructor and entry point for the composable recorder.
  /// Will call Recorder(node_name, node_options) constructor with node_name = "rosbag2_recorder".
  /// \param node_options Node options which will be used during construction of the underlying
  /// node.
  ROSBAG2_TRANSPORT_PUBLIC
  explicit Recorder(const rclcpp::NodeOptions & node_options);

  /// \brief Default constructor and entry point for the composable recorder.
  /// Will construct Recorder class and initialize record_options, storage_options from node
  /// parameters. At the end will call Recorder::record() to automatically start recording in a
  /// separate thread.
  /// \param node_name Name for the underlying node.
  /// \param node_options Node options which will be used during construction of the underlying
  /// node.
  ROSBAG2_TRANSPORT_PUBLIC
  explicit Recorder(
    const std::string & node_name = "rosbag2_recorder",
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());

  /// \brief Constructor which will construct Recorder class with provided parameters and default
  /// KeyboardHandler class initialized with parameter which is disabling signal handlers in it.
  /// \param writer Shared pointer to the instance of the rosbag2_cpp::Writer class. Shall not be
  /// null_ptr.
  /// \param storage_options Storage options which will be applied to the rosbag2_cpp::writer class
  /// when recording will be started.
  /// \param record_options Settings for Recorder class
  /// \param node_name Name for the underlying node.
  /// \param node_options Node options which will be used during construction of the underlying
  /// node.
  ROSBAG2_TRANSPORT_PUBLIC
  Recorder(
    std::shared_ptr<rosbag2_cpp::Writer> writer,
    const rosbag2_storage::StorageOptions & storage_options,
    const rosbag2_transport::RecordOptions & record_options,
    const std::string & node_name = "rosbag2_recorder",
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());

  /// \brief Constructor which will construct Recorder class with provided parameters
  /// \param writer Shared pointer to the instance of the rosbag2_cpp::Writer class. Shall not be
  /// null_ptr.
  /// \param keyboard_handler Keyboard handler class uses to handle user input from keyboard.
  /// \param storage_options Storage options which will be applied to the rosbag2_cpp::writer class
  /// when recording will be started.
  /// \param record_options Settings for Recorder class
  /// \param node_name Name for the underlying node.
  /// \param node_options Node options which will be used during construction of the underlying
  /// node.
  ROSBAG2_TRANSPORT_PUBLIC
  Recorder(
    std::shared_ptr<rosbag2_cpp::Writer> writer,
    std::shared_ptr<KeyboardHandler> keyboard_handler,
    const rosbag2_storage::StorageOptions & storage_options,
    const rosbag2_transport::RecordOptions & record_options,
    const std::string & node_name = "rosbag2_recorder",
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());

  /// \brief Default destructor.
  ROSBAG2_TRANSPORT_PUBLIC
  virtual ~Recorder();

  /// \brief Start recording.
  /// \details The record() method will return almost immediately and recording will happen in
  /// background.
  /// \param uri If provided, it will override the storage_options.uri provided during construction.
  ROSBAG2_TRANSPORT_PUBLIC
  void record(const std::string & uri = "");

  /// @brief Add a new channel (topic) to the rosbag2 writer to be recorded.
  /// \details This is a direct Recorder API equivalent to the rosbag2_cpp::Writer::add_topic().
  /// \note This method does not require the message definition. The recorder will try to find the
  /// corresponding message definition by the given topic name and type.
  /// @param topic_name The name of the topic.
  /// @param topic_type The type of the topic.
  /// @param serialization_format The serialization format of the topic.
  /// @param type_description_hash REP-2011 type description hash of the topic.
  /// @param offered_qos_profiles The list of offered QoS profiles for the topic if available.
  ROSBAG2_TRANSPORT_PUBLIC
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
  /// \param offered_qos_profiles The list of offered QoS profiles for the topic if available.
  ROSBAG2_TRANSPORT_PUBLIC
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
  /// @param serialized_data The serialized message data to write.
  /// @param topic_name The name of the topic the message belongs to.
  /// @param pub_timestamp The original or publication timestamp of the message.
  /// @param sequence_number An optional sequence number of the message. If non-zero, sequence
  /// numbers should be unique per channel and increasing over time.
  /// \throws std::runtime_error if topic has not been added via add_channel().
  ROSBAG2_TRANSPORT_PUBLIC
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
  ROSBAG2_TRANSPORT_PUBLIC
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
  ROSBAG2_TRANSPORT_PUBLIC
  void on_messages_lost_in_transport(
    const std::string & topic_name,
    const rclcpp::QOSMessageLostInfo & qos_msgs_lost_info);

  /// @brief Get total number of messages lost in transport layer.
  /// @return Total number of messages lost in transport layer.
  [[nodiscard]]
  ROSBAG2_TRANSPORT_PUBLIC
  uint64_t get_total_num_messages_lost_in_transport() const;

  /// @brief Stopping recording.
  /// @details The stop() is opposite to the record(uri) operation. It will stop recording, dump
  /// all buffers to the disk and close writer. The record(uri) can be called again after stop().
  ROSBAG2_TRANSPORT_PUBLIC
  void stop();

  ROSBAG2_TRANSPORT_PUBLIC
  const std::unordered_set<std::string> &
  topics_using_fallback_qos() const;

  ROSBAG2_TRANSPORT_PUBLIC
  const std::unordered_map<std::string, std::shared_ptr<rclcpp::SubscriptionBase>> &
  subscriptions() const;

  ROSBAG2_TRANSPORT_PUBLIC
  const rosbag2_cpp::Writer & get_writer_handle();

  /// @brief Pause the recording.
  /// @details Will keep writer open and skip messages upon arrival on subscriptions.
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

  /// \brief Start discovery
  ROSBAG2_TRANSPORT_PUBLIC
  void start_discovery();

  /// \brief Stop discovery
  ROSBAG2_TRANSPORT_PUBLIC
  void stop_discovery();

  /// Return the current discovery state.
  ROSBAG2_TRANSPORT_PUBLIC
  bool is_discovery_running() const;

  inline constexpr static const auto kPauseResumeToggleKey = KeyboardHandler::KeyCode::SPACE;

protected:
  ROSBAG2_TRANSPORT_PUBLIC
  std::unordered_map<std::string, std::string> get_requested_or_available_topics();

  ROSBAG2_TRANSPORT_PUBLIC
  rosbag2_cpp::Writer & get_writer();

  ROSBAG2_TRANSPORT_PUBLIC
  rosbag2_storage::StorageOptions & get_storage_options();

  ROSBAG2_TRANSPORT_PUBLIC
  rosbag2_transport::RecordOptions & get_record_options();

private:
  std::unique_ptr<RecorderImpl> pimpl_;
};

ROSBAG2_TRANSPORT_PUBLIC std::string type_hash_to_string(const rosidl_type_hash_t & type_hash);
// Retrieve the type description hash from endpoint info.
ROSBAG2_TRANSPORT_PUBLIC std::string type_description_hash_for_topic(
  const std::vector<rclcpp::TopicEndpointInfo> & topics_endpoint_info);

}  // namespace rosbag2_transport

#endif  // ROSBAG2_TRANSPORT__RECORDER_HPP_
