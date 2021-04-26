// Copyright 2018, Bosch Software Innovations GmbH.
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

#ifndef ROSBAG2_TRANSPORT__PLAYER_HPP_
#define ROSBAG2_TRANSPORT__PLAYER_HPP_

#include <chrono>
#include <future>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>

#include "moodycamel/readerwriterqueue.h"

#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/qos.hpp"

#include "rosbag2_cpp/clocks/player_clock.hpp"
#include "rosbag2_interfaces/srv/get_rate.hpp"
#include "rosbag2_interfaces/srv/is_paused.hpp"
#include "rosbag2_interfaces/srv/pause.hpp"
#include "rosbag2_interfaces/srv/resume.hpp"
#include "rosbag2_interfaces/srv/set_rate.hpp"
#include "rosbag2_interfaces/srv/toggle_paused.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"
#include "rosbag2_storage/storage_options.hpp"

#include "rosbag2_transport/play_options.hpp"
#include "rosbag2_transport/visibility_control.hpp"

#include "rosgraph_msgs/msg/clock.hpp"

namespace rosbag2_cpp
{
class Reader;
}  // namespace rosbag2_cpp

namespace rosbag2_transport
{

class Player : public rclcpp::Node
{
public:
  ROSBAG2_TRANSPORT_PUBLIC
  explicit Player(
    const std::string & node_name = "rosbag2_player",
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());

  ROSBAG2_TRANSPORT_PUBLIC
  Player(
    const rosbag2_storage::StorageOptions & storage_options,
    const rosbag2_transport::PlayOptions & play_options,
    const std::string & node_name = "rosbag2_player",
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());

  ROSBAG2_TRANSPORT_PUBLIC
  Player(
    std::unique_ptr<rosbag2_cpp::Reader> reader,
    const rosbag2_storage::StorageOptions & storage_options,
    const rosbag2_transport::PlayOptions & play_options,
    const std::string & node_name = "rosbag2_player",
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());

  ROSBAG2_TRANSPORT_PUBLIC
  virtual ~Player();

  ROSBAG2_TRANSPORT_PUBLIC
  void play();

  ROSBAG2_TRANSPORT_PUBLIC
  rosbag2_cpp::Reader * release_reader();

  // Playback control interface
  /// Pause the flow of time for playback.
  ROSBAG2_TRANSPORT_PUBLIC
  void pause();

  /// Start the flow of time for playback.
  ROSBAG2_TRANSPORT_PUBLIC
  void resume();

  /// Pause if time running, resume if paused.
  ROSBAG2_TRANSPORT_PUBLIC
  void toggle_paused();

  /// Return whether the playback is currently paused.
  ROSBAG2_TRANSPORT_PUBLIC
  bool is_paused() const;

  /// Return current playback rate.
  ROSBAG2_TRANSPORT_PUBLIC
  double get_rate() const;

  /// Set the playback rate.
  /**
   * Set the playback rate.
   * \return false if an invalid value was provided (<= 0).
   */
  ROSBAG2_TRANSPORT_PUBLIC
  bool set_rate(double);

private:
  void load_storage_content();
  bool is_storage_completely_loaded() const;
  void enqueue_up_to_boundary(uint64_t boundary);
  void wait_for_filled_queue() const;
  void play_messages_from_queue();
  void play_messages_until_queue_empty();
  void prepare_publishers();
  void prepare_clock(rcutils_time_point_value_t starting_time);
  static constexpr double read_ahead_lower_bound_percentage_ = 0.9;
  static const std::chrono::milliseconds queue_read_wait_period_;

  std::unique_ptr<rosbag2_cpp::Reader> reader_;
  rosbag2_storage::StorageOptions storage_options_;
  rosbag2_transport::PlayOptions play_options_;
  moodycamel::ReaderWriterQueue<rosbag2_storage::SerializedBagMessageSharedPtr> message_queue_;
  mutable std::future<void> storage_loading_future_;
  std::unordered_map<std::string, std::shared_ptr<rclcpp::GenericPublisher>> publishers_;
  std::unordered_map<std::string, rclcpp::QoS> topic_qos_profile_overrides_;
  std::unique_ptr<rosbag2_cpp::PlayerClock> clock_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher_;
  std::shared_ptr<rclcpp::TimerBase> clock_publish_timer_;

  rclcpp::Service<rosbag2_interfaces::srv::Pause>::SharedPtr srv_pause_;
  rclcpp::Service<rosbag2_interfaces::srv::Resume>::SharedPtr srv_resume_;
  rclcpp::Service<rosbag2_interfaces::srv::TogglePaused>::SharedPtr srv_toggle_paused_;
  rclcpp::Service<rosbag2_interfaces::srv::IsPaused>::SharedPtr srv_is_paused_;
  rclcpp::Service<rosbag2_interfaces::srv::GetRate>::SharedPtr srv_get_rate_;
  rclcpp::Service<rosbag2_interfaces::srv::SetRate>::SharedPtr srv_set_rate_;
};

}  // namespace rosbag2_transport

#endif  // ROSBAG2_TRANSPORT__PLAYER_HPP_
