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
  /**
   * rclcpp::Node constructor.
   * Not yet implemented.
   */
  ROSBAG2_TRANSPORT_PUBLIC
  explicit Player(
    const std::string & node_name = "rosbag2_player",
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());

  /**
   * Constructor.
   *
   * Prepares playback publishers and sets up the internal clock.
   * Does not start reading messages from the reader.
   *
   * \param storage_options: Configuration for the storage
   * \param play_options: Configuration for the playback behavior
   * \param node_name: Name of the underlying Node
   * \param node_options: Configuration opitons for the underlying Node.
   */
  ROSBAG2_TRANSPORT_PUBLIC
  Player(
    std::unique_ptr<rosbag2_cpp::Reader> reader,
    const rosbag2_storage::StorageOptions & storage_options,
    const rosbag2_transport::PlayOptions & play_options,
    const std::string & node_name = "rosbag2_player",
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());

  ROSBAG2_TRANSPORT_PUBLIC
  virtual ~Player();

  /**
   * Play the bag from beginning to end, blocking.
   * \return void: when playback is complete, or has been interrupted by rclcpp shutdown.
   */
  ROSBAG2_TRANSPORT_PUBLIC
  void play();

  /**
   * Give back ownership of the reader that was acquired at construction.
   * The player may not play again after this release, it is invalid and must be destroyed.
   */
  ROSBAG2_TRANSPORT_PUBLIC
  rosbag2_cpp::Reader * release_reader();

private:
  /**
   * Load new messages constantly into the play queue - when there is room.
   * Intended to be run in a background thread.
   * Returns when it reaches the end of the underlying storage.
   */
  void load_storage_content();
  /**
   * Return true if load_storage_content has finished,
   * meaning all messages have been read from storage and enqueued.
   */
  bool is_storage_completely_loaded() const;
  /**
   * Read messages into the queue until the message count is reached in the queue.
   * \param boundary: number of messages that should be in the full queue
   */
  void enqueue_up_to_boundary(uint64_t boundary);
  /// Block until the message queue is full.
  void wait_for_filled_queue() const;
  /// Play messages until the end of the storage is reached.
  void play_messages_from_queue();
  /**
   * Play messages until the current queue is empty.
   * If this returns before the end of playback, it means the queue is not filling fast enough.
   */
  void play_messages_until_queue_empty();
  /**
   * Set up all the publishers for the playback.
   * Precondition: reader_ must be open.
   */
  void prepare_publishers();
  /**
   * Construct the internal clock and reset starting time to match the bag.
   * Call before restarting playback.
   * Precondition: reader_ must be open.
   */
  void prepare_clock();
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
};

}  // namespace rosbag2_transport

#endif  // ROSBAG2_TRANSPORT__PLAYER_HPP_
