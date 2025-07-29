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
#include <forward_list>
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "keyboard_handler/keyboard_handler.hpp"

#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/qos.hpp"

#include "rosbag2_cpp/clocks/player_clock.hpp"
#include "rosbag2_interfaces/msg/read_split_event.hpp"
#include "rosbag2_interfaces/srv/get_rate.hpp"
#include "rosbag2_interfaces/srv/is_paused.hpp"
#include "rosbag2_interfaces/srv/pause.hpp"
#include "rosbag2_interfaces/srv/play.hpp"
#include "rosbag2_interfaces/srv/play_next.hpp"
#include "rosbag2_interfaces/srv/burst.hpp"
#include "rosbag2_interfaces/srv/resume.hpp"
#include "rosbag2_interfaces/srv/set_rate.hpp"
#include "rosbag2_interfaces/srv/seek.hpp"
#include "rosbag2_interfaces/srv/stop.hpp"
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

class PlayerImpl;

class Player : public rclcpp::Node
{
public:
  /// \brief Type for callback functions.
  using play_msg_callback_t =
    std::function<void (std::shared_ptr<rosbag2_storage::SerializedBagMessage>)>;

  /// \brief Callback handle returning from add_on_play_message_pre_callback and
  /// add_on_play_message_post_callback and used as an argument for
  /// delete_on_play_message_callback.
  using callback_handle_t = uint64_t;

  /// \brief Utility type for a pair of rosbag2_cpp::Reader and rosbag2_storage::StorageOptions.
  using reader_storage_options_pair_t = \
    std::pair<std::unique_ptr<rosbag2_cpp::Reader>, rosbag2_storage::StorageOptions>;

  /// \brief Const describing invalid value for callback_handle.
  ROSBAG2_TRANSPORT_PUBLIC
  static constexpr callback_handle_t invalid_callback_handle = 0;

  /// \brief Constructor and entry point for the composable player.
  /// Will call Player(node_name, node_options) constructor with node_name = "rosbag2_player".
  /// \param node_options Node options which will be used during construction of the underlying
  /// node.
  ROSBAG2_TRANSPORT_PUBLIC
  explicit Player(const rclcpp::NodeOptions & node_options);

  /// \brief Default constructor and entry point for the composable player.
  /// Will construct Player class and initialize play_options, storage_options from node
  /// parameters. At the end will call Player::play() to automatically start playback in a
  /// separate thread.
  /// \param node_name Name for the underlying node.
  /// \param node_options Node options which will be used during construction of the underlying
  /// node.
  ROSBAG2_TRANSPORT_PUBLIC
  explicit Player(
    const std::string & node_name = "rosbag2_player",
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());

  /// \brief Constructor which will construct Player class with provided parameters and default
  /// rosbag2_cpp::reader and KeyboardHandler classes.
  /// \note The KeyboardHandler class will be initialized with parameter which is disabling
  /// signal handlers in it.
  /// \param storage_options Storage options which will be applied to the rosbag2_cpp::reader
  /// after construction.
  /// \param play_options Playback settings for Player class.
  /// \param node_name Name for the underlying node.
  /// \param node_options Node options which will be used during construction of the underlying
  /// node.
  ROSBAG2_TRANSPORT_PUBLIC
  Player(
    const rosbag2_storage::StorageOptions & storage_options,
    const rosbag2_transport::PlayOptions & play_options,
    const std::string & node_name = "rosbag2_player",
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());

  /// \brief Constructor which will construct Player class with provided parameters and default
  /// KeyboardHandler class initialized with parameter which is disabling signal handlers in it.
  /// \param reader Unique pointer to the rosbag2_cpp::Reader class which will be moved to the
  /// internal instance of the Player class during construction.
  /// \param storage_options Storage options which will be applied to the rosbag2_cpp::reader
  /// after construction.
  /// \param play_options Playback settings for Player class.
  /// \param node_name Name for the underlying node.
  /// \param node_options Node options which will be used during construction of the underlying
  /// node.
  ROSBAG2_TRANSPORT_PUBLIC
  Player(
    std::unique_ptr<rosbag2_cpp::Reader> reader,
    const rosbag2_storage::StorageOptions & storage_options,
    const rosbag2_transport::PlayOptions & play_options,
    const std::string & node_name = "rosbag2_player",
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());

  /// \brief Constructor which will construct Player class with provided parameters.
  /// \param keyboard_handler Keyboard handler class uses to handle user input from keyboard.
  /// \param reader Unique pointer to the rosbag2_cpp::Reader class which will be moved to the
  /// internal instance of the Player class during construction.
  /// \param storage_options Storage options which will be applied to the rosbag2_cpp::reader
  /// after construction.
  /// \param play_options Playback settings for Player class.
  /// \param node_name Name for the underlying node.
  /// \param node_options Node options which will be used during construction of the underlying
  /// node.
  ROSBAG2_TRANSPORT_PUBLIC
  Player(
    std::unique_ptr<rosbag2_cpp::Reader> reader,
    std::shared_ptr<KeyboardHandler> keyboard_handler,
    const rosbag2_storage::StorageOptions & storage_options,
    const rosbag2_transport::PlayOptions & play_options,
    const std::string & node_name = "rosbag2_player",
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());

  /// \brief Constructor which will construct Player class with multiple readers and provided
  /// storage options for each reader.
  /// \note The KeyboardHandler class will be initialized with parameter which is disabling
  /// signal handlers in it.
  /// \param storage_options Storage options which will each be applied to a rosbag2_cpp::reader
  /// after construction.
  /// \param play_options Playback settings for Player class.
  /// \param node_name Name for the underlying node.
  /// \param node_options Node options which will be used during construction of the underlying
  /// node.
  ROSBAG2_TRANSPORT_PUBLIC
  Player(
    const std::vector<rosbag2_storage::StorageOptions> & storage_options,
    const rosbag2_transport::PlayOptions & play_options,
    const std::string & node_name = "rosbag2_player",
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());

  /// \brief Constructor which will construct Player class with provided parameters.
  /// \param readers_with_options Vector of pairs of unique pointer to the rosbag2_cpp::Reader class
  /// (which will be moved to the internal instance of the Player class during construction) and
  /// storage options (which will be applied to the rosbag2_cpp::reader when opening it).
  /// \param play_options Playback settings for Player class.
  /// \param node_name Name for the underlying node.
  /// \param node_options Node options which will be used during construction of the underlying
  /// node.
  ROSBAG2_TRANSPORT_PUBLIC
  Player(
    std::vector<reader_storage_options_pair_t> && readers_with_options,
    const rosbag2_transport::PlayOptions & play_options,
    const std::string & node_name = "rosbag2_player",
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());

  /// \brief Constructor which will construct Player class with provided parameters.
  /// \param readers_with_options Vector of pairs of unique pointer to the rosbag2_cpp::Reader class
  /// (which will be moved to the internal instance of the Player class during construction) and
  /// storage options (which will be applied to the rosbag2_cpp::reader when opening it).
  /// \param keyboard_handler Keyboard handler class uses to handle user input from keyboard.
  /// \param play_options Playback settings for Player class.
  /// \param node_name Name for the underlying node.
  /// \param node_options Node options which will be used during construction of the underlying
  /// node.
  ROSBAG2_TRANSPORT_PUBLIC
  Player(
    std::vector<reader_storage_options_pair_t> && readers_with_options,
    std::shared_ptr<KeyboardHandler> keyboard_handler,
    const rosbag2_transport::PlayOptions & play_options,
    const std::string & node_name = "rosbag2_player",
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());

  /// \brief Default destructor.
  ROSBAG2_TRANSPORT_PUBLIC
  virtual ~Player();

  /// \brief Start playback asynchronously in a separate thread
  /// \return false if playback thread already running, otherwise true
  ROSBAG2_TRANSPORT_PUBLIC
  bool play();

  /// \brief Waits on the condition variable until the play thread starts and the message's queue
  /// will be filled.
  /// @param timeout Maximum time in the fraction of seconds to wait for player to start.
  /// If timeout is negative, the wait_for_playback_to_start will be a blocking call.
  /// @return true if playback successfully started during timeout, otherwise false.
  ROSBAG2_TRANSPORT_PUBLIC
  bool wait_for_playback_to_start(std::chrono::duration<double> timeout = std::chrono::seconds(-1));

  /// \brief Waits on the condition variable until the play thread finishes.
  /// @param timeout Maximum time in the fraction of seconds to wait for player to finish.
  /// If timeout is negative, the wait_for_playback_to_finish will be a blocking call.
  /// @return true if playback finished during timeout, otherwise false.
  ROSBAG2_TRANSPORT_PUBLIC
  bool wait_for_playback_to_finish(
    std::chrono::duration<double> timeout = std::chrono::seconds(-1));

  /// \brief Unpause if in pause mode, stop playback and exit from play.
  ROSBAG2_TRANSPORT_PUBLIC
  void stop();

  // Playback control interface
  /// Pause the flow of time for playback.
  ROSBAG2_TRANSPORT_PUBLIC
  virtual void pause();

  /// Start the flow of time for playback.
  ROSBAG2_TRANSPORT_PUBLIC
  virtual void resume();

  /// Pause if time running, resume if paused.
  ROSBAG2_TRANSPORT_PUBLIC
  void toggle_paused();

  /// Return whether the playback is currently paused.
  ROSBAG2_TRANSPORT_PUBLIC
  bool is_paused() const;

  /// \brief Getter method for starting time of the playback.
  /// \return Returns timestamp of the first message in nanoseconds.
  ROSBAG2_TRANSPORT_PUBLIC
  rcutils_time_point_value_t get_starting_time() const;

  /// \brief Getter method for playback duration
  /// \return Returns duration of the playback in nanoseconds.
  ROSBAG2_TRANSPORT_PUBLIC
  rcutils_duration_value_t get_playback_duration() const;

  /// Return current playback rate.
  ROSBAG2_TRANSPORT_PUBLIC
  double get_rate() const;

  /// \brief Set the playback rate.
  /// \return false if an invalid value was provided (<= 0).
  ROSBAG2_TRANSPORT_PUBLIC
  virtual bool set_rate(double);

  /// \brief Playing next message from queue when in pause.
  /// \details This is blocking call and it will wait until next available message will be
  /// published or rclcpp context shut down.
  /// \note If internal player queue is starving and storage has not been completely loaded,
  /// this method will wait until new element will be pushed to the queue.
  /// \return true if player in pause mode and successfully played next message, otherwise false.
  ROSBAG2_TRANSPORT_PUBLIC
  virtual bool play_next();

  /// \brief Burst the next \p num_messages messages from the queue when paused.
  /// \param num_messages The number of messages to burst from the queue. Specifying zero means no
  /// limit (i.e. burst the entire bag).
  /// \details This call will play the next \p num_messages from the queue in burst mode. The
  /// timing of the messages is ignored.
  /// \note If internal player queue is starving and storage has not been completely loaded,
  /// this method will wait until new element will be pushed to the queue.
  /// \return The number of messages that was played.
  ROSBAG2_TRANSPORT_PUBLIC
  virtual size_t burst(const size_t num_messages);

  /// \brief Advance player to the message with closest timestamp >= time_point.
  /// \details This is blocking call and it will wait until current message will be published
  /// and message queue will be refilled.
  /// If time_point is before the beginning of the bag, then playback time will be set to the
  /// beginning of the bag.
  /// If time_point is after the end of the bag, playback time will be set to the end of the bag,
  /// which will then end playback, or if loop is enabled then will start playing at the beginning
  /// of the next loop.
  /// \param time_point Time point in ROS playback timeline.
  ROSBAG2_TRANSPORT_PUBLIC
  void seek(rcutils_time_point_value_t time_point);

  /// \brief Adding callable object as handler for pre-callback on play message.
  /// \param callback Callable which will be called before next message will be published.
  /// \note In case of registering multiple callbacks later-registered callbacks will be called
  /// first.
  /// \return Returns newly created callback handle if callback was successfully added,
  /// otherwise returns invalid_callback_handle.
  ROSBAG2_TRANSPORT_PUBLIC
  callback_handle_t add_on_play_message_pre_callback(const play_msg_callback_t & callback);

  /// \brief Adding callable object as handler for post-callback on play message.
  /// \param callback Callable which will be called after next message will be published.
  /// \note In case of registering multiple callbacks later-registered callbacks will be called
  /// first.
  /// \return Returns newly created callback handle if callback was successfully added,
  /// otherwise returns invalid_callback_handle.
  ROSBAG2_TRANSPORT_PUBLIC
  callback_handle_t add_on_play_message_post_callback(const play_msg_callback_t & callback);

  /// \brief Delete pre or post on play message callback from internal player lists.
  /// \param handle Callback's handle returned from #add_on_play_message_pre_callback or
  /// #add_on_play_message_post_callback
  ROSBAG2_TRANSPORT_PUBLIC
  void delete_on_play_message_callback(const callback_handle_t & handle);

  /// Wait until sent service requests will receive responses from service servers.
  /// \note The player node shall be spun in the executor in a parallel thread to be able to wait
  /// for responses.
  /// \param service_name - Name of the service or service event from what to wait responses.
  /// \note is service_name is empty the function will wait until all service requests sent to all
  /// service servers will finish. Timeout in this cases will be used for each service name.
  /// \param timeout - Timeout in fraction of seconds to wait for.
  /// \return true if service requests successfully finished, otherwise false.
  ROSBAG2_TRANSPORT_PUBLIC
  bool wait_for_sent_service_requests_to_finish(
    const std::string & service_name,
    std::chrono::duration<double> timeout = std::chrono::seconds(5));

protected:
  /// \brief Getter for publishers corresponding to each topic
  /// \return Hashtable representing topic to publisher map excluding inner clock_publisher
  ROSBAG2_TRANSPORT_PUBLIC
  std::unordered_map<std::string, std::shared_ptr<rclcpp::GenericPublisher>> get_publishers();

  /// \brief Getter for clients corresponding to each service name
  /// \return Hashtable representing service name to client
  ROSBAG2_TRANSPORT_PUBLIC
  std::unordered_map<std::string, std::shared_ptr<rclcpp::GenericClient>> get_service_clients();

  /// \brief Getter for inner clock_publisher
  /// \return Shared pointer to the inner clock_publisher
  ROSBAG2_TRANSPORT_PUBLIC
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr get_clock_publisher();

  /// \brief Getter for the number of registered on_play_msg_pre_callbacks
  /// \return Number of registered on_play_msg_pre_callbacks
  ROSBAG2_TRANSPORT_PUBLIC
  size_t get_number_of_registered_on_play_msg_pre_callbacks();

  /// \brief Getter for the number of registered on_play_msg_post_callbacks
  /// \return Number of registered on_play_msg_post_callbacks
  ROSBAG2_TRANSPORT_PUBLIC
  size_t get_number_of_registered_on_play_msg_post_callbacks();

  /// \brief Getter for the first of the currently stored storage options
  /// \return Reference to the first item in the StorageOptions vector
  ROSBAG2_TRANSPORT_PUBLIC
  const rosbag2_storage::StorageOptions & get_storage_options();

  /// \brief Getter for the currently stored storage options
  /// \return Copy of the currently stored storage options
  ROSBAG2_TRANSPORT_PUBLIC
  std::vector<rosbag2_storage::StorageOptions> get_all_storage_options();

  /// \brief Getter for the currently stored play options
  /// \return Copy of the currently stored play options
  ROSBAG2_TRANSPORT_PUBLIC
  const rosbag2_transport::PlayOptions & get_play_options();

private:
  std::unique_ptr<PlayerImpl> pimpl_;
};

}  // namespace rosbag2_transport

#endif  // ROSBAG2_TRANSPORT__PLAYER_HPP_
