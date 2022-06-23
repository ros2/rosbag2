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
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "keyboard_handler/keyboard_handler.hpp"

#include "moodycamel/readerwriterqueue.h"

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
  /// \brief Type for callback functions.
  using play_msg_callback_t =
    std::function<void (std::shared_ptr<rosbag2_storage::SerializedBagMessage>)>;

  /// \brief Callback handle returning from add_on_play_message_pre_callback and
  /// add_on_play_message_post_callback and used as an argument for
  /// delete_on_play_message_callback.
  using callback_handle_t = uint64_t;

  /// \brief Const describing invalid value for callback_handle.
  ROSBAG2_TRANSPORT_PUBLIC
  static constexpr callback_handle_t invalid_callback_handle = 0;

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
  Player(
    std::unique_ptr<rosbag2_cpp::Reader> reader,
    std::shared_ptr<KeyboardHandler> keyboard_handler,
    const rosbag2_storage::StorageOptions & storage_options,
    const rosbag2_transport::PlayOptions & play_options,
    const std::string & node_name = "rosbag2_player",
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());

  ROSBAG2_TRANSPORT_PUBLIC
  virtual ~Player();

  ROSBAG2_TRANSPORT_PUBLIC
  bool play();

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

protected:
  struct play_msg_callback_data
  {
    callback_handle_t handle;
    play_msg_callback_t callback;
  };

  std::mutex on_play_msg_callbacks_mutex_;
  std::forward_list<play_msg_callback_data> on_play_msg_pre_callbacks_;
  std::forward_list<play_msg_callback_data> on_play_msg_post_callbacks_;

  class PlayerPublisher final
  {
public:
    explicit PlayerPublisher(
      std::shared_ptr<rclcpp::GenericPublisher> pub,
      bool disable_loan_message)
    : publisher_(std::move(pub))
    {
      using std::placeholders::_1;
      if (disable_loan_message || !publisher_->can_loan_messages()) {
        publish_func_ = std::bind(&rclcpp::GenericPublisher::publish, publisher_, _1);
      } else {
        publish_func_ = std::bind(&rclcpp::GenericPublisher::publish_as_loaned_msg, publisher_, _1);
      }
    }

    ~PlayerPublisher() {}

    void publish(const rclcpp::SerializedMessage & message)
    {
      publish_func_(message);
    }

    std::shared_ptr<rclcpp::GenericPublisher> generic_publisher()
    {
      return publisher_;
    }

private:
    std::shared_ptr<rclcpp::GenericPublisher> publisher_;
    std::function<void(const rclcpp::SerializedMessage &)> publish_func_;
  };
  bool is_ready_to_play_from_queue_{false};
  std::mutex ready_to_play_from_queue_mutex_;
  std::condition_variable ready_to_play_from_queue_cv_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher_;
  std::unordered_map<std::string, std::shared_ptr<PlayerPublisher>> publishers_;

private:
  rosbag2_storage::SerializedBagMessageSharedPtr peek_next_message_from_queue();
  void load_storage_content();
  bool is_storage_completely_loaded() const;
  void enqueue_up_to_boundary(size_t boundary) RCPPUTILS_TSA_REQUIRES(reader_mutex_);
  void wait_for_filled_queue() const;
  void play_messages_from_queue();
  void prepare_publishers();
  bool publish_message(rosbag2_storage::SerializedBagMessageSharedPtr message);
  static callback_handle_t get_new_on_play_msg_callback_handle();
  void add_key_callback(
    KeyboardHandler::KeyCode key,
    const std::function<void()> & cb,
    const std::string & op_name);
  void add_keyboard_callbacks();
  void create_control_services();
  void configure_play_until_timestamp();
  bool shall_stop_at_timestamp(const rcutils_time_point_value_t & msg_timestamp) const;

  static constexpr double read_ahead_lower_bound_percentage_ = 0.9;
  static const std::chrono::milliseconds queue_read_wait_period_;
  std::atomic_bool cancel_wait_for_next_message_{false};

  std::mutex reader_mutex_;
  std::unique_ptr<rosbag2_cpp::Reader> reader_ RCPPUTILS_TSA_GUARDED_BY(reader_mutex_);

  rosbag2_storage::StorageOptions storage_options_;
  rosbag2_transport::PlayOptions play_options_;
  rcutils_time_point_value_t play_until_timestamp_ = -1;
  moodycamel::ReaderWriterQueue<rosbag2_storage::SerializedBagMessageSharedPtr> message_queue_;
  mutable std::future<void> storage_loading_future_;
  std::atomic_bool load_storage_content_{true};
  std::unordered_map<std::string, rclcpp::QoS> topic_qos_profile_overrides_;
  std::unique_ptr<rosbag2_cpp::PlayerClock> clock_;
  std::shared_ptr<rclcpp::TimerBase> clock_publish_timer_;
  std::mutex skip_message_in_main_play_loop_mutex_;
  bool skip_message_in_main_play_loop_ RCPPUTILS_TSA_GUARDED_BY(
    skip_message_in_main_play_loop_mutex_) = false;
  std::atomic_bool is_in_playback_{false};

  rcutils_time_point_value_t starting_time_;

  // control services
  rclcpp::Service<rosbag2_interfaces::srv::Pause>::SharedPtr srv_pause_;
  rclcpp::Service<rosbag2_interfaces::srv::Resume>::SharedPtr srv_resume_;
  rclcpp::Service<rosbag2_interfaces::srv::TogglePaused>::SharedPtr srv_toggle_paused_;
  rclcpp::Service<rosbag2_interfaces::srv::IsPaused>::SharedPtr srv_is_paused_;
  rclcpp::Service<rosbag2_interfaces::srv::GetRate>::SharedPtr srv_get_rate_;
  rclcpp::Service<rosbag2_interfaces::srv::SetRate>::SharedPtr srv_set_rate_;
  rclcpp::Service<rosbag2_interfaces::srv::Play>::SharedPtr srv_play_;
  rclcpp::Service<rosbag2_interfaces::srv::PlayNext>::SharedPtr srv_play_next_;
  rclcpp::Service<rosbag2_interfaces::srv::Burst>::SharedPtr srv_burst_;
  rclcpp::Service<rosbag2_interfaces::srv::Seek>::SharedPtr srv_seek_;

  rclcpp::Publisher<rosbag2_interfaces::msg::ReadSplitEvent>::SharedPtr split_event_pub_;

  // defaults
  std::shared_ptr<KeyboardHandler> keyboard_handler_;
  std::vector<KeyboardHandler::callback_handle_t> keyboard_callbacks_;
};

}  // namespace rosbag2_transport

#endif  // ROSBAG2_TRANSPORT__PLAYER_HPP_
