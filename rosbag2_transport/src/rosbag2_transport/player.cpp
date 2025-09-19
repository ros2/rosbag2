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

#include <algorithm>
#include <chrono>
#include <limits>
#include <memory>
#include <regex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include <thread>

#include "rcl/graph.h"

#include "rclcpp/rclcpp.hpp"
#include "rcpputils/unique_lock.hpp"
#include "rcutils/time.h"

#include "rosbag2_cpp/clocks/time_controller_clock.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/service_utils.hpp"

#include "rosbag2_storage/storage_filter.hpp"
#include "rosbag2_storage/qos.hpp"
#include "rosbag2_transport/config_options_from_node_params.hpp"
#include "rosbag2_transport/player.hpp"
#include "rosbag2_transport/player_service_client.hpp"
#include "rosbag2_transport/reader_writer_factory.hpp"
#include "rosbag2_transport/readers_manager.hpp"

#include "logging.hpp"
#include "locked_priority_queue.hpp"

namespace
{
/**
 * Determine which QoS to offer for a topic.
 * The priority of the profile selected is:
 *   1. The override specified in play_options (if one exists for the topic).
 *   2. A profile automatically adapted to the recorded QoS profiles of publishers on the topic.
 *
 * \param topic_name The full name of the topic, with namespace (ex. /arm/joint_status).
 * \param topic_qos_profile_overrides A map of topic to QoS profile overrides.
 * @return The QoS profile to be used for subscribing.
 */
rclcpp::QoS publisher_qos_for_topic(
  const rosbag2_storage::TopicMetadata & topic,
  const std::unordered_map<std::string, rclcpp::QoS> & topic_qos_profile_overrides,
  const rclcpp::Logger & logger)
{
  using rosbag2_storage::Rosbag2QoS;
  auto qos_it = topic_qos_profile_overrides.find(topic.name);
  if (qos_it != topic_qos_profile_overrides.end()) {
    RCLCPP_INFO_STREAM(
      logger,
      "Overriding QoS profile for topic " << topic.name);
    return Rosbag2QoS{qos_it->second};
  } else if (topic.offered_qos_profiles.empty()) {
    return Rosbag2QoS{};
  }

  return Rosbag2QoS::adapt_offer_to_recorded_offers(
    topic.name,
    rosbag2_storage::from_rclcpp_qos_vector(topic.offered_qos_profiles));
}
}  // namespace

namespace rosbag2_transport
{
constexpr Player::callback_handle_t Player::invalid_callback_handle;

class PlayerImpl
{
public:
  using callback_handle_t = Player::callback_handle_t;
  using play_msg_callback_t = Player::play_msg_callback_t;
  using reader_storage_options_pair_t = Player::reader_storage_options_pair_t;

  PlayerImpl(
    Player * owner,
    std::vector<reader_storage_options_pair_t> && readers_with_options,
    std::shared_ptr<KeyboardHandler> keyboard_handler,
    const rosbag2_transport::PlayOptions & play_options);

  virtual ~PlayerImpl();

  bool play();

  /// \brief Unpause if in pause mode, stop playback and exit from play.
  void stop();

  // Playback control interface
  /// Pause the flow of time for playback.
  virtual void pause();

  /// Start the flow of time for playback.
  virtual void resume();

  /// Pause if time running, resume if paused.
  void toggle_paused();

  /// Return whether the playback is currently paused.
  bool is_paused() const;

  /// \brief Getter method for starting time of the playback.
  /// \return Returns timestamp of the first message in nanoseconds.
  rcutils_time_point_value_t get_starting_time() const;

  /// \brief Getter method for playback duration
  /// \return Returns duration of the playback in nanoseconds.
  rcutils_duration_value_t get_playback_duration() const;

  /// Return current playback rate.
  double get_rate() const;

  /// \brief Set the playback rate.
  /// \return false if an invalid value was provided (<= 0).
  virtual bool set_rate(double);

  /// \brief Playing next message from queue when in pause.
  /// \details This is blocking call and it will wait until next available message will be
  /// published or rclcpp context shut down.
  /// \note If internal player queue is starving and storage has not been completely loaded,
  /// this method will wait until new element will be pushed to the queue.
  /// \return true if player in pause mode and successfully played next message, otherwise false.
  virtual bool play_next();

  /// \brief Burst the next \p num_messages messages from the queue when paused.
  /// \param num_messages The number of messages to burst from the queue. Specifying zero means no
  /// limit (i.e. burst the entire bag).
  /// \details This call will play the next \p num_messages from the queue in burst mode. The
  /// timing of the messages is ignored.
  /// \note If internal player queue is starving and storage has not been completely loaded,
  /// this method will wait until new element will be pushed to the queue.
  /// \return The number of messages that was played.
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
  void seek(rcutils_time_point_value_t time_point);

  /// \brief Adding callable object as handler for pre-callback on play message.
  /// \param callback Callable which will be called before next message will be published.
  /// \note In case of registering multiple callbacks later-registered callbacks will be called
  /// first.
  /// \return Returns newly created callback handle if callback was successfully added,
  /// otherwise returns invalid_callback_handle.
  callback_handle_t add_on_play_message_pre_callback(const play_msg_callback_t & callback);

  /// \brief Adding callable object as handler for post-callback on play message.
  /// \param callback Callable which will be called after next message will be published.
  /// \note In case of registering multiple callbacks later-registered callbacks will be called
  /// first.
  /// \return Returns newly created callback handle if callback was successfully added,
  /// otherwise returns invalid_callback_handle.
  callback_handle_t add_on_play_message_post_callback(const play_msg_callback_t & callback);

  /// \brief Delete pre or post on play message callback from internal player lists.
  /// \param handle Callback's handle returned from #add_on_play_message_pre_callback or
  /// #add_on_play_message_post_callback
  void delete_on_play_message_callback(const callback_handle_t & handle);

  /// Wait until sent service requests will receive responses from service servers.
  /// \note The player node shall be spun in the executor in a parallel thread to be able to wait
  /// for responses.
  /// \param service_name - Name of the service or service event from what to wait responses.
  /// \note if service_name is empty the function will wait until all service requests sent to all
  /// service servers will finish. Timeout in this cases will be used for each service name.
  /// \param timeout - Timeout in fraction of seconds to wait for.
  /// \return true if service requests successfully finished, otherwise false.
  bool wait_for_sent_service_requests_to_finish(
    const std::string & service_name,
    std::chrono::duration<double> timeout = std::chrono::seconds(5));

  /// \brief Getter for publishers corresponding to each topic
  /// \return Hashtable representing topic to publisher map excluding inner clock_publisher
  std::unordered_map<std::string, std::shared_ptr<rclcpp::GenericPublisher>> get_publishers();

  /// \brief Getter for clients corresponding to services
  /// \return Hashtable representing service name to client map
  std::unordered_map<std::string, std::shared_ptr<rclcpp::GenericClient>> get_service_clients();

  /// \brief Getter for inner clock_publisher
  /// \return Shared pointer to the inner clock_publisher
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr get_clock_publisher();

  /// \brief Waits on the condition variable until first message will be taken from read queue
  /// @param timeout Maximum time in the fraction of seconds to wait for player to start.
  /// If timeout is negative, the wait_for_playback_to_start will be a blocking call.
  /// @return true if playback successfully started during timeout, otherwise false.
  bool wait_for_playback_to_start(std::chrono::duration<double> timeout = std::chrono::seconds(-1));

  /// \brief Waits on the condition variable until the play thread finishes.
  /// @param timeout Maximum time in the fraction of seconds to wait for player to finish.
  /// If timeout is negative, the wait_for_playback_to_finish will be a blocking call.
  /// @return true if playback finished during timeout, otherwise false.
  bool wait_for_playback_to_finish(
    std::chrono::duration<double> timeout = std::chrono::seconds(-1));

  /// \brief Getter for the number of registered on_play_msg_pre_callbacks
  /// \return Number of registered on_play_msg_pre_callbacks
  size_t get_number_of_registered_on_play_msg_pre_callbacks();

  /// \brief Getter for the number of registered on_play_msg_post_callbacks
  /// \return Number of registered on_play_msg_post_callbacks
  size_t get_number_of_registered_on_play_msg_post_callbacks();

  /// \brief Getter for the first of the currently stored storage options
  /// \return Copy of the first of the currently stored storage options
  const rosbag2_storage::StorageOptions & get_storage_options();

  /// \brief Getter for the currently stored storage options
  /// \return Copy of the currently stored storage options
  std::vector<rosbag2_storage::StorageOptions> get_all_storage_options();

  /// \brief Getter for the currently stored play options
  /// \return Copy of the currently stored play options
  const rosbag2_transport::PlayOptions & get_play_options();

protected:
  struct play_msg_callback_data
  {
    callback_handle_t handle;
    play_msg_callback_t callback;
  };

  std::mutex on_play_msg_callbacks_mutex_;
  std::forward_list<play_msg_callback_data> on_play_msg_pre_callbacks_;
  std::forward_list<play_msg_callback_data> on_play_msg_post_callbacks_;

  void run_play_msg_pre_callbacks(rosbag2_storage::SerializedBagMessageSharedPtr message);
  void run_play_msg_post_callbacks(rosbag2_storage::SerializedBagMessageSharedPtr message);

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

    ~PlayerPublisher() = default;

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
  using PlayerPublisherSharedPtr = std::shared_ptr<PlayerPublisher>;
  using PlayerServiceClientSharedPtr = std::shared_ptr<PlayerServiceClient>;
  std::unordered_map<std::string, PlayerPublisherSharedPtr> publishers_;
  std::unordered_map<std::string, PlayerServiceClientSharedPtr> service_clients_;

private:
  rosbag2_storage::SerializedBagMessageSharedPtr take_next_message_from_queue();
  void load_storage_content();

  /// \brief Returns true if storage_loading_future_ hasn't been launched or successfully finished
  bool is_storage_completely_loaded() const;
  void enqueue_up_to_boundary(size_t boundary, size_t message_queue_size);

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
  std::atomic_bool stop_playback_{false};

  std::unique_ptr<ReadersManager> readers_;

  void publish_clock_update();
  void publish_clock_update(const rclcpp::Time & time);

  Player * owner_;
  rosbag2_transport::PlayOptions play_options_;
  rcutils_time_point_value_t play_until_timestamp_ = -1;
  using BagMessageComparator = std::function<
    int(
      const rosbag2_storage::SerializedBagMessageSharedPtr &,
      const rosbag2_storage::SerializedBagMessageSharedPtr &)>;
  LockedPriorityQueue<
    rosbag2_storage::SerializedBagMessageSharedPtr,
    std::vector<rosbag2_storage::SerializedBagMessageSharedPtr>,
    BagMessageComparator> message_queue_;
  mutable std::future<void> storage_loading_future_;
  std::atomic_bool load_storage_content_{true};
  std::unordered_map<std::string, rclcpp::QoS> topic_qos_profile_overrides_;
  std::unique_ptr<rosbag2_cpp::TimeControllerClock> clock_;
  std::shared_ptr<rclcpp::TimerBase> clock_publish_timer_;
  std::mutex main_play_loop_mutex_;
  bool skip_message_in_main_play_loop_ RCPPUTILS_TSA_GUARDED_BY(main_play_loop_mutex_) = false;
  std::mutex is_in_playback_mutex_;
  std::atomic_bool is_in_playback_ RCPPUTILS_TSA_GUARDED_BY(is_in_playback_mutex_) = false;
  std::thread playback_thread_;
  std::condition_variable is_in_playback_cv_;

  // Request to play next
  std::atomic_bool play_next_{false};
  // Whether we're done trying to play next
  std::atomic_bool finished_play_next_{false};
  std::mutex finished_play_next_mutex_;
  std::condition_variable finished_play_next_cv_;
  // Whether we successfully played next
  std::atomic_bool play_next_result_{false};

  rcutils_time_point_value_t starting_time_ = 0;
  rcutils_duration_value_t playback_duration_ = 0;

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
  rclcpp::Service<rosbag2_interfaces::srv::Stop>::SharedPtr srv_stop_;

  rclcpp::Publisher<rosbag2_interfaces::msg::ReadSplitEvent>::SharedPtr split_event_pub_;

  // defaults
  std::shared_ptr<KeyboardHandler> keyboard_handler_;
  std::vector<KeyboardHandler::callback_handle_t> keyboard_callbacks_;

  std::shared_ptr<PlayerServiceClientManager> player_service_client_manager_;

  /// Comparator for SerializedBagMessageSharedPtr to order chronologically by recv_timestamp.
  static inline const struct
  {
    bool operator()(
      const rosbag2_storage::SerializedBagMessageSharedPtr & l,
      const rosbag2_storage::SerializedBagMessageSharedPtr & r) const
    {
      return l->recv_timestamp > r->recv_timestamp;
    }
  } bag_message_chronological_recv_timestamp_comparator;
};

PlayerImpl::PlayerImpl(
  Player * owner,
  std::vector<reader_storage_options_pair_t> && readers_with_options,
  std::shared_ptr<KeyboardHandler> keyboard_handler,
  const rosbag2_transport::PlayOptions & play_options)
: readers_(std::make_unique<ReadersManager>(std::move(readers_with_options))),
  owner_(owner),
  play_options_(play_options),
  message_queue_(bag_message_chronological_recv_timestamp_comparator),
  keyboard_handler_(std::move(keyboard_handler)),
  player_service_client_manager_(std::make_shared<PlayerServiceClientManager>())
{
  for (auto & topic : play_options_.topics_to_filter) {
    topic = rclcpp::expand_topic_or_service_name(
      topic, owner_->get_name(),
      owner_->get_namespace(), false);
  }

  for (auto & exclude_topic : play_options_.exclude_topics_to_filter) {
    exclude_topic = rclcpp::expand_topic_or_service_name(
      exclude_topic, owner_->get_name(),
      owner_->get_namespace(), false);
  }

  for (auto & service_event_topic : play_options_.services_to_filter) {
    service_event_topic = rclcpp::expand_topic_or_service_name(
      service_event_topic, owner_->get_name(),
      owner_->get_namespace(), false);
  }

  for (auto & exclude_service_event_topic : play_options_.exclude_services_to_filter) {
    exclude_service_event_topic = rclcpp::expand_topic_or_service_name(
      exclude_service_event_topic, owner_->get_name(),
      owner_->get_namespace(), false);
  }

  starting_time_ = readers_->get_earliest_timestamp();
  const rcutils_time_point_value_t ending_time = readers_->get_latest_timestamp();
  // If a non-default (positive) starting time offset is provided in PlayOptions,
  // then add the offset to the starting time obtained from reader metadata
  if (play_options_.start_offset < 0) {
    RCLCPP_WARN_STREAM(
      owner_->get_logger(),
      "Invalid start offset value: " <<
        RCUTILS_NS_TO_S(static_cast<double>(play_options_.start_offset)) <<
        ". Negative start offset ignored.");
  } else {
    starting_time_ += play_options_.start_offset;
  }

  playback_duration_ = ending_time - starting_time_;

  clock_ = std::make_unique<rosbag2_cpp::TimeControllerClock>(
    starting_time_, std::chrono::steady_clock::now,
    std::chrono::milliseconds{100}, play_options_.start_paused);
  set_rate(play_options_.rate);
  topic_qos_profile_overrides_ = play_options_.topic_qos_profile_overrides;
  prepare_publishers();
  configure_play_until_timestamp();

  create_control_services();
  add_keyboard_callbacks();
}

PlayerImpl::~PlayerImpl()
{
  // Remove callbacks from keyboard_handler_ to prevent race conditions.
  // Note: keyboard_handler handles locks between removing & executing callbacks
  if (keyboard_handler_) {
    for (auto cb_handle : keyboard_callbacks_) {
      keyboard_handler_->delete_key_press_callback(cb_handle);
    }
  }

  // Force to stop playback to avoid hangout in case of unexpected exception or when smart
  // pointer to the player object goes out of scope
  stop();

  if (playback_thread_.joinable()) {
    playback_thread_.join();
  }
}

const std::chrono::milliseconds
PlayerImpl::queue_read_wait_period_ = std::chrono::milliseconds(100);

bool PlayerImpl::is_storage_completely_loaded() const
{
  if (storage_loading_future_.valid() &&
    storage_loading_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
  {
    storage_loading_future_.get();
  }
  return !storage_loading_future_.valid();
}

bool PlayerImpl::play()
{
  rcpputils::unique_lock<std::mutex> is_in_playback_lk(is_in_playback_mutex_);
  if (is_in_playback_.exchange(true)) {
    RCLCPP_WARN_STREAM(
      owner_->get_logger(),
      "Trying to play() while in playback, dismissing request.");
    return false;
  }
  is_in_playback_cv_.notify_all();

  // May need to join the previous thread if we are calling play() a second time
  if (playback_thread_.joinable()) {
    playback_thread_.join();
  }
  stop_playback_ = false;

  rclcpp::Duration delay(0, 0);
  if (play_options_.delay >= rclcpp::Duration(0, 0)) {
    delay = play_options_.delay;
  } else {
    RCLCPP_WARN_STREAM(
      owner_->get_logger(),
      "Invalid delay value: " << play_options_.delay.nanoseconds() << ". Delay is disabled.");
  }

  RCLCPP_INFO_STREAM(owner_->get_logger(), "Playback until timestamp: " << play_until_timestamp_);

  playback_thread_ = std::thread(
    [&, delay]() {
      try {
        if (delay > rclcpp::Duration(0, 0)) {
          RCLCPP_INFO_STREAM(owner_->get_logger(), "Sleep " << delay.nanoseconds() << " ns");
          std::chrono::nanoseconds delay_duration(delay.nanoseconds());
          std::this_thread::sleep_for(delay_duration);
        }
        do {
          {
            // By locking main_play_loop_mutex_ we make sure that seek(time_point) will not be
            // called during cleaning message queue and loading storage in next iteration of play
            // loop. i.e. in case when play_options_.loop = true
            std::lock_guard<std::mutex> main_play_loop_lk(main_play_loop_mutex_);
            load_storage_content_ = false;
            if (storage_loading_future_.valid()) {storage_loading_future_.get();}
            message_queue_.purge();
            {
              std::lock_guard<std::mutex> lk(ready_to_play_from_queue_mutex_);
              is_ready_to_play_from_queue_ = false;
              ready_to_play_from_queue_cv_.notify_all();
            }
            readers_->seek(starting_time_);

            load_storage_content_ = true;
            storage_loading_future_ = std::async(
              std::launch::async, [this]() {
                load_storage_content();
              });
            wait_for_filled_queue();

            if (clock_publish_timer_ != nullptr) {
              clock_publish_timer_->reset();
            }
            clock_->jump(starting_time_);
          }
          play_messages_from_queue();
        } while (rclcpp::ok() && !stop_playback_ && play_options_.loop);
      } catch (const std::exception & e) {
        RCLCPP_ERROR(owner_->get_logger(), "Failed to play: %s", e.what());
      }

      // Wait for all published messages to be acknowledged.
      if (play_options_.wait_acked_timeout >= 0) {
        std::chrono::milliseconds timeout(play_options_.wait_acked_timeout);
        if (timeout == std::chrono::milliseconds(0)) {
          timeout = std::chrono::milliseconds(-1);
        }
        for (auto & [topic, pub] : publishers_) {
          try {
            if (!pub->generic_publisher()->wait_for_all_acked(timeout)) {
              RCLCPP_ERROR(
                owner_->get_logger(),
                "Timed out while waiting for all published messages to be acknowledged for topic "
                "%s", topic.c_str());
            }
          } catch (std::exception & e) {
            RCLCPP_ERROR(
              owner_->get_logger(),
              "Exception occurred while waiting for all published messages to be acknowledged for "
              "topic %s : %s", topic.c_str(), e.what());
          }
        }
      }

      {
        rcpputils::unique_lock<std::mutex> is_in_playback_lk(is_in_playback_mutex_);
        is_in_playback_ = false;

        // Stop loading storage content, wait until loading thread will be finished,
        // purge message queue.
        load_storage_content_ = false;
        if (storage_loading_future_.valid()) {storage_loading_future_.get();}
        message_queue_.purge();

        std::lock_guard<std::mutex> lk(ready_to_play_from_queue_mutex_);
        is_ready_to_play_from_queue_ = false;
        ready_to_play_from_queue_cv_.notify_all();
      }
      is_in_playback_cv_.notify_all();

      // If we get here and still have/just got a play next request, make sure to notify play_next()
      // After that, requests will be automatically rejected since is_in_playback_ becomes false
      if (play_next_.exchange(false)) {
        std::lock_guard<std::mutex> lk(finished_play_next_mutex_);
        finished_play_next_ = true;
        play_next_result_ = false;
        finished_play_next_cv_.notify_all();
      }
    });
  return true;
}

bool PlayerImpl::wait_for_playback_to_finish(std::chrono::duration<double> timeout)
{
  rcpputils::unique_lock<std::mutex> is_in_playback_lk(is_in_playback_mutex_);
  if (timeout.count() < 0) {
    is_in_playback_cv_.wait(is_in_playback_lk, [this] {return !is_in_playback_.load();});
    return true;
  } else {
    return is_in_playback_cv_.wait_for(
      is_in_playback_lk,
      timeout, [this] {return !is_in_playback_.load();});
  }
}

void PlayerImpl::stop()
{
  rcpputils::unique_lock<std::mutex> is_in_playback_lk(is_in_playback_mutex_);
  if (is_in_playback_) {
    RCLCPP_INFO_STREAM(owner_->get_logger(), "Stopping playback.");
    stop_playback_ = true;
    // Temporary stop playback in play_messages_from_queue() and block play_next() and seek() or
    // wait until those operations will be finished with stop_playback_ = true;
    {
      std::lock_guard<std::mutex> main_play_loop_lk(main_play_loop_mutex_);
      // resume playback if it was in pause and waiting on clock in play_messages_from_queue()
      skip_message_in_main_play_loop_ = true;
      cancel_wait_for_next_message_ = true;
    }

    // If in pause mode, we need to wake up the clock to let playback thread finish
    if (clock_->is_paused()) {
      // Wake up the clock in case it's in a sleep_until(time) call
      clock_->wakeup();
    }
    // Note: Don't clean up message queue here. It will be cleaned up automatically at the end of
    // playback thread.
    // Wait for playback thread to finish. Make sure that we have unlocked
    // is_in_playback_mutex_, otherwise playback_thread_ will wait forever at the end
    is_in_playback_lk.unlock();
    wait_for_playback_to_finish(std::chrono::seconds(-1));
  } else {
    RCLCPP_DEBUG_STREAM(owner_->get_logger(),
      "Trying to stop when playback is not running. Dismissing stop request.");
  }
}

void PlayerImpl::pause()
{
  clock_->pause();
  RCLCPP_INFO_STREAM(owner_->get_logger(), "Pausing play.");
}

void PlayerImpl::resume()
{
  clock_->resume();
  RCLCPP_INFO_STREAM(owner_->get_logger(), "Resuming play.");
}

void PlayerImpl::toggle_paused()
{
  // Note: Use upper level public API from owner class to facilitate unit tests
  owner_->is_paused() ? owner_->resume() : owner_->pause();
}

bool PlayerImpl::is_paused() const
{
  return clock_->is_paused();
}

rcutils_time_point_value_t PlayerImpl::get_starting_time() const
{
  return starting_time_;
}

rcutils_duration_value_t PlayerImpl::get_playback_duration() const
{
  return playback_duration_;
}

double PlayerImpl::get_rate() const
{
  return clock_->get_rate();
}

bool PlayerImpl::set_rate(double rate)
{
  bool ok = clock_->set_rate(rate);
  if (ok) {
    RCLCPP_INFO_STREAM(owner_->get_logger(), "Set rate to " << rate);
  } else {
    RCLCPP_WARN_STREAM(owner_->get_logger(), "Failed to set rate to invalid value " << rate);
  }
  return ok;
}

rosbag2_storage::SerializedBagMessageSharedPtr PlayerImpl::take_next_message_from_queue()
{
  // Wait until there's a message in the queue, unless there are no new messages or we have to stop
  while (!stop_playback_ && message_queue_.empty() &&
    !is_storage_completely_loaded() && rclcpp::ok())
  {
    RCLCPP_WARN_THROTTLE(
      owner_->get_logger(),
      *owner_->get_clock(),
      1000,
      "Message queue starved. Messages will be delayed. Consider "
      "increasing the --read-ahead-queue-size option.");
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }
  // Note: this only returns nullptr if no more messages
  return message_queue_.take().value_or(nullptr);
}

bool PlayerImpl::play_next()
{
  // First check if we can proceed with playing next message
  {
    rcpputils::unique_lock<std::mutex> is_in_playback_lk(is_in_playback_mutex_);
    if (!is_in_playback_) {
      RCLCPP_WARN_STREAM(owner_->get_logger(), "Called play next, but player is not playing.");
      return false;
    }
    if (!clock_->is_paused()) {
      RCLCPP_WARN_STREAM(owner_->get_logger(), "Called play next, but not in paused state.");
      return false;
    }
  }  // Release is_in_playback_mutex_ before proceeding to avoid deadlock with
     // playback_thread_ which is waiting for this mutex to be released at the end.

  // Use RCLCPP_DEBUG_STREAM to avoid delays in the burst mode
  RCLCPP_DEBUG_STREAM(owner_->get_logger(), "Playing next message.");

  // Request to play next
  play_next_ = true;

  // Wait for play next to be done, and then return the result
  std::unique_lock<std::mutex> lk(finished_play_next_mutex_);
  // Wake up the clock in case it's in a sleep_until(time) call
  clock_->wakeup();
  finished_play_next_ = false;
  finished_play_next_cv_.wait(lk, [this] {return finished_play_next_.load();});
  play_next_ = false;
  return play_next_result_.exchange(false);
}

size_t PlayerImpl::burst(const size_t num_messages)
{
  if (!clock_->is_paused()) {
    RCLCPP_WARN_STREAM(owner_->get_logger(), "Burst can only be used when in the paused state.");
    return 0;
  }

  uint64_t messages_played = 0;

  for (auto ii = 0u; ii < num_messages || num_messages == 0; ++ii) {
    if (play_next()) {
      ++messages_played;
    } else {
      break;
    }
  }

  RCLCPP_INFO_STREAM(owner_->get_logger(), "Burst " << messages_played << " messages.");
  return messages_played;
}

void PlayerImpl::seek(rcutils_time_point_value_t time_point)
{
  {
    rcpputils::unique_lock<std::mutex> is_in_playback_lk(is_in_playback_mutex_);
    if (!is_in_playback_) {
      RCLCPP_WARN(owner_->get_logger(), "Called seek, but player is not playing.");
      return;
    }
  }
  // Temporary stop playback in play_messages_from_queue() and block play_next()
  std::lock_guard<std::mutex> main_play_loop_lk(main_play_loop_mutex_);
  skip_message_in_main_play_loop_ = true;
  // Wait for player to be ready for playback messages from queue i.e. wait for Player:play() to
  // be called if not yet and queue to be filled with messages.
  {
    std::unique_lock<std::mutex> lk(ready_to_play_from_queue_mutex_);
    ready_to_play_from_queue_cv_.wait(lk, [this] {return is_ready_to_play_from_queue_;});
  }
  cancel_wait_for_next_message_ = true;
  // if given seek value is earlier than the beginning of the bag, then clamp
  // it to the beginning of the bag
  if (time_point < starting_time_) {
    time_point = starting_time_;
  }

  // Stop loading storage content, wait until loading thread will be finished,
  // purge message queue, seek readers to the specified time point,
  // jump clock to the specified time point and restart loading storage content.
  load_storage_content_ = false;
  if (storage_loading_future_.valid()) {storage_loading_future_.get();}

  message_queue_.purge();
  readers_->seek(time_point);
  clock_->jump(time_point);

  // Restart queuing thread
  load_storage_content_ = true;
  storage_loading_future_ =
    std::async(std::launch::async, [this]() {load_storage_content();});

  clock_->wakeup();  // Finally wake up the clock in case it's in a sleep_until(time) call
}

Player::callback_handle_t PlayerImpl::add_on_play_message_pre_callback(
  const play_msg_callback_t & callback)
{
  if (callback == nullptr) {
    return Player::invalid_callback_handle;
  }
  std::lock_guard<std::mutex> lk(on_play_msg_callbacks_mutex_);
  callback_handle_t new_handle = get_new_on_play_msg_callback_handle();
  on_play_msg_pre_callbacks_.emplace_front(play_msg_callback_data{new_handle, callback});
  return new_handle;
}

Player::callback_handle_t PlayerImpl::add_on_play_message_post_callback(
  const play_msg_callback_t & callback)
{
  if (callback == nullptr) {
    return Player::invalid_callback_handle;
  }
  std::lock_guard<std::mutex> lk(on_play_msg_callbacks_mutex_);
  callback_handle_t new_handle = get_new_on_play_msg_callback_handle();
  on_play_msg_post_callbacks_.emplace_front(play_msg_callback_data{new_handle, callback});
  return new_handle;
}

void PlayerImpl::delete_on_play_message_callback(const callback_handle_t & handle)
{
  std::lock_guard<std::mutex> lk(on_play_msg_callbacks_mutex_);
  on_play_msg_pre_callbacks_.remove_if(
    [handle](const play_msg_callback_data & data) {
      return data.handle == handle;
    });
  on_play_msg_post_callbacks_.remove_if(
    [handle](const play_msg_callback_data & data) {
      return data.handle == handle;
    });
}

bool PlayerImpl::wait_for_sent_service_requests_to_finish(
  const std::string & service_name,
  std::chrono::duration<double> timeout)
{
  bool is_requests_complete = true;
  if (!service_name.empty()) {
    auto service_event_name = rosbag2_cpp::service_name_to_service_event_topic_name(service_name);
    auto client_iter = service_clients_.find(service_event_name);
    if (client_iter != service_clients_.end()) {
      is_requests_complete = client_iter->second->wait_for_sent_requests_to_finish(timeout);
    } else {
      is_requests_complete = false;
    }
  } else {
    is_requests_complete =
      player_service_client_manager_->wait_for_sent_requests_to_finish(nullptr, timeout);
  }
  return is_requests_complete;
}

std::unordered_map<std::string,
  std::shared_ptr<rclcpp::GenericPublisher>> PlayerImpl::get_publishers()
{
  std::unordered_map<std::string, std::shared_ptr<rclcpp::GenericPublisher>> topic_to_publisher_map;
  for (const auto & [topic, pub] : publishers_) {
    topic_to_publisher_map[topic] = pub->generic_publisher();
  }
  return topic_to_publisher_map;
}

std::unordered_map<std::string,
  std::shared_ptr<rclcpp::GenericClient>> PlayerImpl::get_service_clients()
{
  std::unordered_map<std::string, std::shared_ptr<rclcpp::GenericClient>> topic_to_client_map;
  for (const auto & [service_name, client] : service_clients_) {
    topic_to_client_map[service_name] = client->generic_client();
  }
  return topic_to_client_map;
}

rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr PlayerImpl::get_clock_publisher()
{
  return clock_publisher_;
}

bool PlayerImpl::wait_for_playback_to_start(std::chrono::duration<double> timeout)
{
  using namespace std::chrono_literals;  // NOLINT
  // Lambda for try_lock_with_timeout
  auto try_lock_with_timeout =
    [](std::mutex & mutex, std::chrono::duration<double> max_wait,
    std::chrono::milliseconds poll_interval = 10ms) -> std::unique_lock<std::mutex>
    {
      auto start = std::chrono::steady_clock::now();
      std::unique_lock<std::mutex> lock(mutex, std::defer_lock);  // Do not lock yet

      if (max_wait.count() < 0) {
      // If timeout is negative, wait indefinitely
        lock.lock();
        return lock;  // Lock acquired
      } else {
        while (std::chrono::steady_clock::now() - start < max_wait) {
          if (lock.try_lock()) {
            return lock;  // Lock acquired
          }
          std::this_thread::sleep_for(poll_interval);
        }
      }
      return {};  // Return empty (non-owning) lock
    };

  auto start = std::chrono::steady_clock::now();
  auto is_in_playback_lk = try_lock_with_timeout(is_in_playback_mutex_, timeout);
  if (!is_in_playback_lk.owns_lock()) {
    RCLCPP_DEBUG(owner_->get_logger(),
                 "Timeout occurred. The ready_to_play_from_queue_mutex_ hasn't been acquired.");
    return false;
  }
  if (timeout.count() < 0) {
    is_in_playback_cv_.wait(is_in_playback_lk, [this] {return is_in_playback_.load();});
  } else {
    auto lock_duration = std::chrono::steady_clock::now() - start;
    auto residual_time =
      timeout > lock_duration ? timeout - lock_duration : std::chrono::microseconds(1);
    is_in_playback_cv_.wait_for(
      is_in_playback_lk, residual_time, [this] {return is_in_playback_.load();});
  }
  if (!is_in_playback_) {
    RCLCPP_DEBUG(owner_->get_logger(), "Timeout occurred. The play() hasn't been called.");
    return false;
  }
  auto lock_duration = std::chrono::steady_clock::now() - start;
  auto residual_time =
    timeout > lock_duration ? timeout - lock_duration : std::chrono::microseconds(1);
  auto ready_to_play_lock = try_lock_with_timeout(ready_to_play_from_queue_mutex_, residual_time);
  if (!ready_to_play_lock.owns_lock()) {
    RCLCPP_DEBUG(owner_->get_logger(),
                 "Timeout occurred. The ready_to_play_from_queue_mutex_ hasn't been acquired.");
    return false;
  }

  if (timeout.count() < 0) {
    ready_to_play_from_queue_cv_.wait(
      ready_to_play_lock, [this] {return is_ready_to_play_from_queue_;});
    return true;
  } else {
    lock_duration = std::chrono::steady_clock::now() - start;
    residual_time = timeout > lock_duration ?
      timeout - lock_duration : std::chrono::microseconds(1);
    return ready_to_play_from_queue_cv_.wait_for(
      ready_to_play_lock, residual_time, [this] {return is_ready_to_play_from_queue_;}
    );
  }
}

size_t PlayerImpl::get_number_of_registered_on_play_msg_pre_callbacks()
{
  size_t callback_counter = 0;
  std::lock_guard<std::mutex> lk(on_play_msg_callbacks_mutex_);
  for (auto & pre_callback_data : on_play_msg_pre_callbacks_) {
    (void)pre_callback_data;
    callback_counter++;
  }
  return callback_counter;
}

size_t PlayerImpl::get_number_of_registered_on_play_msg_post_callbacks()
{
  size_t callback_counter = 0;
  std::lock_guard<std::mutex> lk(on_play_msg_callbacks_mutex_);
  for (auto & post_callback_data : on_play_msg_post_callbacks_) {
    (void)post_callback_data;
    callback_counter++;
  }
  return callback_counter;
}

Player::callback_handle_t PlayerImpl::get_new_on_play_msg_callback_handle()
{
  static std::atomic<callback_handle_t> handle_count{0};
  return handle_count.fetch_add(1, std::memory_order_relaxed) + 1;
}

void PlayerImpl::wait_for_filled_queue() const
{
  while (
    message_queue_.size() < play_options_.read_ahead_queue_size &&
    !is_storage_completely_loaded() && rclcpp::ok() && !stop_playback_)
  {
    std::this_thread::sleep_for(queue_read_wait_period_);
  }
}

void PlayerImpl::load_storage_content()
{
  auto queue_lower_boundary =
    static_cast<size_t>(play_options_.read_ahead_queue_size * read_ahead_lower_bound_percentage_);
  auto queue_upper_boundary = play_options_.read_ahead_queue_size;

  while (load_storage_content_ && !stop_playback_ && readers_->has_next()) {
    // The message queue size may get smaller after this, but that's OK
    const size_t message_queue_size = message_queue_.size();
    if (message_queue_size < queue_lower_boundary) {
      enqueue_up_to_boundary(queue_upper_boundary, message_queue_size);
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
}

void PlayerImpl::enqueue_up_to_boundary(size_t boundary, size_t message_queue_size)
{
  while (load_storage_content_ && message_queue_size < boundary) {
    auto next_message = readers_->get_next_message_in_chronological_order();
    if (next_message != nullptr) {
      message_queue_.push(next_message);
      message_queue_size++;
    } else {
      // If we have no more messages, we shall stop reading and exit the loop to avoid endless
      // cycle.
      break;
    }
  }
}

void PlayerImpl::play_messages_from_queue()
{
  rosbag2_storage::SerializedBagMessageSharedPtr message_ptr = nullptr;
  { // Notify play_next()/seek() that we are ready for playback
    std::lock_guard<std::mutex> lk(ready_to_play_from_queue_mutex_);
    is_ready_to_play_from_queue_ = true;
    //  Take the first message from the queue under the lock to avoid race with seek(time_point)
    message_ptr = take_next_message_from_queue();
  }
  ready_to_play_from_queue_cv_.notify_all();

  // While we haven't stopped playing, try to play messages and wait for a potential request to play
  // the next message.
  std::unique_lock<std::mutex> main_play_loop_lk(main_play_loop_mutex_);
  while (rclcpp::ok() && !stop_playback_) {
    main_play_loop_lk.unlock();  // Unlock while we are waiting/sleeping
    if (is_paused()) {
      // If we're paused, we need to wait until the stop of playback or until we get a play next.
      // Also while we're in a pause state, make sure we don't return if we happen to be at the end
      // of the queue. We need to be able to make seek back in time from the end of the queue
      // while in pause.
      while (rclcpp::ok() && is_paused() && !stop_playback_ && !play_next_.load()) {
        // When clock is in pause mode, sleep_until() will be blocked until we wake it up or
        // change the clock state from pause to resume, so this is not a tight busy loop on pause.
        (void)clock_->sleep_until(clock_->now());
        // Stop sleeping if cancelled
        if (std::atomic_exchange(&cancel_wait_for_next_message_, false)) {
          break;
        }
      }
      if (!is_paused()) {
        main_play_loop_lk.lock();
        continue;  // If we just exited from the pause mode, go back to the top of the loop to
        // handle the non-paused case
      }
    } else {
      // While there's a message to play, and we haven't reached the end yet.
      if (message_ptr == nullptr ||
        shall_stop_at_timestamp(message_ptr->recv_timestamp))
      {
        break;
      }
      // Sleep until the message's replay time, do not move on until sleep_until returns true
      // However, skip sleeping if we're trying to play the next message
      while (rclcpp::ok() && !stop_playback_ && !play_next_.load() &&
        !clock_->sleep_until(message_ptr->recv_timestamp))
      {
        // Stop sleeping if cancelled
        if (std::atomic_exchange(&cancel_wait_for_next_message_, false)) {
          break;
        }
      }
    }

    main_play_loop_lk.lock();
    if (!stop_playback_) {
      // This means that the message we took from the queue's top was invalidated after a seek(),
      // so we need to take a fresh element from the top of the queue, unless we have to stop
      if (skip_message_in_main_play_loop_) {
        skip_message_in_main_play_loop_ = false;
        cancel_wait_for_next_message_ = false;
        message_ptr = take_next_message_from_queue();
        continue;
      }

      if (message_ptr == nullptr) {
        // We can get here with a null message_ptr if we are processing play_next() but there are
        // actually no more messages to play. In that case, we just notify play_next(). Or if we
        // have reached the end of playback and resumed playback from pause mode. In this case,
        // we need to continue loop to gracefully exit from play_messages_from_queue()
        if (play_next_.load()) {
          play_next_ = false;
          std::lock_guard<std::mutex> lk(finished_play_next_mutex_);
          finished_play_next_ = true;
          play_next_result_ = false;
          finished_play_next_cv_.notify_all();
        }
        continue;
      }

      bool message_published = false;
      // We shall respect shall_stop_at_timestamp() even do play_next in pause mode
      if (!shall_stop_at_timestamp(message_ptr->recv_timestamp)) {
        message_published = publish_message(message_ptr);
      }

      // If we tried to publish because of play_next(), jump the clock
      if (play_next_.load()) {
        clock_->jump(message_ptr->recv_timestamp);
        play_next_ = false;
        std::lock_guard<std::mutex> lk(finished_play_next_mutex_);
        finished_play_next_ = true;
        play_next_result_ = message_published;
        finished_play_next_cv_.notify_all();
      }
    }
    message_ptr = take_next_message_from_queue();
  }
  // Note: We are still under the lock of main_play_loop_mutex_ here
  { // Notify seek() that we are NOT ready for playback anymore. The seek() will wait until
    // play_messages_from_queue() will be called again in the next iteration of the play loop
    // in Player::play(). This is to avoid race condition between seek() and
    // play_messages_from_queue() when play_options_.loop = true
    std::lock_guard<std::mutex> lk(ready_to_play_from_queue_mutex_);
    is_ready_to_play_from_queue_ = false;
  }
  ready_to_play_from_queue_cv_.notify_all();
}

namespace
{
bool allow_topic(
  bool is_service,
  const std::string & topic_name,
  const rosbag2_storage::StorageFilter & storage_filter)
{
  auto & include_topics = storage_filter.topics;
  auto & exclude_topics = storage_filter.exclude_topics;
  auto & include_services = storage_filter.services_events;
  auto & exclude_services = storage_filter.exclude_service_events;
  auto & regex = storage_filter.regex;
  auto & regex_to_exclude = storage_filter.regex_to_exclude;

  if (is_service) {
    if (!exclude_services.empty()) {
      auto it = std::find(exclude_services.begin(), exclude_services.end(), topic_name);
      if (it != exclude_services.end()) {
        return false;
      }
    }
  } else {
    if (!exclude_topics.empty()) {
      auto it = std::find(exclude_topics.begin(), exclude_topics.end(), topic_name);
      if (it != exclude_topics.end()) {
        return false;
      }
    }
  }

  if (!regex_to_exclude.empty()) {
    std::smatch m;
    std::regex re(regex_to_exclude);

    if (std::regex_match(topic_name, m, re)) {
      return false;
    }
  }

  bool set_include = is_service ? !include_services.empty() : !include_topics.empty();
  bool set_regex = !regex.empty();

  if (set_include || set_regex) {
    if (is_service) {
      auto iter = std::find(include_services.begin(), include_services.end(), topic_name);
      if (iter == include_services.end()) {
        // If include_service is set and regex isn't set, service must be in include_service.
        if (!set_regex) {
          return false;
        }
      } else {
        return true;
      }
    } else {
      auto iter = std::find(include_topics.begin(), include_topics.end(), topic_name);
      if (iter == include_topics.end()) {
        // If include_service is set and regex isn't set, service must be in include_service.
        if (!set_regex) {
          return false;
        }
      } else {
        return true;
      }
    }

    if (set_regex) {
      std::smatch m;
      std::regex re(regex);

      if (!std::regex_match(topic_name, m, re)) {
        return false;
      }
    }
  }

  return true;
}
}  // namespace

void PlayerImpl::prepare_publishers()
{
  rosbag2_storage::StorageFilter storage_filter;
  storage_filter.topics = play_options_.topics_to_filter;
  storage_filter.services_events = play_options_.services_to_filter;
  storage_filter.regex = play_options_.regex_to_filter;
  storage_filter.regex_to_exclude = play_options_.exclude_regex_to_filter;
  storage_filter.exclude_topics = play_options_.exclude_topics_to_filter;
  storage_filter.exclude_service_events = play_options_.exclude_services_to_filter;

  readers_->set_filter(storage_filter);

  // Create /clock publisher
  if (play_options_.clock_publish_frequency > 0.f || play_options_.clock_publish_on_topic_publish) {
    // NOTE: PlayerClock does not own this publisher because rosbag2_cpp
    // should not own transport-based functionality
    clock_publisher_ = owner_->create_publisher<rosgraph_msgs::msg::Clock>(
      "/clock", rclcpp::ClockQoS());
  }

  if (play_options_.clock_publish_frequency > 0.f) {
    const auto publish_period = std::chrono::nanoseconds(
      static_cast<uint64_t>(RCUTILS_S_TO_NS(1) / play_options_.clock_publish_frequency));

    clock_publish_timer_ = owner_->create_wall_timer(
      publish_period, [this]() {
        publish_clock_update();
      }, nullptr, false);
  }

  if (play_options_.clock_publish_on_topic_publish) {
    add_on_play_message_pre_callback(
      [this](const std::shared_ptr<rosbag2_storage::SerializedBagMessage> msg) {
        if (play_options_.clock_trigger_topics.empty()) {
          publish_clock_update();
        } else {
          for (const auto & topic : play_options_.clock_trigger_topics) {
            if (topic == msg->topic_name) {
              publish_clock_update();
              break;
            }
          }
        }
      });
  }

  // Create topic publishers
  // We could have duplicate topic names here, but we correctly handle it when creating publishers
  std::vector<rosbag2_storage::TopicMetadata> topics = readers_->get_all_topics_and_types();
  std::string topic_without_support_acked;
  for (const auto & topic : topics) {
    const bool is_service_event_topic = rosbag2_cpp::is_service_event_topic(topic.name, topic.type);
    if (play_options_.publish_service_requests && is_service_event_topic) {
      // Check if sender was created
      if (service_clients_.find(topic.name) != service_clients_.end()) {
        continue;
      }

      // filter service event topic to add client if necessary
      if (!allow_topic(true, topic.name, storage_filter)) {
        continue;
      }

      auto service_name = rosbag2_cpp::service_event_topic_name_to_service_name(topic.name);
      auto service_type = rosbag2_cpp::service_event_topic_type_to_service_type(topic.type);
      try {
        auto generic_client = owner_->create_generic_client(service_name, service_type);
        auto player_client = std::make_shared<PlayerServiceClient>(
          std::move(generic_client), service_name, topic.type, owner_->get_logger(),
          player_service_client_manager_);
        service_clients_.insert(std::make_pair(topic.name, player_client));
      } catch (const std::runtime_error & e) {
        RCLCPP_WARN(
          owner_->get_logger(),
          "Ignoring a service '%s', reason: %s.", service_name.c_str(), e.what());
      }
    } else {
      // Check if sender was created
      if (publishers_.find(topic.name) != publishers_.end()) {
        continue;
      }

      // filter topics to add publishers if necessary
      if (!allow_topic(is_service_event_topic, topic.name, storage_filter)) {
        continue;
      }

      auto topic_qos = publisher_qos_for_topic(
        topic, topic_qos_profile_overrides_,
        owner_->get_logger());
      try {
        std::shared_ptr<rclcpp::GenericPublisher> pub =
          owner_->create_generic_publisher(topic.name, topic.type, topic_qos);
        std::shared_ptr<PlayerPublisher> player_pub =
          std::make_shared<PlayerPublisher>(std::move(pub), play_options_.disable_loan_message);
        publishers_.insert(std::make_pair(topic.name, player_pub));
        if (play_options_.wait_acked_timeout >= 0 &&
          topic_qos.reliability() == rclcpp::ReliabilityPolicy::BestEffort)
        {
          topic_without_support_acked += topic.name + ", ";
        }
      } catch (const std::runtime_error & e) {
        // using a warning log seems better than adding a new option
        // to ignore some unknown message type library
        RCLCPP_WARN(
          owner_->get_logger(),
          "Ignoring a topic '%s', reason: %s.", topic.name.c_str(), e.what());
      }
    }
  }

  if (!topic_without_support_acked.empty()) {
    // remove the last ", "
    topic_without_support_acked.erase(
      topic_without_support_acked.end() - 2,
      topic_without_support_acked.end());

    RCLCPP_WARN(
      owner_->get_logger(),
      "--wait-for-all-acked is invalid for the below topics since reliability of QOS is "
      "BestEffort.\n%s", topic_without_support_acked.c_str());
  }

  // Create a publisher and callback for when encountering a split in the input
  split_event_pub_ = owner_->create_publisher<rosbag2_interfaces::msg::ReadSplitEvent>(
    "events/read_split",
    1);
  rosbag2_cpp::bag_events::ReaderEventCallbacks callbacks;
  callbacks.read_split_callback =
    [this](rosbag2_cpp::bag_events::BagSplitInfo & info) {
      auto message = rosbag2_interfaces::msg::ReadSplitEvent();
      message.closed_file = info.closed_file;
      message.opened_file = info.opened_file;
      message.node_name = owner_->get_fully_qualified_name();
      split_event_pub_->publish(message);
    };
  readers_->add_event_callbacks(callbacks);
}

void PlayerImpl::run_play_msg_pre_callbacks(
  rosbag2_storage::SerializedBagMessageSharedPtr message)
{
  std::lock_guard<std::mutex> lk(on_play_msg_callbacks_mutex_);
  for (auto & pre_callback_data : on_play_msg_pre_callbacks_) {
    if (pre_callback_data.callback != nullptr) {  // Sanity check
      pre_callback_data.callback(message);
    }
  }
}

void PlayerImpl::run_play_msg_post_callbacks(
  rosbag2_storage::SerializedBagMessageSharedPtr message)
{
  std::lock_guard<std::mutex> lk(on_play_msg_callbacks_mutex_);
  for (auto & post_callback_data : on_play_msg_post_callbacks_) {
    if (post_callback_data.callback != nullptr) {  // Sanity check
      post_callback_data.callback(message);
    }
  }
}

bool PlayerImpl::publish_message(rosbag2_storage::SerializedBagMessageSharedPtr message)
{
  auto pub_iter = publishers_.find(message->topic_name);
  if (pub_iter != publishers_.end()) {
    bool message_published = false;
    bool pre_callbacks_failed = true;
    try {
      // Calling on play message pre-callbacks
      run_play_msg_pre_callbacks(message);
      pre_callbacks_failed = false;
    } catch (const std::exception & e) {
      RCLCPP_ERROR_STREAM(owner_->get_logger(),
        "Failed to call on play message pre-callback on '" << message->topic_name <<
        "' topic. \nError: " << e.what());
    }

    if (!pre_callbacks_failed) {
      try {
        pub_iter->second->publish(rclcpp::SerializedMessage(*message->serialized_data));
        message_published = true;
      } catch (const std::exception & e) {
        RCLCPP_ERROR_STREAM(owner_->get_logger(),
          "Failed to publish message on '" << message->topic_name <<
          "' topic. \nError: " << e.what());
      }
    }

    try {
      // Calling on play message post-callbacks
      run_play_msg_post_callbacks(message);
    } catch (const std::exception & e) {
      RCLCPP_ERROR_STREAM(owner_->get_logger(),
        "Failed to call on play message post-callback on '" << message->topic_name <<
        "' topic. \nError: " << e.what());
    }
    return message_published;
  }

  // Try to publish message as service request
  auto client_iter = service_clients_.find(message->topic_name);
  if (play_options_.publish_service_requests && client_iter != service_clients_.end()) {
    const auto & service_client = client_iter->second;
    auto service_event = service_client->deserialize_service_event(*message->serialized_data);
    if (!service_event) {
      RCLCPP_ERROR_STREAM(
        owner_->get_logger(), "Failed to deserialize service event message for '" <<
          service_client->get_service_name() << "' service!\n");
      return false;
    }

    try {
      auto [service_event_type, client_gid] =
        service_client->get_service_event_type_and_client_gid(service_event);
      // Ignore response message
      if (service_event_type == service_msgs::msg::ServiceEventInfo::RESPONSE_SENT ||
        service_event_type == service_msgs::msg::ServiceEventInfo::RESPONSE_RECEIVED)
      {
        return false;
      }

      if (play_options_.service_requests_source == ServiceRequestsSource::SERVICE_INTROSPECTION &&
        service_event_type != service_msgs::msg::ServiceEventInfo::REQUEST_RECEIVED)
      {
        return false;
      }

      if (play_options_.service_requests_source == ServiceRequestsSource::CLIENT_INTROSPECTION &&
        service_event_type != service_msgs::msg::ServiceEventInfo::REQUEST_SENT)
      {
        return false;
      }

      if (!service_client->generic_client()->service_is_ready()) {
        RCLCPP_ERROR(
          owner_->get_logger(), "Service request hasn't been sent. The '%s' service isn't ready !",
          service_client->get_service_name().c_str());
        return false;
      }

      if (!service_client->is_service_event_include_request_message(service_event)) {
        if (service_event_type == service_msgs::msg::ServiceEventInfo::REQUEST_RECEIVED) {
          RCUTILS_LOG_WARN_ONCE_NAMED(
            ROSBAG2_TRANSPORT_PACKAGE_NAME,
            "Can't send service request. "
            "The configuration of introspection for '%s' was metadata only on service side!",
            service_client->get_service_name().c_str());
        } else if (service_event_type == service_msgs::msg::ServiceEventInfo::REQUEST_SENT) {
          RCUTILS_LOG_WARN_ONCE_NAMED(
            ROSBAG2_TRANSPORT_PACKAGE_NAME,
            "Can't send service request. "
            "The configuration of introspection for '%s' client [ID: %s]` was metadata only!",
            service_client->get_service_name().c_str(),
            rosbag2_cpp::client_id_to_string(client_gid).c_str());
        }
        return false;
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR_STREAM(
        owner_->get_logger(), "Failed to send request on '" <<
          rosbag2_cpp::service_event_topic_name_to_service_name(message->topic_name) <<
          "' service. \nError: " << e.what());
      return false;
    }

    // Calling on play message pre-callbacks
    run_play_msg_pre_callbacks(message);

    bool message_published = false;
    try {
      service_client->async_send_request(service_event);
      message_published = true;
    } catch (const std::exception & e) {
      RCLCPP_ERROR_STREAM(
        owner_->get_logger(), "Failed to send request on '" <<
          rosbag2_cpp::service_event_topic_name_to_service_name(message->topic_name) <<
          "' service. \nError: " << e.what());
    }

    // Calling on play message post-callbacks
    run_play_msg_post_callbacks(message);
    return message_published;
  }

  RCUTILS_LOG_WARN_ONCE_NAMED(
    ROSBAG2_TRANSPORT_PACKAGE_NAME,
    "Publisher for topic '%s' not found", message->topic_name.c_str());

  return false;
}

void PlayerImpl::add_key_callback(
  KeyboardHandler::KeyCode key,
  const std::function<void()> & cb,
  const std::string & op_name)
{
  if (keyboard_handler_) {
    std::string key_str = enum_key_code_to_str(key);
    if (key == KeyboardHandler::KeyCode::UNKNOWN) {
      RCLCPP_ERROR_STREAM(
        owner_->get_logger(),
        "Invalid key binding " << key_str << " for " << op_name);
      throw std::invalid_argument("Invalid key binding.");
    }
    keyboard_callbacks_.push_back(
      keyboard_handler_->add_key_press_callback(
        [cb](KeyboardHandler::KeyCode /*key_code*/,
        KeyboardHandler::KeyModifiers /*key_modifiers*/) {cb();},
        key));
    // show instructions
    RCLCPP_INFO_STREAM(owner_->get_logger(), "Press " << key_str << " for " << op_name);
  }
}

void PlayerImpl::add_keyboard_callbacks()
{
  // skip if disabled
  if (play_options_.disable_keyboard_controls) {
    return;
  }
  RCLCPP_INFO_STREAM(owner_->get_logger(), "Adding keyboard callbacks.");
  // Add keybindings
  // Note: Use upper level public API from owner class for callbacks to facilitate unit tests
  add_key_callback(
    play_options_.pause_resume_toggle_key,
    [this]() {owner_->toggle_paused();},
    "Pause/Resume"
  );
  add_key_callback(
    play_options_.play_next_key,
    [this]() {owner_->play_next();},
    "Play Next Message"
  );
  add_key_callback(
    play_options_.increase_rate_key,
    [this]() {owner_->set_rate(get_rate() + 0.1);},
    "Increase Rate 10%"
  );
  add_key_callback(
    play_options_.decrease_rate_key,
    [this]() {owner_->set_rate(get_rate() - 0.1);},
    "Decrease Rate 10%"
  );
}

void PlayerImpl::create_control_services()
{
  // Note: Use upper level public API from owner class for callbacks to facilitate unit tests
  srv_pause_ = owner_->create_service<rosbag2_interfaces::srv::Pause>(
    "~/pause",
    [this](
      rosbag2_interfaces::srv::Pause::Request::ConstSharedPtr,
      rosbag2_interfaces::srv::Pause::Response::SharedPtr)
    {
      owner_->pause();
    });
  srv_resume_ = owner_->create_service<rosbag2_interfaces::srv::Resume>(
    "~/resume",
    [this](
      rosbag2_interfaces::srv::Resume::Request::ConstSharedPtr,
      rosbag2_interfaces::srv::Resume::Response::SharedPtr)
    {
      owner_->resume();
    });
  srv_toggle_paused_ = owner_->create_service<rosbag2_interfaces::srv::TogglePaused>(
    "~/toggle_paused",
    [this](
      rosbag2_interfaces::srv::TogglePaused::Request::ConstSharedPtr,
      rosbag2_interfaces::srv::TogglePaused::Response::SharedPtr)
    {
      owner_->toggle_paused();
    });
  srv_is_paused_ = owner_->create_service<rosbag2_interfaces::srv::IsPaused>(
    "~/is_paused",
    [this](
      rosbag2_interfaces::srv::IsPaused::Request::ConstSharedPtr,
      rosbag2_interfaces::srv::IsPaused::Response::SharedPtr response)
    {
      response->paused = owner_->is_paused();
    });
  srv_get_rate_ = owner_->create_service<rosbag2_interfaces::srv::GetRate>(
    "~/get_rate",
    [this](
      rosbag2_interfaces::srv::GetRate::Request::ConstSharedPtr,
      rosbag2_interfaces::srv::GetRate::Response::SharedPtr response)
    {
      response->rate = owner_->get_rate();
    });
  srv_set_rate_ = owner_->create_service<rosbag2_interfaces::srv::SetRate>(
    "~/set_rate",
    [this](
      rosbag2_interfaces::srv::SetRate::Request::ConstSharedPtr request,
      rosbag2_interfaces::srv::SetRate::Response::SharedPtr response)
    {
      response->success = owner_->set_rate(request->rate);
    });
  srv_play_ = owner_->create_service<rosbag2_interfaces::srv::Play>(
    "~/play",
    [this](
      rosbag2_interfaces::srv::Play::Request::ConstSharedPtr request,
      rosbag2_interfaces::srv::Play::Response::SharedPtr response)
    {
      play_options_.start_offset = rclcpp::Time(request->start_offset).nanoseconds();
      play_options_.playback_duration = rclcpp::Duration(request->playback_duration);
      play_options_.playback_until_timestamp =
      rclcpp::Time(request->playback_until_timestamp).nanoseconds();
      configure_play_until_timestamp();
      response->success = owner_->play();
    });
  srv_play_next_ = owner_->create_service<rosbag2_interfaces::srv::PlayNext>(
    "~/play_next",
    [this](
      rosbag2_interfaces::srv::PlayNext::Request::ConstSharedPtr,
      rosbag2_interfaces::srv::PlayNext::Response::SharedPtr response)
    {
      response->success = owner_->play_next();
    });
  srv_burst_ = owner_->create_service<rosbag2_interfaces::srv::Burst>(
    "~/burst",
    [this](
      rosbag2_interfaces::srv::Burst::Request::ConstSharedPtr request,
      rosbag2_interfaces::srv::Burst::Response::SharedPtr response)
    {
      response->actually_burst = owner_->burst(request->num_messages);
    });
  srv_seek_ = owner_->create_service<rosbag2_interfaces::srv::Seek>(
    "~/seek",
    [this](
      rosbag2_interfaces::srv::Seek::Request::ConstSharedPtr request,
      rosbag2_interfaces::srv::Seek::Response::SharedPtr response)
    {
      owner_->seek(rclcpp::Time(request->time).nanoseconds());
      response->success = true;
    });
  srv_stop_ = owner_->create_service<rosbag2_interfaces::srv::Stop>(
    "~/stop",
    [this](
      rosbag2_interfaces::srv::Stop::Request::ConstSharedPtr,
      rosbag2_interfaces::srv::Stop::Response::SharedPtr)
    {
      owner_->stop();
    });
}

void PlayerImpl::configure_play_until_timestamp()
{
  if (play_options_.playback_duration >= rclcpp::Duration(0, 0) ||
    play_options_.playback_until_timestamp >= rcutils_time_point_value_t{0})
  {
    // Handling special case when playback_duration = 0
    auto play_until_from_duration = (play_options_.playback_duration == rclcpp::Duration(0, 0)) ?
      0 : starting_time_ + play_options_.playback_duration.nanoseconds();

    play_until_timestamp_ =
      std::max(play_until_from_duration, play_options_.playback_until_timestamp);
  } else {
    play_until_timestamp_ = -1;
  }
}

inline bool PlayerImpl::shall_stop_at_timestamp(
  const rcutils_time_point_value_t & msg_timestamp) const
{
  if ((play_until_timestamp_ > -1 && msg_timestamp > play_until_timestamp_) ||
    play_until_timestamp_ == 0)
  {
    return true;
  } else {
    return false;
  }
}

void PlayerImpl::publish_clock_update()
{
  publish_clock_update(rclcpp::Time(clock_->now()));
}

void PlayerImpl::publish_clock_update(const rclcpp::Time & time)
{
  if (clock_publisher_->can_loan_messages()) {
    auto loaned_timestamp{clock_publisher_->borrow_loaned_message()};
    loaned_timestamp.get().clock = time;
    clock_publisher_->publish(std::move(loaned_timestamp));
  } else {
    rosgraph_msgs::msg::Clock timestamp;
    timestamp.clock = time;
    clock_publisher_->publish(timestamp);
  }
}

const rosbag2_storage::StorageOptions & PlayerImpl::get_storage_options()
{
  auto all_storage_options = get_all_storage_options();
  if (all_storage_options.size() < 1) {
    throw std::runtime_error("Storage options not available.");
  }
  return all_storage_options[0];
}

std::vector<rosbag2_storage::StorageOptions> PlayerImpl::get_all_storage_options()
{
  return readers_->get_all_storage_options();
}

const rosbag2_transport::PlayOptions & PlayerImpl::get_play_options()
{
  return play_options_;
}

///////////////////////////////
// Player public interface

Player::Player(const rclcpp::NodeOptions & node_options)
: Player("rosbag2_player", node_options) {}

Player::Player(const std::string & node_name, const rclcpp::NodeOptions & node_options)
: rclcpp::Node(node_name, node_options)
{
  rosbag2_storage::StorageOptions storage_options = get_storage_options_from_node_params(*this);
  PlayOptions play_options = get_play_options_from_node_params(*this);

  std::shared_ptr<KeyboardHandler> keyboard_handler;
  if (!play_options.disable_keyboard_controls) {
    keyboard_handler = std::make_shared<KeyboardHandler>();
  }

  auto reader = ReaderWriterFactory::make_reader(storage_options);

  std::vector<reader_storage_options_pair_t> readers_with_options{};
  readers_with_options.emplace_back(std::move(reader), storage_options);
  pimpl_ = std::make_unique<PlayerImpl>(
    this, std::move(readers_with_options), keyboard_handler, play_options);
  pimpl_->play();
}

Player::Player(
  const rosbag2_storage::StorageOptions & storage_options,
  const rosbag2_transport::PlayOptions & play_options,
  const std::string & node_name,
  const rclcpp::NodeOptions & node_options)
: Player(std::vector{storage_options}, play_options, node_name, node_options)
{}

Player::Player(
  const std::vector<rosbag2_storage::StorageOptions> & storage_options,
  const rosbag2_transport::PlayOptions & play_options,
  const std::string & node_name,
  const rclcpp::NodeOptions & node_options)
: rclcpp::Node(
    node_name,
    rclcpp::NodeOptions(node_options).arguments(play_options.topic_remapping_options))
{
  std::shared_ptr<KeyboardHandler> keyboard_handler;
  if (!play_options.disable_keyboard_controls) {
    keyboard_handler = std::make_shared<KeyboardHandler>();
  }

  std::vector<reader_storage_options_pair_t> readers_with_options{};
  for (const auto & options : storage_options) {
    readers_with_options.emplace_back(ReaderWriterFactory::make_reader(options), options);
  }
  pimpl_ = std::make_unique<PlayerImpl>(
    this, std::move(readers_with_options), keyboard_handler, play_options);
}

Player::Player(
  std::unique_ptr<rosbag2_cpp::Reader> reader,
  const rosbag2_storage::StorageOptions & storage_options,
  const rosbag2_transport::PlayOptions & play_options,
  const std::string & node_name,
  const rclcpp::NodeOptions & node_options)
: Player(std::move(reader),
    play_options.disable_keyboard_controls ? nullptr : std::make_shared<KeyboardHandler>(),
    storage_options, play_options, node_name, node_options)
{}

Player::Player(
  std::unique_ptr<rosbag2_cpp::Reader> reader,
  std::shared_ptr<KeyboardHandler> keyboard_handler,
  const rosbag2_storage::StorageOptions & storage_options,
  const rosbag2_transport::PlayOptions & play_options,
  const std::string & node_name,
  const rclcpp::NodeOptions & node_options)
: rclcpp::Node(
    node_name,
    rclcpp::NodeOptions(node_options).arguments(play_options.topic_remapping_options))
{
  std::vector<reader_storage_options_pair_t> readers_with_options{};
  readers_with_options.emplace_back(std::move(reader), storage_options);
  pimpl_ = std::make_unique<PlayerImpl>(
    this, std::move(readers_with_options), keyboard_handler, play_options);
}

Player::Player(
  std::vector<reader_storage_options_pair_t> && readers_with_options,
  const rosbag2_transport::PlayOptions & play_options,
  const std::string & node_name,
  const rclcpp::NodeOptions & node_options)
: Player(
    std::move(readers_with_options),
    play_options.disable_keyboard_controls ? nullptr : std::make_shared<KeyboardHandler>(),
    play_options,
    node_name,
    node_options)
{}

Player::Player(
  std::vector<reader_storage_options_pair_t> && readers_with_options,
  std::shared_ptr<KeyboardHandler> keyboard_handler,
  const rosbag2_transport::PlayOptions & play_options,
  const std::string & node_name,
  const rclcpp::NodeOptions & node_options)
: rclcpp::Node(
    node_name,
    rclcpp::NodeOptions(node_options).arguments(play_options.topic_remapping_options)),
  pimpl_(std::make_unique<PlayerImpl>(
      this, std::move(readers_with_options), std::move(keyboard_handler), play_options))
{}

Player::~Player() = default;


bool Player::play()
{
  return pimpl_->play();
}

bool Player::wait_for_playback_to_finish(std::chrono::duration<double> timeout)
{
  return pimpl_->wait_for_playback_to_finish(timeout);
}

void Player::stop()
{
  pimpl_->stop();
}

void Player::pause()
{
  pimpl_->pause();
}

void Player::resume()
{
  pimpl_->resume();
}

void Player::toggle_paused()
{
  pimpl_->toggle_paused();
}

bool Player::is_paused() const
{
  return pimpl_->is_paused();
}

rcutils_time_point_value_t Player::get_starting_time() const
{
  return pimpl_->get_starting_time();
}

rcutils_duration_value_t Player::get_playback_duration() const
{
  return pimpl_->get_playback_duration();
}

double Player::get_rate() const
{
  return pimpl_->get_rate();
}

bool Player::set_rate(double rate)
{
  return pimpl_->set_rate(rate);
}

bool Player::play_next()
{
  return pimpl_->play_next();
}

size_t Player::burst(const size_t num_messages)
{
  return pimpl_->burst(num_messages);
}

void Player::seek(rcutils_time_point_value_t time_point)
{
  pimpl_->seek(time_point);
}

Player::callback_handle_t Player::add_on_play_message_pre_callback(
  const play_msg_callback_t & callback)
{
  return pimpl_->add_on_play_message_pre_callback(callback);
}

Player::callback_handle_t Player::add_on_play_message_post_callback(
  const play_msg_callback_t & callback)
{
  return pimpl_->add_on_play_message_post_callback(callback);
}

void Player::delete_on_play_message_callback(const Player::callback_handle_t & handle)
{
  pimpl_->delete_on_play_message_callback(handle);
}

bool Player::wait_for_sent_service_requests_to_finish(
  const std::string & service_name, std::chrono::duration<double> timeout)
{
  return pimpl_->wait_for_sent_service_requests_to_finish(service_name, timeout);
}

std::unordered_map<std::string, std::shared_ptr<rclcpp::GenericPublisher>> Player::get_publishers()
{
  return pimpl_->get_publishers();
}

std::unordered_map<std::string,
  std::shared_ptr<rclcpp::GenericClient>> Player::get_service_clients()
{
  return pimpl_->get_service_clients();
}

rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr Player::get_clock_publisher()
{
  return pimpl_->get_clock_publisher();
}

bool Player::wait_for_playback_to_start(std::chrono::duration<double> timeout)
{
  return pimpl_->wait_for_playback_to_start(timeout);
}

size_t Player::get_number_of_registered_on_play_msg_pre_callbacks()
{
  return pimpl_->get_number_of_registered_on_play_msg_pre_callbacks();
}

size_t Player::get_number_of_registered_on_play_msg_post_callbacks()
{
  return pimpl_->get_number_of_registered_on_play_msg_post_callbacks();
}

const rosbag2_storage::StorageOptions & Player::get_storage_options()
{
  return pimpl_->get_storage_options();
}

std::vector<rosbag2_storage::StorageOptions> Player::get_all_storage_options()
{
  return pimpl_->get_all_storage_options();
}

const rosbag2_transport::PlayOptions & Player::get_play_options()
{
  return pimpl_->get_play_options();
}

}  // namespace rosbag2_transport

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be
// discoverable when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rosbag2_transport::Player)
