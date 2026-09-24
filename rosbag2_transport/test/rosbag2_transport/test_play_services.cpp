// Copyright 2021 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

#include <gmock/gmock.h>

#include <memory>
#include <string>
#include <vector>
#include <utility>
#include <condition_variable>
#include <mutex>

#include "rclcpp/client.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/utilities.hpp"
#include "mock_player.hpp"
#include "rosbag2_interfaces/srv/is_paused.hpp"
#include "rosbag2_interfaces/srv/pause.hpp"
#include "rosbag2_interfaces/srv/resume.hpp"
#include "rosbag2_interfaces/srv/stop.hpp"
#include "rosbag2_interfaces/srv/play.hpp"
#include "rosbag2_interfaces/srv/toggle_paused.hpp"
#include "rosbag2_test_common/wait_for.hpp"
#include "rosbag2_transport/player.hpp"
#include "test_msgs/msg/basic_types.hpp"
#include "test_msgs/message_fixtures.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"

#include "rosbag2_play_test_fixture.hpp"

#if !defined(__PRETTY_FUNCTION__) && !defined(__GNUC__)
#define __PRETTY_FUNCTION__ __FUNCSIG__
#endif

using namespace ::testing;  // NOLINT
using namespace std::chrono_literals;  // NOLINT

class PlaySrvsTest : public RosBag2PlayTestFixture
{
public:
  using Pause = rosbag2_interfaces::srv::Pause;
  using Resume = rosbag2_interfaces::srv::Resume;
  using TogglePaused = rosbag2_interfaces::srv::TogglePaused;
  using IsPaused = rosbag2_interfaces::srv::IsPaused;
  using GetRate = rosbag2_interfaces::srv::GetRate;
  using SetRate = rosbag2_interfaces::srv::SetRate;
  using PlayNext = rosbag2_interfaces::srv::PlayNext;
  using Stop = rosbag2_interfaces::srv::Stop;
  using Play = rosbag2_interfaces::srv::Play;

  explicit PlaySrvsTest(bool use_sim_time = false)
  : RosBag2PlayTestFixture(),
    client_node_(std::make_shared<rclcpp::Node>("test_play_client")),
    use_sim_time_(use_sim_time)
  {}

  ~PlaySrvsTest() override
  {
    exec_.cancel();
    rclcpp::shutdown();
    if (spin_thread_.joinable()) {spin_thread_.join();}
  }

  /// Use SetUp instead of ctor because we want to ASSERT some preconditions for the tests
  void SetUp() override
  {
    setup_player();

    if (use_sim_time_) {
      player_->set_parameter(rclcpp::Parameter("use_sim_time", true));
    }

    const std::string ns = "/" + player_name_;
    cli_pause_ = client_node_->create_client<Pause>(ns + "/pause");
    cli_resume_ = client_node_->create_client<Resume>(ns + "/resume");
    cli_toggle_paused_ = client_node_->create_client<TogglePaused>(ns + "/toggle_paused");
    cli_is_paused_ = client_node_->create_client<IsPaused>(ns + "/is_paused");
    cli_get_rate_ = client_node_->create_client<GetRate>(ns + "/get_rate");
    cli_set_rate_ = client_node_->create_client<SetRate>(ns + "/set_rate");
    cli_play_next_ = client_node_->create_client<PlayNext>(ns + "/play_next");
    cli_stop_ = client_node_->create_client<Stop>(ns + "/stop");
    cli_play_ = client_node_->create_client<Play>(ns + "/play");
    topic_sub_ = client_node_->create_subscription<test_msgs::msg::BasicTypes>(
      test_topic_, 10,
      std::bind(&PlaySrvsTest::topic_callback, this, std::placeholders::_1));

    exec_.add_node(player_);
    exec_.add_node(client_node_);
    spin_thread_ = std::thread(
      [this]() {
        exec_.spin();
      });

    if (use_sim_time_) {
      clock_pub_ = client_node_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
      current_sim_time_ = rclcpp::Time(1, 0, RCL_ROS_TIME);
      publish_clock(current_sim_time_);
      ASSERT_TRUE(
        rosbag2_test_common::wait_until_condition(
          [this]() {return player_->get_clock()->started();},
          std::chrono::seconds(5)));
      EXPECT_EQ(player_->get_clock()->get_clock_type(), RCL_ROS_TIME);
      ASSERT_TRUE(player_->get_clock()->ros_time_is_active());
    }

    // Wait for the executor to start spinning in the newly spawned thread to avoid race conditions
    if (!wait_until_condition([this]() {return exec_.is_spinning();}, std::chrono::seconds(5))) {
      std::cerr << "Failed to start spinning nodes: '" <<
        player_->get_name() << ", " << client_node_->get_name() << "'" << std::endl;
      throw std::runtime_error("Failed to start spinning nodes");
    }

    // Make sure all expected services are present before starting any test
    ASSERT_TRUE(cli_resume_->wait_for_service(service_wait_timeout_));
    ASSERT_TRUE(cli_pause_->wait_for_service(service_wait_timeout_));
    ASSERT_TRUE(cli_is_paused_->wait_for_service(service_wait_timeout_));
    ASSERT_TRUE(cli_toggle_paused_->wait_for_service(service_wait_timeout_));
    ASSERT_TRUE(cli_get_rate_->wait_for_service(service_wait_timeout_));
    ASSERT_TRUE(cli_set_rate_->wait_for_service(service_wait_timeout_));
    ASSERT_TRUE(cli_play_next_->wait_for_service(service_wait_timeout_));
    ASSERT_TRUE(cli_stop_->wait_for_service(service_wait_timeout_));
    ASSERT_TRUE(cli_play_->wait_for_service(service_wait_timeout_));
  }

  void TearDown() override
  {
    // Gracefully stop player before calling rclcpp::shutdown() in destructor
    player_->stop();
  }

  /// Call a service client, and expect it to successfully return within a reasonable timeout
  template<typename Srv>
  typename Srv::Response::SharedPtr successful_call(
    typename rclcpp::Client<Srv>::SharedPtr cli,
    typename Srv::Request::SharedPtr request,
    const char * function_name,
    int line_number)
  {
    auto future = cli->async_send_request(request);
    EXPECT_TRUE(future.valid());
    EXPECT_EQ(future.wait_for(service_call_timeout_), std::future_status::ready) <<
      function_name << ", line : " << line_number;
    // std::cout << function_name << ", line : " << line_number << std::endl;
    auto result = std::make_shared<typename Srv::Response>();
    EXPECT_NO_THROW({result = future.get();});
    EXPECT_TRUE(result);
    return result;
  }

  template<typename Srv>
  typename Srv::Response::SharedPtr successful_call(
    typename rclcpp::Client<Srv>::SharedPtr cli, const char * function_name, int line_number)
  {
    auto request = std::make_shared<typename Srv::Request>();
    return successful_call<Srv>(cli, request, function_name, line_number);
  }

  bool is_paused(const char * function_name, int line_number)
  {
    auto result = successful_call<IsPaused>(cli_is_paused_, function_name, line_number);
    return result->paused;
  }

#define service_call_play_next() \
  successful_call<PlayNext>(cli_play_next_, __PRETTY_FUNCTION__, __LINE__)

#define service_call_stop() \
  successful_call<Stop>(cli_stop_, __PRETTY_FUNCTION__, __LINE__)

#define service_call_set_rate(set_request) \
  successful_call<SetRate>(cli_set_rate_, set_request, __PRETTY_FUNCTION__, __LINE__)

#define service_call_get_rate() \
  successful_call<GetRate>(cli_get_rate_, __PRETTY_FUNCTION__, __LINE__)

#define service_call_toggle_paused() \
  successful_call<TogglePaused>(cli_toggle_paused_, __PRETTY_FUNCTION__, __LINE__);

#define service_call_resume() \
  successful_call<Resume>(cli_resume_, __PRETTY_FUNCTION__, __LINE__);

#define service_call_pause() \
  successful_call<Pause>(cli_pause_, __PRETTY_FUNCTION__, __LINE__);

#define service_call_is_paused() is_paused(__PRETTY_FUNCTION__, __LINE__)

  /// EXPECT to receive (or not receive) any messages for a period
  void expect_messages(bool messages_should_arrive, bool reset_message_counter = true)
  {
    // Not too worried about the exact timing in this test, give a lot of leeway
    const auto condition_clear_time = std::chrono::milliseconds(ms_between_msgs_ * 100);
    std::unique_lock<std::mutex> lock(got_msg_mutex_);
    if (reset_message_counter) {
      message_counter_ = 0;
    }
    if (!messages_should_arrive) {
      EXPECT_EQ(
        got_msg_.wait_for(
          lock, condition_clear_time,
          [this]() {return message_counter_ > 0;}), false);
    } else {
      EXPECT_EQ(
        got_msg_.wait_for(
          lock, condition_clear_time,
          [this]() {return message_counter_ > 0;}), true);
    }
  }

private:
  /// Create a player with some messages to play back, and start it on loop
  void setup_player()
  {
    rosbag2_storage::StorageOptions storage_options;
    rosbag2_transport::PlayOptions play_options;
    play_options.loop = true;

    auto message = get_messages_basic_types()[0];
    message->int32_value = 42;

    auto topic_types = std::vector<rosbag2_storage::TopicMetadata>{
      {1u, test_topic_, "test_msgs/BasicTypes", "", {}, ""},
    };
    std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages;
    for (size_t i = 0; i < num_msgs_to_publish_; i++) {
      messages.push_back(serialize_test_message(test_topic_, i * ms_between_msgs_, message));
    }

    auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
    prepared_mock_reader->prepare(messages, topic_types);
    auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));

    if (use_sim_time_) {
      rclcpp::NodeOptions node_options = rclcpp::NodeOptions()
        .start_parameter_event_publisher(false)
        .append_parameter_override("use_sim_time", true)
        .enable_rosout(false);

      player_ = std::make_shared<MockPlayer>(
        std::move(reader), storage_options, play_options, player_name_, node_options);
    } else {
      player_ = std::make_shared<MockPlayer>(
        std::move(reader), storage_options, play_options, player_name_);
    }

    player_->pause();  // Start playing in pause mode. Require for play_next test. For all other
    // tests we will resume playback via explicit call to start_playback().
    player_->play();
    player_->wait_for_playback_to_start();
  }

  void topic_callback(std::shared_ptr<const test_msgs::msg::BasicTypes>/* msg */)
  {
    {
      std::lock_guard<std::mutex> lk(got_msg_mutex_);
      message_counter_++;
    }
    got_msg_.notify_all();
  }

protected:
/// \brief  Wait for paused == false to know that playback has begun
  void start_playback()
  {
    player_->resume();
    bool is_playing = false;
    for (size_t retry = 0; retry < 3 && rclcpp::ok(); retry++) {
      if (!player_->is_paused()) {
        is_playing = true;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    ASSERT_TRUE(is_playing);
  }

public:
  // Basic configuration
  const std::string player_name_ = "rosbag2_player_for_test_srvs";
  const std::chrono::seconds service_wait_timeout_ {6};
  const std::chrono::seconds service_call_timeout_ {6};
  const std::string test_topic_ = "/player_srvs_test_topic";
  // publishing at 50hz
  const size_t ms_between_msgs_ = 20;
  const size_t num_msgs_to_publish_ = 200;

  // Orchestration
  std::thread spin_thread_;
  rclcpp::executors::SingleThreadedExecutor exec_;
  std::shared_ptr<MockPlayer> player_;

  // Service clients
  rclcpp::Node::SharedPtr client_node_;
  rclcpp::Client<Pause>::SharedPtr cli_pause_;
  rclcpp::Client<Resume>::SharedPtr cli_resume_;
  rclcpp::Client<TogglePaused>::SharedPtr cli_toggle_paused_;
  rclcpp::Client<IsPaused>::SharedPtr cli_is_paused_;
  rclcpp::Client<GetRate>::SharedPtr cli_get_rate_;
  rclcpp::Client<SetRate>::SharedPtr cli_set_rate_;
  rclcpp::Client<PlayNext>::SharedPtr cli_play_next_;
  rclcpp::Client<Stop>::SharedPtr cli_stop_;
  rclcpp::Client<Play>::SharedPtr cli_play_;

  // Mechanism to check on playback status
  rclcpp::Subscription<test_msgs::msg::BasicTypes>::SharedPtr topic_sub_;
  std::mutex got_msg_mutex_;
  std::condition_variable got_msg_;
  size_t message_counter_ = 0;

  bool use_sim_time_ = false;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::Time current_sim_time_{0, 0, RCL_ROS_TIME};

  void publish_clock(const rclcpp::Time & time)
  {
    if (!use_sim_time_) {
      return;
    }
    rosgraph_msgs::msg::Clock msg;
    msg.clock = time;
    clock_pub_->publish(msg);
  }

  void advance_sim_time(const rclcpp::Duration & delta)
  {
    if (!use_sim_time_) {
      return;
    }
    current_sim_time_ = current_sim_time_ + delta;
    publish_clock(current_sim_time_);
  }
};

class PlaySrvsSimTimeTest : public PlaySrvsTest
{
protected:
  PlaySrvsSimTimeTest()
  : PlaySrvsTest(true /*use_sim_time*/)
  {}
};

TEST_F(PlaySrvsTest, pause_resume)
{
  start_playback();
  // No matter how many times we call pause, it's paused
  for (size_t i = 0; i < 3; i++) {
    service_call_pause();
    ASSERT_TRUE(player_->is_paused());
  }
  expect_messages(false);

  // No matter how many times we call resume, it's resumed
  for (size_t i = 0; i < 3; i++) {
    service_call_resume();
    ASSERT_FALSE(player_->is_paused());
  }
  expect_messages(true);

  // Let's do pause again to make sure back-and-forth works
  for (size_t i = 0; i < 3; i++) {
    service_call_pause();
    ASSERT_TRUE(player_->is_paused());
  }
  expect_messages(false);

  // resume to make sure we exit
  for (size_t i = 0; i < 3; i++) {
    service_call_resume();
    ASSERT_FALSE(player_->is_paused());
  }
}

TEST_F(PlaySrvsTest, toggle_paused)
{
  start_playback();
  service_call_toggle_paused();
  ASSERT_TRUE(player_->is_paused());
  expect_messages(false);

  service_call_toggle_paused();
  ASSERT_FALSE(player_->is_paused());
  expect_messages(true);

  service_call_toggle_paused();
  ASSERT_TRUE(player_->is_paused());
  expect_messages(false);

  service_call_toggle_paused();
  ASSERT_FALSE(player_->is_paused());
  expect_messages(true);
}

TEST_F(PlaySrvsTest, is_paused)
{
  start_playback();
  player_->toggle_paused();
  ASSERT_TRUE(service_call_is_paused());
  expect_messages(false);

  player_->toggle_paused();
  ASSERT_FALSE(service_call_is_paused());
  expect_messages(true);

  player_->toggle_paused();
  ASSERT_TRUE(service_call_is_paused());
  expect_messages(false);

  player_->toggle_paused();
  ASSERT_FALSE(service_call_is_paused());
  expect_messages(true);
}

TEST_F(PlaySrvsTest, set_rate_good_values)
{
  start_playback();
  auto set_request = std::make_shared<SetRate::Request>();
  SetRate::Response::SharedPtr set_response;
  GetRate::Response::SharedPtr get_response;

  set_request->rate = 2.0;
  set_response = service_call_set_rate(set_request);
  ASSERT_TRUE(set_response->success);
  get_response = service_call_get_rate();
  ASSERT_EQ(get_response->rate, 2.0);

  set_request->rate = 0.5;
  set_response = service_call_set_rate(set_request);
  ASSERT_TRUE(set_response->success);
  get_response = service_call_get_rate();
  ASSERT_EQ(get_response->rate, 0.5);
}

TEST_F(PlaySrvsTest, set_rate_bad_values)
{
  start_playback();
  auto set_request = std::make_shared<SetRate::Request>();
  SetRate::Response::SharedPtr set_response;

  set_request->rate = 0.0;
  set_response = service_call_set_rate(set_request);
  ASSERT_FALSE(set_response->success);

  set_request->rate = -1.0;
  set_response = service_call_set_rate(set_request);
  ASSERT_FALSE(set_response->success);
}

TEST_F(PlaySrvsTest, play_next) {
  ASSERT_TRUE(player_->is_paused());
  PlayNext::Response::SharedPtr play_next_response;
  // Check that we will be able to play all messages via play_next
  for (size_t i = 0; i < num_msgs_to_publish_; i++) {
    {
      std::lock_guard<std::mutex> lk(got_msg_mutex_);
      message_counter_ = 0;
    }
    play_next_response = service_call_play_next();
    ASSERT_TRUE(play_next_response->success);
    expect_messages(true, false);
  }

  // Check that when no more messages to play, play_next will return false
  {
    std::lock_guard<std::mutex> lk(got_msg_mutex_);
    message_counter_ = 0;
  }
  play_next_response = service_call_play_next();
  ASSERT_FALSE(play_next_response->success);
  expect_messages(false, false);

  // Check that play_next will return false when player not in pause mode.
  start_playback();
  ASSERT_FALSE(player_->is_paused());
  play_next_response = service_call_play_next();
  ASSERT_FALSE(play_next_response->success);
  expect_messages(true);
}

TEST_F(PlaySrvsTest, stop_in_pause) {
  ASSERT_TRUE(player_->is_paused());
  // Make sure that player reached out main play loop
  player_->wait_for_playback_to_start();
  Stop::Response::SharedPtr stop_response = service_call_stop();
  ASSERT_EQ(stop_response->return_code, 0);
  // playback shall successfully finish after "Stop" without rclcpp::shutdown()
  player_->wait_for_playback_to_finish();
  expect_messages(false);
}

TEST_F(PlaySrvsTest, stop_in_active_play) {
  auto num_calls = 0;
  std::mutex calls_counter_update_mutex;
  std::condition_variable calls_counter_update_cv;
  ASSERT_TRUE(player_->is_paused());

  const auto callback = [&](std::shared_ptr<rosbag2_storage::SerializedBagMessage>) {
      std::unique_lock<std::mutex> lk{calls_counter_update_mutex};
      ++num_calls;
      lk.unlock();
      calls_counter_update_cv.notify_one();
      std::this_thread::sleep_for(50ms);
    };
  const auto pre_callback_handle = player_->add_on_play_message_pre_callback(callback);
  ASSERT_NE(pre_callback_handle, rosbag2_transport::Player::invalid_callback_handle);

  player_->wait_for_playback_to_start(10s);
  ASSERT_TRUE(player_->is_paused());

  // Lock calls_counter_update_mutex to avoid missing the first message published after resume
  std::unique_lock<std::mutex> lk{calls_counter_update_mutex};
  player_->resume();
  ASSERT_FALSE(player_->is_paused());
  // Wait until first message is going to be published in active playback mode
  ASSERT_TRUE(calls_counter_update_cv.wait_for(lk, 2s, [&] {return num_calls == 1;}));
  // Unlock calls_counter_update_mutex before calling stop() to avoid deadlock in the callback
  // if we happened to call stop() after the next message started being processed in the callback
  lk.unlock();
  // Now call stop() while player is in active playback mode
  Stop::Response::SharedPtr stop_response = service_call_stop();
  ASSERT_EQ(stop_response->return_code, 0);
  // playback shall successfully finish after "Stop" without rclcpp::shutdown()
  player_->wait_for_playback_to_finish(10s);
  // The second stop() call after playback has already stopped shall return return_code = 1
  stop_response = service_call_stop();
  ASSERT_EQ(stop_response->return_code, 1);
}

TEST_F(PlaySrvsSimTimeTest, resume_can_be_scheduled_in_future_sim_time)
{
  ASSERT_TRUE(player_->is_paused());
  expect_messages(false);

  // Schedule a resume in the future
  auto resume_request = std::make_shared<Resume::Request>();
  auto target_time = current_sim_time_ + rclcpp::Duration(std::chrono::milliseconds(200));
  resume_request->resume_time = target_time;

  auto resume_response = successful_call<Resume>(
    cli_resume_, resume_request, __PRETTY_FUNCTION__, __LINE__);
  ASSERT_TRUE(resume_response);

  EXPECT_TRUE(player_->is_paused());
  expect_messages(false);

  auto delta = target_time - current_sim_time_;
  advance_sim_time(delta);

  ASSERT_TRUE(
    rosbag2_test_common::wait_until_condition(
      [this]() {return !player_->is_paused();},
      std::chrono::seconds(5))
  ) << "Timed out waiting for scheduled resume to execute.";

  expect_messages(true);
}

TEST_F(PlaySrvsSimTimeTest, play_can_be_scheduled_in_future_sim_time)
{
  ASSERT_TRUE(player_->is_paused());
  player_->resume();
  expect_messages(true);
  player_->stop();  // Stop playback to prepare for scheduled play test
  expect_messages(false);

  auto play_request = std::make_shared<Play::Request>();
  auto target_time = current_sim_time_ + rclcpp::Duration(2, 0);
  play_request->start_time = target_time;

  play_request->start_offset = rclcpp::Time(0, 0);
  play_request->playback_duration = rclcpp::Duration(-1, 0);
  play_request->playback_until_timestamp.sec = -1;
  play_request->playback_until_timestamp.nanosec = 0;

  auto play_response =
    successful_call<Play>(cli_play_, play_request, __PRETTY_FUNCTION__, __LINE__);
  ASSERT_TRUE(play_response);
  EXPECT_EQ(play_response->return_code,
            rosbag2_interfaces::srv::Play::Response::RETURN_CODE_SUCCESS);
  EXPECT_TRUE(play_response->error_string.empty());

  expect_messages(false);

  // Advance time to trigger the scheduled playback
  auto delta = target_time - current_sim_time_;
  advance_sim_time(delta);

  ASSERT_TRUE(
    rosbag2_test_common::wait_until_condition(
      [this]()
      {
        std::unique_lock<std::mutex> lock(got_msg_mutex_);
        return message_counter_ > 0;
      },
      std::chrono::seconds(5))
  ) << "Timed out waiting for scheduled play to execute.";

  expect_messages(true);

  // Test that calling play with default start_time (0) returns false when already in playback
  play_request->start_time = rclcpp::Time(0, 0);
  play_response = successful_call<Play>(cli_play_, play_request, __PRETTY_FUNCTION__, __LINE__);
  ASSERT_TRUE(play_response);
  EXPECT_EQ(play_response->return_code,
            rosbag2_interfaces::srv::Play::Response::RETURN_CODE_ALREADY_RUNNING);
  EXPECT_FALSE(play_response->error_string.empty());

  expect_messages(true);

  // Test that scheduling play in the future while already playing works
  target_time = current_sim_time_ + rclcpp::Duration(2, 0);
  play_request->start_time = target_time;
  play_response = successful_call<Play>(cli_play_, play_request, __PRETTY_FUNCTION__, __LINE__);
  ASSERT_TRUE(play_response);
  EXPECT_EQ(play_response->return_code,
            rosbag2_interfaces::srv::Play::Response::RETURN_CODE_SUCCESS);
  EXPECT_TRUE(play_response->error_string.empty());

  expect_messages(true);  // Messages continue from current playback

  player_->stop();   // Stop playback to prepare for the next already scheduled playback to start
  expect_messages(false);

  // Advance time to trigger the scheduled playback
  delta = target_time - current_sim_time_;
  advance_sim_time(delta);

  ASSERT_TRUE(
    rosbag2_test_common::wait_until_condition(
      [this]()
      {
        std::unique_lock<std::mutex> lock(got_msg_mutex_);
        return message_counter_ > 0;
      }, std::chrono::seconds(5))
  ) << "Timed out waiting for scheduled play to execute.";

  expect_messages(true);
}
