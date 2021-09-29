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

#include "rclcpp/rclcpp.hpp"

#include "rosbag2_interfaces/srv/is_paused.hpp"
#include "rosbag2_interfaces/srv/pause.hpp"
#include "rosbag2_interfaces/srv/resume.hpp"
#include "rosbag2_interfaces/srv/toggle_paused.hpp"
#include "rosbag2_transport/player.hpp"
#include "test_msgs/msg/basic_types.hpp"
#include "test_msgs/message_fixtures.hpp"

#include "rosbag2_play_test_fixture.hpp"

using namespace ::testing;  // NOLINT

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

  PlaySrvsTest()
  : RosBag2PlayTestFixture(),
    client_node_(std::make_shared<rclcpp::Node>("test_play_client"))
  {}

  ~PlaySrvsTest() override
  {
    exec_.cancel();
    rclcpp::shutdown();
    spin_thread_.join();
    play_thread_.join();
  }

  /// Use SetUp instead of ctor because we want to ASSERT some preconditions for the tests
  void SetUp() override
  {
    setup_player();

    const std::string ns = "/" + player_name_;
    cli_pause_ = client_node_->create_client<Pause>(ns + "/pause");
    cli_resume_ = client_node_->create_client<Resume>(ns + "/resume");
    cli_toggle_paused_ = client_node_->create_client<TogglePaused>(ns + "/toggle_paused");
    cli_is_paused_ = client_node_->create_client<IsPaused>(ns + "/is_paused");
    cli_get_rate_ = client_node_->create_client<GetRate>(ns + "/get_rate");
    cli_set_rate_ = client_node_->create_client<SetRate>(ns + "/set_rate");
    cli_play_next_ = client_node_->create_client<PlayNext>(ns + "/play_next");
    topic_sub_ = client_node_->create_subscription<test_msgs::msg::BasicTypes>(
      test_topic_, 10,
      std::bind(&PlaySrvsTest::topic_callback, this, std::placeholders::_1));

    exec_.add_node(player_);
    exec_.add_node(client_node_);
    spin_thread_ = std::thread(
      [this]() {
        exec_.spin();
      });

    // Make sure all expected services are present before starting any test
    ASSERT_TRUE(cli_resume_->wait_for_service(service_wait_timeout_));
    ASSERT_TRUE(cli_pause_->wait_for_service(service_wait_timeout_));
    ASSERT_TRUE(cli_is_paused_->wait_for_service(service_wait_timeout_));
    ASSERT_TRUE(cli_toggle_paused_->wait_for_service(service_wait_timeout_));
    ASSERT_TRUE(cli_get_rate_->wait_for_service(service_wait_timeout_));
    ASSERT_TRUE(cli_set_rate_->wait_for_service(service_wait_timeout_));
    ASSERT_TRUE(cli_play_next_->wait_for_service(service_wait_timeout_));
  }

  /// Call a service client, and expect it to successfully return within a reasonable timeout
  template<typename Srv>
  typename Srv::Response::SharedPtr successful_call(
    typename rclcpp::Client<Srv>::SharedPtr cli,
    typename Srv::Request::SharedPtr request)
  {
    auto future = cli->async_send_request(request);
    EXPECT_EQ(future.wait_for(service_call_timeout_), std::future_status::ready);
    EXPECT_TRUE(future.valid());
    auto result = std::make_shared<typename Srv::Response>();
    EXPECT_NO_THROW({result = future.get();});
    EXPECT_TRUE(result);
    return result;
  }

  template<typename Srv>
  typename Srv::Response::SharedPtr successful_call(typename rclcpp::Client<Srv>::SharedPtr cli)
  {
    auto request = std::make_shared<typename Srv::Request>();
    return successful_call<Srv>(cli, request);
  }

  bool is_paused()
  {
    auto result = successful_call<IsPaused>(cli_is_paused_);
    return result->paused;
  }

  /// EXPECT to receive (or not receive) any messages for a period
  void expect_messages(bool messages_should_arrive, bool reset_message_counter = true)
  {
    // Not too worried about the exact timing in this test, give a lot of leeway
    const auto condition_clear_time = std::chrono::milliseconds(ms_between_msgs_ * 10);
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
      {test_topic_, "test_msgs/BasicTypes", "", ""},
    };
    std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages;
    for (size_t i = 0; i < num_msgs_to_publish_; i++) {
      messages.push_back(serialize_test_message(test_topic_, i * ms_between_msgs_, message));
    }

    auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
    prepared_mock_reader->prepare(messages, topic_types);
    auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));
    player_ = std::make_shared<rosbag2_transport::Player>(
      std::move(reader), storage_options, play_options, player_name_);
    player_->pause();  // Start playing in pause mode. Require for play_next test. For all other
    // tests we will resume playback via explicit call to start_playback().
    play_thread_ = std::thread(
      [this]() {
        player_->play();
      });
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
  const std::chrono::seconds service_wait_timeout_ {2};
  const std::chrono::seconds service_call_timeout_ {1};
  const std::string test_topic_ = "/player_srvs_test_topic";
  // publishing at 50hz
  const size_t ms_between_msgs_ = 20;
  const size_t num_msgs_to_publish_ = 200;

  // Orchestration
  std::thread spin_thread_;
  std::thread play_thread_;
  rclcpp::executors::SingleThreadedExecutor exec_;
  std::shared_ptr<rosbag2_transport::Player> player_;

  // Service clients
  rclcpp::Node::SharedPtr client_node_;
  rclcpp::Client<Pause>::SharedPtr cli_pause_;
  rclcpp::Client<Resume>::SharedPtr cli_resume_;
  rclcpp::Client<TogglePaused>::SharedPtr cli_toggle_paused_;
  rclcpp::Client<IsPaused>::SharedPtr cli_is_paused_;
  rclcpp::Client<GetRate>::SharedPtr cli_get_rate_;
  rclcpp::Client<SetRate>::SharedPtr cli_set_rate_;
  rclcpp::Client<PlayNext>::SharedPtr cli_play_next_;

  // Mechanism to check on playback status
  rclcpp::Subscription<test_msgs::msg::BasicTypes>::SharedPtr topic_sub_;
  std::mutex got_msg_mutex_;
  std::condition_variable got_msg_;
  size_t message_counter_ = 0;
};

TEST_F(PlaySrvsTest, pause_resume)
{
  start_playback();
  // No matter how many times we call pause, it's paused
  for (size_t i = 0; i < 3; i++) {
    successful_call<Pause>(cli_pause_);
    ASSERT_TRUE(is_paused());
  }
  expect_messages(false);

  // No matter how many times we call resume, it's resumed
  for (size_t i = 0; i < 3; i++) {
    successful_call<Resume>(cli_resume_);
    ASSERT_FALSE(is_paused());
  }
  expect_messages(true);

  // Let's do pause again to make sure back-and-forth works
  for (size_t i = 0; i < 3; i++) {
    successful_call<Pause>(cli_pause_);
    ASSERT_TRUE(is_paused());
  }
  expect_messages(false);

  // resume to make sure we exit
  for (size_t i = 0; i < 3; i++) {
    successful_call<Resume>(cli_resume_);
    ASSERT_FALSE(is_paused());
  }
}

TEST_F(PlaySrvsTest, toggle_paused)
{
  start_playback();
  successful_call<TogglePaused>(cli_toggle_paused_);
  ASSERT_TRUE(is_paused());
  expect_messages(false);

  successful_call<TogglePaused>(cli_toggle_paused_);
  ASSERT_FALSE(is_paused());
  expect_messages(true);

  successful_call<TogglePaused>(cli_toggle_paused_);
  ASSERT_TRUE(is_paused());
  expect_messages(false);

  successful_call<TogglePaused>(cli_toggle_paused_);
  ASSERT_FALSE(is_paused());
  expect_messages(true);
}

TEST_F(PlaySrvsTest, set_rate_good_values)
{
  start_playback();
  auto set_request = std::make_shared<SetRate::Request>();
  SetRate::Response::SharedPtr set_response;
  GetRate::Response::SharedPtr get_response;

  set_request->rate = 2.0;
  set_response = successful_call<SetRate>(cli_set_rate_, set_request);
  ASSERT_TRUE(set_response->success);
  get_response = successful_call<GetRate>(cli_get_rate_);
  ASSERT_EQ(get_response->rate, 2.0);

  set_request->rate = 0.5;
  set_response = successful_call<SetRate>(cli_set_rate_, set_request);
  ASSERT_TRUE(set_response->success);
  get_response = successful_call<GetRate>(cli_get_rate_);
  ASSERT_EQ(get_response->rate, 0.5);
}

TEST_F(PlaySrvsTest, set_rate_bad_values)
{
  start_playback();
  auto set_request = std::make_shared<SetRate::Request>();
  SetRate::Response::SharedPtr set_response;

  set_request->rate = 0.0;
  set_response = successful_call<SetRate>(cli_set_rate_, set_request);
  ASSERT_FALSE(set_response->success);

  set_request->rate = -1.0;
  set_response = successful_call<SetRate>(cli_set_rate_, set_request);
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
    play_next_response = successful_call<PlayNext>(cli_play_next_);
    ASSERT_TRUE(play_next_response->success);
    expect_messages(true, false);
  }

  // Check that when no more messages to play, play_next will return false
  {
    std::lock_guard<std::mutex> lk(got_msg_mutex_);
    message_counter_ = 0;
  }
  play_next_response = successful_call<PlayNext>(cli_play_next_);
  ASSERT_FALSE(play_next_response->success);
  expect_messages(false, false);

  // Check that play_next will return false when player not in pause mode.
  start_playback();
  ASSERT_FALSE(player_->is_paused());
  play_next_response = successful_call<PlayNext>(cli_play_next_);
  ASSERT_FALSE(play_next_response->success);
  expect_messages(true);
}
