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

    // Wait for paused == false to know that playback has begun
    bool is_playing = false;
    IsPaused::Response::SharedPtr is_paused_result;
    for (size_t retry = 0; retry < 3 && rclcpp::ok(); retry++) {
      is_paused_result = successful_call<IsPaused>(cli_is_paused_);
      if (is_paused_result->paused == false) {
        is_playing = true;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    ASSERT_TRUE(is_playing);
  }

  /// Call a service client, and expect it to successfully return within a reasonable timeout
  template<typename T>
  typename T::Response::SharedPtr successful_call(typename rclcpp::Client<T>::SharedPtr cli)
  {
    auto request = std::make_shared<typename T::Request>();
    auto future = cli->async_send_request(request);
    EXPECT_EQ(future.wait_for(service_call_timeout_), std::future_status::ready);
    auto result = future.get();
    EXPECT_TRUE(result);
    return result;
  }

  bool is_paused()
  {
    auto result = successful_call<IsPaused>(cli_is_paused_);
    return result->paused;
  }

  /// EXPECT to receive (or not receive) any messages for a period
  void expect_message(bool messages_should_arrive)
  {
    // Not too worried about the exact timing in this test, give a lot of leeway
    const auto condition_clear_time = std::chrono::milliseconds(ms_between_msgs_ * 10);
    std::unique_lock<std::mutex> lock(got_msg_mutex_, std::defer_lock);
    if (!messages_should_arrive) {
      // Wait momentarily to let any messages that were published before our pause to get cleared
      std::this_thread::sleep_for(condition_clear_time);
      EXPECT_EQ(got_msg_.wait_for(lock, condition_clear_time), std::cv_status::timeout);
    } else {
      EXPECT_EQ(got_msg_.wait_for(lock, condition_clear_time), std::cv_status::no_timeout);
    }
  }

private:
  /// Create a player with some messages to play back, and start it on loop
  void setup_player()
  {
    rosbag2_storage::StorageOptions storage_options;
    rosbag2_transport::PlayOptions play_options;
    play_options.loop = true;

    const size_t num_msgs_to_publish = 200;
    auto message = get_messages_basic_types()[0];
    message->int32_value = 42;

    auto topic_types = std::vector<rosbag2_storage::TopicMetadata>{
      {test_topic_, "test_msgs/BasicTypes", "", ""},
    };
    std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages;
    for (size_t i = 0; i < num_msgs_to_publish; i++) {
      messages.push_back(serialize_test_message(test_topic_, i * ms_between_msgs_, message));
    }

    auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
    prepared_mock_reader->prepare(messages, topic_types);
    auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));
    player_ = std::make_shared<rosbag2_transport::Player>(
      std::move(reader), storage_options, play_options, player_name_);
    play_thread_ = std::thread(
      [this]() {
        player_->play();
      });
  }

  void topic_callback(const test_msgs::msg::BasicTypes::SharedPtr /* msg */)
  {
    got_msg_.notify_all();
  }

public:
  // Basic configuration
  const std::string player_name_ = "rosbag2_player_for_test_srvs";
  const std::chrono::seconds service_wait_timeout_ {2};
  const std::chrono::seconds service_call_timeout_ {1};
  const std::string test_topic_ = "/player_srvs_test_topic";
  // publishing at 50hz
  const size_t ms_between_msgs_ = 20;

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

  // Mechanism to check on playback status
  rclcpp::Subscription<test_msgs::msg::BasicTypes>::SharedPtr topic_sub_;
  std::mutex got_msg_mutex_;
  std::condition_variable got_msg_;
};

TEST_F(PlaySrvsTest, pause_resume)
{
  // No matter how many times we call pause, it's paused
  for (size_t i = 0; i < 3; i++) {
    successful_call<Pause>(cli_pause_);
    ASSERT_TRUE(is_paused());
  }
  expect_message(false);

  // No matter how many times we call resume, it's resumed
  for (size_t i = 0; i < 3; i++) {
    successful_call<Resume>(cli_resume_);
    ASSERT_FALSE(is_paused());
  }
  expect_message(true);

  // Let's do pause again to make sure back-and-forth works
  for (size_t i = 0; i < 3; i++) {
    successful_call<Pause>(cli_pause_);
    ASSERT_TRUE(is_paused());
  }
  expect_message(false);
}

TEST_F(PlaySrvsTest, toggle_paused)
{
  successful_call<TogglePaused>(cli_toggle_paused_);
  ASSERT_TRUE(is_paused());
  expect_message(false);

  successful_call<TogglePaused>(cli_toggle_paused_);
  ASSERT_FALSE(is_paused());
  expect_message(true);

  successful_call<TogglePaused>(cli_toggle_paused_);
  ASSERT_TRUE(is_paused());
  expect_message(false);

  successful_call<TogglePaused>(cli_toggle_paused_);
  ASSERT_FALSE(is_paused());
  expect_message(true);
}
