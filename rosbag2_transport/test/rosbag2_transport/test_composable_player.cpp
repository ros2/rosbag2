// Copyright 2023, Patrick Roncagliolo and Michael Orlov
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

#include <chrono>
#include <filesystem>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "composition_manager_test_fixture.hpp"
#include "mock_player.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosbag2_test_common/subscription_manager.hpp"
#include "rosbag2_test_common/tested_storage_ids.hpp"
#include "test_msgs/msg/basic_types.hpp"

using namespace std::chrono_literals;  // NOLINT
using namespace ::testing;  // NOLINT

namespace fs = std::filesystem;

class ComposablePlayerTests
  : public ::testing::Test, public WithParamInterface<std::string>
{
public:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }
};

class ComposablePlayerIntegrationTests : public CompositionManagerTestFixture
{
public:
  using srvPlay = rosbag2_interfaces::srv::Play;
  using srvIsPaused = rosbag2_interfaces::srv::IsPaused;
  using srvResume = rosbag2_interfaces::srv::Resume;

  void SetUp() override
  {
    CompositionManagerTestFixture::SetUp();
    sub_ = std::make_shared<rosbag2_test_common::SubscriptionManager>();
    is_paused_client_ = node_->create_client<srvIsPaused>(player_node_name_ + "/is_paused");
    resume_client_ = node_->create_client<srvResume>(player_node_name_ + "/resume");
    play_client_ = node_->create_client<srvPlay>(player_node_name_ + "/play");
  }

  bool player_is_paused_service_request()
  {
    // Wait for service to be ready
    EXPECT_TRUE(is_paused_client_->wait_for_service(std::chrono::seconds(10)));
    auto is_paused_request = std::make_shared<srvIsPaused::Request>();
    auto is_paused_future = is_paused_client_->async_send_request(is_paused_request);
    auto ret = exec_->spin_until_future_complete(is_paused_future, 10s);  // Wait for the result.
    EXPECT_EQ(ret, rclcpp::FutureReturnCode::SUCCESS);
    auto result = std::make_shared<srvIsPaused::Response>();
    EXPECT_NO_THROW({result = is_paused_future.get();});
    EXPECT_TRUE(result);
    return result->paused;
  }

  void player_resume_service_request()
  {
    // Wait for service to be ready
    EXPECT_TRUE(resume_client_->wait_for_service(std::chrono::seconds(10)));
    auto resume_request = std::make_shared<srvResume::Request>();
    auto resume_future = resume_client_->async_send_request(resume_request);
    auto ret = exec_->spin_until_future_complete(resume_future, 10s);  // Wait for the result.
    EXPECT_EQ(ret, rclcpp::FutureReturnCode::SUCCESS);
    auto result = std::make_shared<srvResume::Response>();
    EXPECT_NO_THROW({result = resume_future.get();});
    EXPECT_TRUE(result);
  }

  bool player_play_service_request()
  {
    // Wait for service to be ready
    EXPECT_TRUE(play_client_->wait_for_service(std::chrono::seconds(10)));
    auto play_request = std::make_shared<srvPlay::Request>();
    play_request->start_offset = rclcpp::Time(0, 0);
    play_request->playback_until_timestamp = rclcpp::Time(0, -1);
    play_request->playback_duration = rclcpp::Duration(-1, 0);
    auto play_future = play_client_->async_send_request(play_request);
    auto ret = exec_->spin_until_future_complete(play_future, 10s);  // Wait for the result.
    EXPECT_EQ(ret, rclcpp::FutureReturnCode::SUCCESS);
    auto result = std::make_shared<srvPlay::Response>();
    EXPECT_NO_THROW({result = play_future.get();});
    EXPECT_TRUE(result);
    return result->success;
  }

protected:
  const std::string player_node_name_ = "/rosbag2_player";
  std::shared_ptr<rclcpp::Client<srvIsPaused>> is_paused_client_;
  std::shared_ptr<rclcpp::Client<srvPlay>> play_client_;
  std::shared_ptr<rclcpp::Client<srvResume>> resume_client_;
  std::shared_ptr<rosbag2_test_common::SubscriptionManager> sub_;
  const std::string topic_name_ = "/topic1";
  const size_t num_msgs_in_bag_ = 5;
};

TEST_P(ComposablePlayerTests, player_can_parse_parameters_from_file) {
  // _SRC_RESOURCES_DIR_PATH defined in CMakeLists.txt
  rclcpp::NodeOptions opts;
  opts.arguments(
  {
    "--ros-args",
    "--params-file", _SRC_RESOURCES_DIR_PATH "/player_node_params.yaml"
  });
  opts.append_parameter_override(
    "play.qos_profile_overrides_path",
    _SRC_RESOURCES_DIR_PATH "/qos_profile_overrides.yaml");
  opts.append_parameter_override("storage.storage_id", GetParam());
  const std::string uri_str = (fs::path(
      _SRC_RESOURCES_DIR_PATH) / GetParam() / "test_bag_for_seek").generic_string();
  opts.append_parameter_override("storage.uri", uri_str);

  auto player = std::make_shared<MockPlayer>("player_params_node", opts);
  auto play_options = player->get_play_options();
  auto storage_options = player->get_all_storage_options();

  EXPECT_EQ(play_options.read_ahead_queue_size, 3);
  EXPECT_EQ(play_options.node_prefix, "test");
  EXPECT_EQ(play_options.rate, 13.0);
  std::vector<std::string> topics_to_filter {"/foo", "/bar"};
  EXPECT_EQ(play_options.topics_to_filter, topics_to_filter);
  std::vector<std::string> services_to_filter {
    "/service1/_service_event",
    "/service2/_service_event"};
  EXPECT_EQ(play_options.services_to_filter, services_to_filter);
  EXPECT_EQ(play_options.regex_to_filter, "[xyz]/topic_service");
  std::vector<std::string> exclude_topics_to_filter {"/exclude_foo", "/exclude_bar"};
  EXPECT_EQ(play_options.exclude_topics_to_filter, exclude_topics_to_filter);
  std::vector<std::string> exclude_services_to_filter {
    "/exclude_service1/_service_event",
    "/exclude_service2/_service_event"};
  EXPECT_EQ(play_options.exclude_services_to_filter, exclude_services_to_filter);
  EXPECT_EQ(play_options.exclude_regex_to_filter, "[abc]/topic_service");

  std::unordered_map<std::string, rclcpp::QoS> topic_qos_profile_overrides{
    std::pair{
      "/overrided_topic_qos",
      rclcpp::QoS{rclcpp::KeepLast(10)}.reliable().durability_volatile()}
  };
  EXPECT_EQ(play_options.topic_qos_profile_overrides, topic_qos_profile_overrides);
  EXPECT_EQ(play_options.loop, false);
  EXPECT_EQ(play_options.clock_publish_frequency, 19.0);
  std::vector<std::string> clock_trigger_topics {"/triggers/clock"};
  EXPECT_EQ(play_options.clock_trigger_topics, clock_trigger_topics);
  EXPECT_EQ(play_options.delay.nanoseconds(), 1);
  EXPECT_DOUBLE_EQ(play_options.playback_duration.seconds(), -1.0);
  EXPECT_EQ(play_options.playback_until_timestamp, -2500000000LL);
  EXPECT_EQ(play_options.start_offset, 999999999);
  EXPECT_EQ(play_options.disable_keyboard_controls, true);
  EXPECT_EQ(play_options.wait_acked_timeout, -999999999);
  EXPECT_EQ(play_options.disable_loan_message, false);
  EXPECT_EQ(play_options.publish_service_requests, false);
  EXPECT_EQ(
    play_options.service_requests_source,
    rosbag2_transport::ServiceRequestsSource::CLIENT_INTROSPECTION);

  ASSERT_EQ(1, storage_options.size());
  EXPECT_EQ(storage_options[0].uri, uri_str);
  EXPECT_EQ(storage_options[0].storage_id, GetParam());
  EXPECT_EQ(storage_options[0].storage_config_uri, "");
  EXPECT_EQ(storage_options[0].max_bagfile_size, 12345);
  EXPECT_EQ(storage_options[0].max_bagfile_duration, 54321);
  EXPECT_EQ(storage_options[0].max_cache_size, 9898);
  EXPECT_EQ(storage_options[0].storage_preset_profile, "resilient");
  EXPECT_EQ(storage_options[0].snapshot_mode, false);
  std::unordered_map<std::string, std::string> custom_data{
    std::pair{"key1", "value1"},
    std::pair{"key2", "value2"}
  };
  EXPECT_EQ(storage_options[0].custom_data, custom_data);
}

TEST_P(ComposablePlayerIntegrationTests, player_can_automatically_play_file_after_composition) {
  const size_t expected_number_of_messages = num_msgs_in_bag_;
  auto load_node_request = std::make_shared<composition_interfaces::srv::LoadNode::Request>();
  load_node_request->package_name = "rosbag2_transport";
  load_node_request->plugin_name = "rosbag2_transport::Player";

  const std::string uri_str = (fs::path(
      _SRC_RESOURCES_DIR_PATH) / GetParam() / "test_bag_for_seek").generic_string();
  rclcpp::Parameter uri("storage.uri", rclcpp::ParameterValue(uri_str));
  rclcpp::Parameter start_paused("play.start_paused", rclcpp::ParameterValue(true));

  load_node_request->parameters.push_back(uri.to_parameter_msg());
  load_node_request->parameters.push_back(start_paused.to_parameter_msg());

  auto load_node_future = load_node_client_->async_send_request(load_node_request);
  auto ret = exec_->spin_until_future_complete(load_node_future, 10s);  // Wait for the result.
  ASSERT_EQ(ret, rclcpp::FutureReturnCode::SUCCESS);
  auto result = load_node_future.get();
  EXPECT_EQ(result->success, true);
  EXPECT_EQ(result->error_message, "");
  EXPECT_EQ(result->full_node_name, player_node_name_);

  sub_->add_subscription<test_msgs::msg::BasicTypes>(topic_name_, expected_number_of_messages);

  ASSERT_TRUE(player_is_paused_service_request());

  // Wait for discovery to match publishers with subscribers
  ASSERT_TRUE(sub_->spin_and_wait_for_matched({topic_name_}, std::chrono::seconds(30)));

  player_resume_service_request();

  sub_->spin_subscriptions_sync();
  auto replayed_topic = sub_->get_received_messages<test_msgs::msg::BasicTypes>(topic_name_);
  ASSERT_THAT(replayed_topic, SizeIs(expected_number_of_messages));
}

INSTANTIATE_TEST_SUITE_P(
  ParametrizedComposablePlayerTests,
  ComposablePlayerTests,
  ValuesIn(rosbag2_test_common::kTestedStorageIDs)
);

INSTANTIATE_TEST_SUITE_P(
  ParametrizedComposablePlayerIntegrationTests,
  ComposablePlayerIntegrationTests,
  ValuesIn(rosbag2_test_common::kTestedStorageIDs)
);
