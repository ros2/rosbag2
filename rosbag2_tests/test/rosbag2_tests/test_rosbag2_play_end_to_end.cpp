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

#include <gmock/gmock.h>

#include <cstdlib>
#include <future>
#include <memory>
#include <string>
#include <vector>

#include "rosbag2_interfaces/srv/resume.hpp"
#include "rosbag2_test_common/process_execution_helpers.hpp"
#include "rosbag2_test_common/subscription_manager.hpp"
#include "rosbag2_test_common/tested_storage_ids.hpp"

#include "test_msgs/msg/arrays.hpp"
#include "test_msgs/msg/basic_types.hpp"

using namespace ::testing;  // NOLINT
using namespace rosbag2_test_common;  // NOLINT

class PlayEndToEndTestFixture : public Test, public WithParamInterface<std::string>
{
public:
  using Resume = rosbag2_interfaces::srv::Resume;

  PlayEndToEndTestFixture()
  : sub_qos_(rclcpp::QoS{10}
      .reliability(rclcpp::ReliabilityPolicy::Reliable)
      .durability(rclcpp::DurabilityPolicy::TransientLocal))
  {
    // _SRC_RESOURCES_DIR_PATH defined in CMakeLists.txt
    bags_path_ = (rcpputils::fs::path(_SRC_RESOURCES_DIR_PATH) / GetParam()).string();
    sub_ = std::make_unique<SubscriptionManager>();
    client_node_ = std::make_shared<rclcpp::Node>("test_record_client");
    cli_resume_ = client_node_->create_client<Resume>("/rosbag2_player/resume");
    exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    exec_->add_node(client_node_);
    spin_thread_ = std::thread(
      [this]() {
        exec_->spin();
      });
  }

  ~PlayEndToEndTestFixture() override
  {
    exec_->cancel();
    if (spin_thread_.joinable()) {spin_thread_.join();}
  }

  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  /// Send a service request, and expect it to successfully return within a reasonable timeout
  template<typename Srv>
  typename Srv::Response::SharedPtr successful_service_request(
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
  typename Srv::Response::SharedPtr successful_service_request(
    typename rclcpp::Client<Srv>::SharedPtr cli)
  {
    auto request = std::make_shared<typename Srv::Request>();
    return successful_service_request<Srv>(cli, request);
  }

  rclcpp::QoS sub_qos_;
  std::string bags_path_;
  std::unique_ptr<SubscriptionManager> sub_;
  rclcpp::Node::SharedPtr client_node_;
  rclcpp::Client<Resume>::SharedPtr cli_resume_;
  std::thread spin_thread_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec_;
  const std::chrono::seconds service_call_timeout_ {10};
};

TEST_P(PlayEndToEndTestFixture, play_end_to_end_test) {
  sub_->add_subscription<test_msgs::msg::Arrays>("/array_topic", 4, sub_qos_);
  sub_->add_subscription<test_msgs::msg::BasicTypes>("/test_topic", 3, sub_qos_);

  // Start ros2 bag play in pause mode
  auto process_id = start_execution("ros2 bag play -p " + bags_path_ + "/cdr_test");
  auto cleanup_process_handle = rcpputils::make_scope_exit(
    [process_id]() {
      stop_execution(process_id);
    });

  EXPECT_TRUE(sub_->spin_and_wait_for_matched({"/test_topic", "/array_topic"}));
  auto subscription_future = sub_->spin_subscriptions();

  // Send resume service call for player
  ASSERT_TRUE(cli_resume_->wait_for_service(service_call_timeout_));
  successful_service_request<Resume>(cli_resume_);

  subscription_future.get();

  auto primitive_messages = sub_->get_received_messages<test_msgs::msg::BasicTypes>("/test_topic");
  auto array_messages = sub_->get_received_messages<test_msgs::msg::Arrays>("/array_topic");

  EXPECT_THAT(primitive_messages, SizeIs(Ge(3u)));
  EXPECT_THAT(
    primitive_messages,
    Each(Pointee(Field(&test_msgs::msg::BasicTypes::int32_value, 123))));

  EXPECT_THAT(array_messages, SizeIs(Ge(4u)));
  EXPECT_THAT(
    array_messages,
    Each(
      Pointee(
        Field(
          &test_msgs::msg::Arrays::bool_values,
          ElementsAre(true, false, true)))));
  EXPECT_THAT(
    array_messages,
    Each(
      Pointee(
        Field(
          &test_msgs::msg::Arrays::string_values,
          ElementsAre("Complex Hello1", "Complex Hello2", "Complex Hello3")))));
  EXPECT_TRUE(wait_until_completion(process_id));
}

TEST_P(PlayEndToEndTestFixture, play_fails_gracefully_if_bag_does_not_exist) {
  internal::CaptureStderr();
  auto exit_code =
    execute_and_wait_until_completion("ros2 bag play does_not_exist", bags_path_);
  auto error_output = internal::GetCapturedStderr();

  // Exit code could be EXIT_FAILURE (1) or 2 (no such file or directory)
  EXPECT_THAT(exit_code, Ne(EXIT_SUCCESS));
  EXPECT_THAT(error_output, HasSubstr("'does_not_exist' does not exist"));
}

TEST_P(PlayEndToEndTestFixture, play_fails_gracefully_if_needed_coverter_plugin_does_not_exist) {
  internal::CaptureStderr();
  auto exit_code =
    execute_and_wait_until_completion("ros2 bag play wrong_rmw_test", bags_path_);
  auto error_output = internal::GetCapturedStderr();

  EXPECT_THAT(exit_code, Eq(EXIT_FAILURE));
  EXPECT_THAT(error_output, HasSubstr("Could not find converter for format wrong_format"));
}

TEST_P(PlayEndToEndTestFixture, play_filters_by_topic) {
  const unsigned int num_basic_msgs = 3u;
  const unsigned int num_array_msgs = 4u;

  {  // Play with filter for `/test_topic` only
    sub_->add_subscription<test_msgs::msg::BasicTypes>("/test_topic", num_basic_msgs, sub_qos_);
    sub_->add_subscription<test_msgs::msg::Arrays>("/array_topic", 0, sub_qos_);

    // Start ros2 bag play in pause mode
    auto process_id =
      start_execution("ros2 bag play -p " + bags_path_ + "/cdr_test --topics /test_topic");
    auto cleanup_process_handle = rcpputils::make_scope_exit(
      [process_id]() {
        stop_execution(process_id);
      });

    EXPECT_TRUE(sub_->spin_and_wait_for_matched({"/test_topic"}));
    auto subscription_future = sub_->spin_subscriptions();

    // Send resume service call for player
    ASSERT_TRUE(cli_resume_->wait_for_service(service_call_timeout_));
    successful_service_request<Resume>(cli_resume_);

    subscription_future.get();

    auto primitive_messages =
      sub_->get_received_messages<test_msgs::msg::BasicTypes>("/test_topic");
    auto array_messages = sub_->get_received_messages<test_msgs::msg::Arrays>("/array_topic");

    EXPECT_THAT(primitive_messages, SizeIs(Ge(num_basic_msgs)));
    EXPECT_THAT(array_messages, SizeIs(Eq(0u)));
    EXPECT_TRUE(wait_until_completion(process_id));
  }

  // Play with filter for both topics
  {
    sub_ = std::make_unique<SubscriptionManager>();
    sub_->add_subscription<test_msgs::msg::BasicTypes>("/test_topic", num_basic_msgs, sub_qos_);
    sub_->add_subscription<test_msgs::msg::Arrays>("/array_topic", num_array_msgs, sub_qos_);

    // Start ros2 bag play in pause mode
    auto process_id = start_execution(
      "ros2 bag play -p " + bags_path_ + "/cdr_test --topics /test_topic /array_topic");
    auto cleanup_process_handle = rcpputils::make_scope_exit(
      [process_id]() {
        stop_execution(process_id);
      });

    EXPECT_TRUE(sub_->spin_and_wait_for_matched({"/test_topic", "/array_topic"}));
    auto subscription_future = sub_->spin_subscriptions();

    // Send resume service call for player
    ASSERT_TRUE(cli_resume_->wait_for_service(service_call_timeout_));
    successful_service_request<Resume>(cli_resume_);

    subscription_future.get();

    auto primitive_messages =
      sub_->get_received_messages<test_msgs::msg::BasicTypes>("/test_topic");
    auto array_messages = sub_->get_received_messages<test_msgs::msg::Arrays>("/array_topic");

    EXPECT_THAT(primitive_messages, SizeIs(Ge(num_basic_msgs)));
    EXPECT_THAT(array_messages, SizeIs(Ge(num_array_msgs)));
    EXPECT_TRUE(wait_until_completion(process_id));
  }

  // Play with filter for non-existent topic
  {
    sub_ = std::make_unique<SubscriptionManager>();
    sub_->add_subscription<test_msgs::msg::BasicTypes>("/test_topic", 1, sub_qos_);
    sub_->add_subscription<test_msgs::msg::Arrays>("/array_topic", 1, sub_qos_);

    // Start ros2 bag play in pause mode
    auto process_id = start_execution(
      "ros2 bag play -p " + bags_path_ + "/cdr_test --topics /nonexistent_topic");
    auto cleanup_process_handle = rcpputils::make_scope_exit(
      [process_id]() {
        stop_execution(process_id);
      });

    EXPECT_FALSE(
      sub_->spin_and_wait_for_matched(
        std::vector<std::string>{"/test_topic", "/array_topic"}, std::chrono::seconds(1)));
    auto subscription_future = sub_->spin_subscriptions(std::chrono::seconds(1));

    // Send resume service call for player
    ASSERT_TRUE(cli_resume_->wait_for_service(service_call_timeout_));
    successful_service_request<Resume>(cli_resume_);

    subscription_future.get();

    auto primitive_messages =
      sub_->get_received_messages<test_msgs::msg::BasicTypes>("/test_topic");
    auto array_messages = sub_->get_received_messages<test_msgs::msg::Arrays>("/array_topic");

    EXPECT_THAT(primitive_messages, SizeIs(Eq(0u)));
    EXPECT_THAT(array_messages, SizeIs(Eq(0u)));
    EXPECT_TRUE(wait_until_completion(process_id));
  }
}

#ifndef _WIN32
TEST_P(PlayEndToEndTestFixture, play_end_to_end_exits_gracefully_on_sigint) {
  sub_->add_subscription<test_msgs::msg::BasicTypes>("/test_topic", 3, sub_qos_);
  sub_->add_subscription<test_msgs::msg::Arrays>("/array_topic", 2, sub_qos_);

  // Start playback in child process
  auto process_id = start_execution("ros2 bag play --loop " + bags_path_ + "/cdr_test");
  auto cleanup_process_handle = rcpputils::make_scope_exit(
    [process_id]() {
      stop_execution(process_id);
    });

  // Wait for a few messages to arrive. This way we deterministically check that playback has been
  // successfully started at this point.
  sub_->spin_subscriptions_sync();
  auto primitive_messages = sub_->get_received_messages<test_msgs::msg::BasicTypes>("/test_topic");
  ASSERT_THAT(primitive_messages, SizeIs(Ge(3u)));

  // Send SIGINT to child process and check exit code
  stop_execution(process_id, SIGINT);
  cleanup_process_handle.cancel();
}
#endif  // #ifndef _WIN32

INSTANTIATE_TEST_SUITE_P(
  TestPlayEndToEnd,
  PlayEndToEndTestFixture,
  ::testing::ValuesIn(rosbag2_test_common::kTestedStorageIDs)
);
