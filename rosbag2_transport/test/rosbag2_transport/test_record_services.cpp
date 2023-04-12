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

#include "rosbag2_interfaces/srv/snapshot.hpp"
#include "rosbag2_transport/recorder.hpp"

#include "rosbag2_test_common/publication_manager.hpp"

#include "test_msgs/message_fixtures.hpp"

#include "record_integration_fixture.hpp"

using namespace ::testing;  // NOLINT

class RecordSrvsTest : public RecordIntegrationTestFixture
{
public:
  using Snapshot = rosbag2_interfaces::srv::Snapshot;

  RecordSrvsTest()
  : RecordIntegrationTestFixture()
  {}

  ~RecordSrvsTest() override
  {
    exec_->cancel();
    rclcpp::shutdown();
    spin_thread_.join();
  }

  void TearDown() override
  {
  }

  /// Use SetUp instead of ctor because we want to ASSERT some preconditions for the tests
  void SetUp() override
  {
    RecordIntegrationTestFixture::SetUp();
    client_node_ = std::make_shared<rclcpp::Node>("test_record_client");

    rosbag2_transport::RecordOptions record_options =
    {false, false, {test_topic_}, "rmw_format", 100ms};
    storage_options_.snapshot_mode = true;
    storage_options_.max_cache_size = 200;
    recorder_ = std::make_shared<rosbag2_transport::Recorder>(
      std::move(writer_), storage_options_, record_options, recorder_name_);
    recorder_->record();

    auto string_message = get_messages_strings()[1];
    rosbag2_test_common::PublicationManager pub_manager;
    pub_manager.setup_publisher(test_topic_, string_message, 10);

    const std::string ns = "/" + recorder_name_;
    cli_snapshot_ = client_node_->create_client<Snapshot>(ns + "/snapshot");

    exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    exec_->add_node(recorder_);
    exec_->add_node(client_node_);
    spin_thread_ = std::thread(
      [this]() {
        exec_->spin();
      });

    ASSERT_TRUE(pub_manager.wait_for_matched(test_topic_.c_str()));
    pub_manager.run_publishers();

    // Make sure expected service is present before starting test
    ASSERT_TRUE(cli_snapshot_->wait_for_service(service_wait_timeout_));
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

public:
  // Basic configuration
  const std::string recorder_name_ = "rosbag2_recorder_for_test_srvs";
  const std::chrono::seconds service_wait_timeout_ {2};
  const std::chrono::seconds service_call_timeout_ {2};
  const std::string test_topic_ = "/recorder_srvs_test_topic";

  // Orchestration
  std::thread spin_thread_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec_;
  std::shared_ptr<rosbag2_transport::Recorder> recorder_;

  // Service clients
  rclcpp::Node::SharedPtr client_node_;
  rclcpp::Client<Snapshot>::SharedPtr cli_snapshot_;
};

TEST_F(RecordSrvsTest, trigger_snapshot)
{
  auto & writer = recorder_->get_writer_handle();
  auto & mock_writer = dynamic_cast<MockSequentialWriter &>(writer.get_implementation_handle());
  EXPECT_THAT(mock_writer.get_messages().size(), Eq(0u));

  // Wait for messages to be appeared in snapshot_buffer
  std::chrono::duration<int> timeout = std::chrono::seconds(10);
  using clock = std::chrono::steady_clock;
  auto start = clock::now();
  while (mock_writer.get_snapshot_buffer().empty()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    EXPECT_LE((clock::now() - start), timeout) << "Failed to capture messages in time";
  }
  EXPECT_THAT(mock_writer.get_messages().size(), Eq(0u));

  successful_service_request<Snapshot>(cli_snapshot_);
  EXPECT_THAT(mock_writer.get_messages().size(), Ne(0u));
}
