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
#include <type_traits>

#include "rclcpp/rclcpp.hpp"

#include "rosbag2_interfaces/srv/is_discovery_running.hpp"
#include "rosbag2_interfaces/srv/is_paused.hpp"
#include "rosbag2_interfaces/srv/pause.hpp"
#include "rosbag2_interfaces/srv/record.hpp"
#include "rosbag2_interfaces/srv/resume.hpp"
#include "rosbag2_interfaces/srv/snapshot.hpp"
#include "rosbag2_interfaces/srv/split_bagfile.hpp"
#include "rosbag2_interfaces/srv/start_discovery.hpp"
#include "rosbag2_interfaces/srv/stop_discovery.hpp"
#include "rosbag2_interfaces/srv/stop.hpp"
#include "rosbag2_transport/recorder.hpp"

#include "rosbag2_test_common/publication_manager.hpp"
#include "rosbag2_test_common/wait_for.hpp"

#include "test_msgs/message_fixtures.hpp"

#include "record_integration_fixture.hpp"

using namespace ::testing;  // NOLINT


template<class T, class = void>
struct type_has_return_code : std::false_type {};

template<class T>
struct type_has_return_code<T, std::void_t<decltype(std::declval<T &>().return_code)>>
  : std::true_type
{};

class RecordSrvsTest : public RecordIntegrationTestFixture
{
public:
  using IsDiscoveryRunning = rosbag2_interfaces::srv::IsDiscoveryRunning;
  using IsPaused = rosbag2_interfaces::srv::IsPaused;
  using Pause = rosbag2_interfaces::srv::Pause;
  using Record = rosbag2_interfaces::srv::Record;
  using Resume = rosbag2_interfaces::srv::Resume;
  using Snapshot = rosbag2_interfaces::srv::Snapshot;
  using SplitBagfile = rosbag2_interfaces::srv::SplitBagfile;
  using StartDiscovery = rosbag2_interfaces::srv::StartDiscovery;
  using Stop = rosbag2_interfaces::srv::Stop;
  using StopDiscovery = rosbag2_interfaces::srv::StopDiscovery;

  explicit RecordSrvsTest(bool snapshot_mode = false, bool is_discovery_disabled = false)
  : RecordIntegrationTestFixture(),
    snapshot_mode_(snapshot_mode),
    is_discovery_disabled_(is_discovery_disabled)
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

    const auto record_topics =
      is_discovery_disabled_ ? std::vector<std::string>{} : std::vector<std::string>{test_topic_};
    rosbag2_transport::RecordOptions record_options =
    {
      false, false, false, is_discovery_disabled_, record_topics,
      {}, {}, {}, {}, {}, {}, {}, {}, {},
      "rmw_format", 100ms
    };
    storage_options_.snapshot_mode = snapshot_mode_;
    storage_options_.max_cache_size = 700;
    recorder_ = std::make_shared<rosbag2_transport::Recorder>(
      std::move(writer_), storage_options_, record_options, recorder_name_);
    recorder_->record();

    auto string_message = get_messages_strings()[1];
    rosbag2_test_common::PublicationManager pub_manager;
    pub_manager.setup_publisher(test_topic_, string_message, 10);

    const std::string ns = "/" + recorder_name_;
    cli_is_discovery_running_ = client_node_->create_client<IsDiscoveryRunning>(
      ns + "/is_discovery_running");
    cli_is_paused_ = client_node_->create_client<IsPaused>(ns + "/is_paused");
    cli_pause_ = client_node_->create_client<Pause>(ns + "/pause");
    cli_record_ = client_node_->create_client<Record>(ns + "/record");
    cli_resume_ = client_node_->create_client<Resume>(ns + "/resume");
    cli_snapshot_ = client_node_->create_client<Snapshot>(ns + "/snapshot");
    cli_split_bagfile_ = client_node_->create_client<SplitBagfile>(ns + "/split_bagfile");
    cli_start_discovery_ = client_node_->create_client<StartDiscovery>(ns + "/start_discovery");
    cli_stop_ = client_node_->create_client<Stop>(ns + "/stop");
    cli_stop_discovery_ = client_node_->create_client<StopDiscovery>(ns + "/stop_discovery");
    exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    exec_->add_node(recorder_);
    exec_->add_node(client_node_);
    spin_thread_ = std::thread(
      [this]() {
        exec_->spin();
      });

    // Wait for the executor to start spinning in the newly spawned thread to avoid race conditions
    if (!wait_until_condition([this]() {return exec_->is_spinning();}, std::chrono::seconds(5))) {
      std::cerr << "Failed to start spinning nodes: '" <<
        recorder_->get_name() << ", " << client_node_->get_name() << "'" << std::endl;
      throw std::runtime_error("Failed to start spinning nodes");
    }

    if (!is_discovery_disabled_) {
      ASSERT_TRUE(pub_manager.wait_for_matched(test_topic_.c_str()));
    }
    pub_manager.run_publishers();

    // Make sure expected service is present before starting test
    if (snapshot_mode_) {
      ASSERT_TRUE(cli_snapshot_->wait_for_service(service_wait_timeout_));
    }
    ASSERT_TRUE(cli_is_discovery_running_->wait_for_service(service_wait_timeout_));
    ASSERT_TRUE(cli_is_paused_->wait_for_service(service_wait_timeout_));
    ASSERT_TRUE(cli_pause_->wait_for_service(service_wait_timeout_));
    ASSERT_TRUE(cli_record_->wait_for_service(service_wait_timeout_));
    ASSERT_TRUE(cli_resume_->wait_for_service(service_wait_timeout_));
    ASSERT_TRUE(cli_split_bagfile_->wait_for_service(service_wait_timeout_));
    ASSERT_TRUE(cli_start_discovery_->wait_for_service(service_wait_timeout_));
    ASSERT_TRUE(cli_stop_->wait_for_service(service_wait_timeout_));
    ASSERT_TRUE(cli_stop_discovery_->wait_for_service(service_wait_timeout_));
  }

  /// Send a service request, and expect it to successfully return within a reasonable timeout
  template<typename Srv>
  [[nodiscard]] ::testing::AssertionResult successful_service_request(
    typename rclcpp::Client<Srv>::SharedPtr cli,
    typename Srv::Request::SharedPtr request,
    typename Srv::Response::SharedPtr & response)
  {
    auto future = cli->async_send_request(request);
    if (future.wait_for(service_call_timeout_) != std::future_status::ready) {
      return ::testing::AssertionFailure() << "Service call timed out";
    }
    if (!future.valid()) {
      return ::testing::AssertionFailure() << "Service call returned invalid future";
    }
    response = future.get();
    if (!response) {
      return ::testing::AssertionFailure() << "Service call returned unsuccessful response";
    }

    if constexpr (type_has_return_code<typename Srv::Response>::value) {
      if (response->return_code) {  // success is indicated by return_code == 0 or non-empty
        return ::testing::AssertionFailure() << "Service call return_code = " <<
               response->return_code;
      } else {
        return ::testing::AssertionSuccess();
      }
    }
    // No further checks possible for services with empty response
    return ::testing::AssertionSuccess();
  }

  template<typename Srv>
  [[nodiscard]] ::testing::AssertionResult successful_service_request(
    typename rclcpp::Client<Srv>::SharedPtr cli,
    typename Srv::Response::SharedPtr & response)
  {
    auto request = std::make_shared<typename Srv::Request>();
    return successful_service_request<Srv>(cli, request, response);
  }

  template<typename Srv>
  [[nodiscard]] ::testing::AssertionResult successful_service_request(
    typename rclcpp::Client<Srv>::SharedPtr cli)
  {
    auto request = std::make_shared<typename Srv::Request>();
    auto response = std::make_shared<typename Srv::Response>();
    return successful_service_request<Srv>(cli, request, response);
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
  rclcpp::Client<IsDiscoveryRunning>::SharedPtr cli_is_discovery_running_;
  rclcpp::Client<IsPaused>::SharedPtr cli_is_paused_;
  rclcpp::Client<Pause>::SharedPtr cli_pause_;
  rclcpp::Client<Record>::SharedPtr cli_record_;
  rclcpp::Client<Resume>::SharedPtr cli_resume_;
  rclcpp::Client<Snapshot>::SharedPtr cli_snapshot_;
  rclcpp::Client<SplitBagfile>::SharedPtr cli_split_bagfile_;
  rclcpp::Client<StartDiscovery>::SharedPtr cli_start_discovery_;
  rclcpp::Client<Stop>::SharedPtr cli_stop_;
  rclcpp::Client<StopDiscovery>::SharedPtr cli_stop_discovery_;

  bool snapshot_mode_;
  bool is_discovery_disabled_;
};

class RecordSrvsSnapshotTest : public RecordSrvsTest
{
protected:
  RecordSrvsSnapshotTest()
  : RecordSrvsTest(true /*snapshot_mode*/) {}
};

TEST_F(RecordSrvsSnapshotTest, trigger_snapshot)
{
  auto & writer = recorder_->get_writer_handle();
  auto & mock_writer = dynamic_cast<MockSequentialWriter &>(writer.get_implementation_handle());
  EXPECT_THAT(mock_writer.get_number_of_recorded_messages(), Eq(0u));

  // Wait for messages to be appeared in snapshot_buffer
  std::chrono::duration<int> timeout = std::chrono::seconds(10);
  using clock = std::chrono::steady_clock;
  auto start = clock::now();
  while (mock_writer.get_snapshot_buffer_size() == 0) {
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    EXPECT_LE((clock::now() - start), timeout) << "Failed to capture messages in time";
  }
  EXPECT_THAT(mock_writer.get_number_of_recorded_messages(), Eq(0u));

  EXPECT_TRUE(successful_service_request<Snapshot>(cli_snapshot_));
  EXPECT_THAT(mock_writer.get_number_of_recorded_messages(), Ne(0u));
}

TEST_F(RecordSrvsSnapshotTest, trigger_snapshot_ignored_when_not_recording)
{
  auto & writer = recorder_->get_writer_handle();
  auto & mock_writer = dynamic_cast<MockSequentialWriter &>(writer.get_implementation_handle());
  EXPECT_THAT(mock_writer.get_number_of_recorded_messages(), Eq(0u));

  ASSERT_TRUE(successful_service_request<Stop>(cli_stop_));
  ASSERT_TRUE(mock_writer.closed_was_called());

  EXPECT_TRUE(successful_service_request<Snapshot>(cli_snapshot_));
  EXPECT_THAT(mock_writer.get_number_of_recorded_messages(), Eq(0u));
}

TEST_F(RecordSrvsTest, split_bagfile)
{
  auto & writer = recorder_->get_writer_handle();
  auto & mock_writer = dynamic_cast<MockSequentialWriter &>(writer.get_implementation_handle());
  bool callback_called = false;
  std::string closed_file, opened_file;
  rosbag2_cpp::bag_events::WriterEventCallbacks callbacks;
  callbacks.write_split_callback =
    [&callback_called, &closed_file, &opened_file](rosbag2_cpp::bag_events::BagSplitInfo & info) {
      closed_file = info.closed_file;
      opened_file = info.opened_file;
      callback_called = true;
    };
  mock_writer.add_event_callbacks(callbacks);

  // Wait for messages to be appeared in writer buffer
  std::chrono::duration<int> timeout = std::chrono::seconds(10);
  using clock = std::chrono::steady_clock;
  auto start = clock::now();
  while (mock_writer.get_number_of_recorded_messages() == 0) {
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    EXPECT_LE((clock::now() - start), timeout) << "Failed to capture messages in time";
  }

  ASSERT_FALSE(callback_called);
  EXPECT_TRUE(successful_service_request<SplitBagfile>(cli_split_bagfile_));

  // Confirm that the callback was called and the file names have been sent with the event
  ASSERT_TRUE(callback_called);
  EXPECT_EQ(closed_file, "BagFile0");
  EXPECT_EQ(opened_file, "BagFile1");
}

TEST_F(RecordSrvsTest, split_bagfile_ignored_when_not_recording)
{
  auto & writer = recorder_->get_writer_handle();
  auto & mock_writer = dynamic_cast<MockSequentialWriter &>(writer.get_implementation_handle());
  bool callback_called = false;
  rosbag2_cpp::bag_events::WriterEventCallbacks callbacks;
  callbacks.write_split_callback =
    [&callback_called](rosbag2_cpp::bag_events::BagSplitInfo & /*info*/) {
      callback_called = true;
    };
  mock_writer.add_event_callbacks(callbacks);
  ASSERT_TRUE(successful_service_request<Stop>(cli_stop_));
  ASSERT_TRUE(mock_writer.closed_was_called());
  EXPECT_FALSE(callback_called);
  EXPECT_TRUE(successful_service_request<SplitBagfile>(cli_split_bagfile_));
  EXPECT_FALSE(callback_called);
}

TEST_F(RecordSrvsTest, pause_resume)
{
  EXPECT_FALSE(recorder_->is_paused());
  auto is_paused_response = std::make_shared<IsPaused::Response>();
  EXPECT_TRUE(successful_service_request<IsPaused>(cli_is_paused_, is_paused_response));
  EXPECT_FALSE(is_paused_response->paused);

  EXPECT_TRUE(successful_service_request<Pause>(cli_pause_));
  EXPECT_TRUE(recorder_->is_paused());
  is_paused_response = std::make_shared<IsPaused::Response>();
  EXPECT_TRUE(successful_service_request<IsPaused>(cli_is_paused_, is_paused_response));
  EXPECT_TRUE(is_paused_response->paused);

  EXPECT_TRUE(successful_service_request<Resume>(cli_resume_));
  EXPECT_FALSE(recorder_->is_paused());
  is_paused_response = std::make_shared<IsPaused::Response>();
  EXPECT_TRUE(successful_service_request<IsPaused>(cli_is_paused_, is_paused_response));
  EXPECT_FALSE(is_paused_response->paused);
}

class RecordSrvsDiscoveryTest : public RecordSrvsTest
{
protected:
  RecordSrvsDiscoveryTest()
  : RecordSrvsTest(false /*snapshot_mode*/, true /*discovery_mode*/) {}
};

TEST_F(RecordSrvsDiscoveryTest, stop_start_discovery)
{
  EXPECT_FALSE(recorder_->is_discovery_running());
  auto is_discovery_running_response = std::make_shared<IsDiscoveryRunning::Response>();
  EXPECT_TRUE(successful_service_request<IsDiscoveryRunning>(cli_is_discovery_running_,
    is_discovery_running_response));
  EXPECT_FALSE(is_discovery_running_response->running);

  auto start_discovery_response = std::make_shared<StartDiscovery::Response>();
  EXPECT_TRUE(successful_service_request<StartDiscovery>(cli_start_discovery_,
    start_discovery_response));
  EXPECT_EQ(0, start_discovery_response->return_code);
  EXPECT_TRUE(start_discovery_response->error_string.empty());

  EXPECT_TRUE(recorder_->is_discovery_running());
  is_discovery_running_response = std::make_shared<IsDiscoveryRunning::Response>();
  EXPECT_TRUE(successful_service_request<IsDiscoveryRunning>(cli_is_discovery_running_,
    is_discovery_running_response));
  EXPECT_TRUE(is_discovery_running_response->running);

  auto stop_discovery_response = std::make_shared<StopDiscovery::Response>();
  EXPECT_TRUE(successful_service_request<StopDiscovery>(cli_stop_discovery_,
    stop_discovery_response));
  EXPECT_EQ(0, stop_discovery_response->return_code);
  EXPECT_TRUE(stop_discovery_response->error_string.empty());
  EXPECT_FALSE(recorder_->is_discovery_running());
  is_discovery_running_response = std::make_shared<IsDiscoveryRunning::Response>();
  EXPECT_TRUE(successful_service_request<IsDiscoveryRunning>(cli_is_discovery_running_,
    is_discovery_running_response));
  EXPECT_FALSE(is_discovery_running_response->running);
}

TEST_F(RecordSrvsTest, record_stop)
{
  auto & writer = recorder_->get_writer_handle();
  auto & mock_writer = dynamic_cast<MockSequentialWriter &>(writer.get_implementation_handle());

  EXPECT_FALSE(mock_writer.closed_was_called());

  EXPECT_TRUE(successful_service_request<Stop>(cli_stop_));
  EXPECT_TRUE(mock_writer.closed_was_called());
  EXPECT_FALSE(successful_service_request<Stop>(cli_stop_));  // second stop should fail

  auto record_response = std::make_shared<Record::Response>();
  EXPECT_TRUE(successful_service_request<Record>(cli_record_, record_response));
  EXPECT_EQ(0, record_response->return_code);
  EXPECT_TRUE(record_response->error_string.empty());
  EXPECT_FALSE(mock_writer.closed_was_called());

  EXPECT_TRUE(successful_service_request<Stop>(cli_stop_));
  EXPECT_TRUE(mock_writer.closed_was_called());
}

TEST_F(RecordSrvsTest, record_with_uri)
{
  static const std::string test_uri = "file:///tmp/test_bag_with_uri";

  auto & writer = recorder_->get_writer_handle();
  auto & mock_writer = dynamic_cast<MockSequentialWriter &>(writer.get_implementation_handle());

  EXPECT_NE(mock_writer.get_storage_options().uri, test_uri);

  ASSERT_TRUE(successful_service_request<Stop>(cli_stop_));

  auto record_request = std::make_shared<Record::Request>();
  record_request->uri = test_uri;
  auto record_response = std::make_shared<Record::Response>();
  EXPECT_TRUE(successful_service_request<Record>(cli_record_, record_request, record_response));
  EXPECT_EQ(0, record_response->return_code);
  EXPECT_TRUE(record_response->error_string.empty());

  EXPECT_EQ(mock_writer.get_storage_options().uri, test_uri);
}
