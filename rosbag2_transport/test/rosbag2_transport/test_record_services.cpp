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

#include <atomic>
#include <memory>
#include <string>
#include <vector>
#include <utility>
#include <type_traits>

#include "rcutils/time.h"

#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

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
#include "rosbag2_storage/qos.hpp"
#include "rosbag2_transport/recorder.hpp"

#include "rosbag2_test_common/publication_manager.hpp"
#include "rosbag2_test_common/wait_for.hpp"

#include "test_msgs/message_fixtures.hpp"

#include "record_integration_fixture.hpp"

using namespace ::testing;  // NOLINT
using namespace std::chrono_literals;  // NOLINT

using Rosbag2QoS = rosbag2_storage::Rosbag2QoS;  // NOLINT

constexpr const char kTestTopic[] = "/recorder_srvs_test_topic";
constexpr const char kSplitOtherTopic[] = "/recorder_srvs_other_topic";


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

  explicit RecordSrvsTest(
    bool snapshot_mode = false,
    bool is_discovery_disabled = false,
    bool use_sim_time = false,
    std::vector<std::string> topics_to_record = {kTestTopic})
  : RecordIntegrationTestFixture(),
    snapshot_mode_(snapshot_mode),
    is_discovery_disabled_(is_discovery_disabled),
    use_sim_time_(use_sim_time),
    topics_to_record_(std::move(topics_to_record))
  {}

  ~RecordSrvsTest() override
  {
    exec_->cancel();
    rclcpp::shutdown();
    spin_thread_.join();
  }

  void TearDown() override {}

  /// Use SetUp instead of ctor because we want to ASSERT some preconditions for the tests
  void SetUp() override
  {
    RecordIntegrationTestFixture::SetUp();
    client_node_ = std::make_shared<rclcpp::Node>("test_record_client");
    if (use_sim_time_) {
      client_node_->set_parameter(rclcpp::Parameter("use_sim_time", true));
    }

    auto record_topics =
      is_discovery_disabled_ ? std::vector<std::string>{} : topics_to_record_;

    rosbag2_transport::RecordOptions record_options;
    record_options.is_discovery_disabled = is_discovery_disabled_;
    record_options.topics = record_topics;
    record_options.rmw_serialization_format = "rmw_format";
    record_options.topic_polling_interval = 100ms;
    record_options.use_sim_time = use_sim_time_;
    storage_options_.snapshot_mode = snapshot_mode_;
    storage_options_.max_cache_size = 700;
    recorder_ = std::make_shared<rosbag2_transport::Recorder>(
      std::move(writer_), storage_options_, record_options, recorder_name_);
    if (use_sim_time_) {
      recorder_->set_parameter(rclcpp::Parameter("use_sim_time", true));
    }
    recorder_->record();

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
    spin_thread_ = std::thread(
      [this]() {
        exec_->spin();
      });

    // Wait for the executor to start spinning in the newly spawned thread to avoid race conditions
    if (!wait_until_condition([this]() {return exec_->is_spinning();}, std::chrono::seconds(10))) {
      std::cerr << "Failed to start spinning nodes: '" <<
        recorder_->get_name() << ", " << client_node_->get_name() << "'" << std::endl;
      throw std::runtime_error("Failed to start spinning nodes");
    }

    if (use_sim_time_) {
      clock_pub_ =
        client_node_->create_publisher<rosgraph_msgs::msg::Clock>("/clock",
                                                                  Rosbag2QoS::UnitTestQoS());
      ASSERT_TRUE(
        rosbag2_test_common::wait_until_condition(
          [this]() {return clock_pub_->get_subscription_count() > 0;},
          std::chrono::seconds(10)));
      current_sim_time_ = rclcpp::Time(1, 0, RCL_ROS_TIME);
      publish_clock(current_sim_time_);
      ASSERT_TRUE(
        rosbag2_test_common::wait_until_condition(
          [this]() {return recorder_->get_clock()->started();},
          std::chrono::seconds(10)));
      EXPECT_EQ(recorder_->get_clock()->get_clock_type(), RCL_ROS_TIME);
      ASSERT_TRUE(recorder_->get_clock()->ros_time_is_active());
    }

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
    // Create a separate executor for service calls to avoid "already spinning" error
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(client_node_);

    auto future = cli->async_send_request(request);

    const auto ret = exec.spin_until_future_complete(future, service_call_timeout_);

    if (ret != rclcpp::FutureReturnCode::SUCCESS) {
      cli->remove_pending_request(future);
      if (ret == rclcpp::FutureReturnCode::TIMEOUT) {
        return ::testing::AssertionFailure() << "Service call timed out";
      } else if (ret == rclcpp::FutureReturnCode::INTERRUPTED) {
        return ::testing::AssertionFailure() << "Service call interrupted";
      }
      return ::testing::AssertionFailure() << "Service call failed with unknown error";
    }

    if (!future.valid()) {
      return ::testing::AssertionFailure() << "Service call returned invalid future";
    }

    response = future.get();
    if (!response) {
      return ::testing::AssertionFailure() << "Service call returned unsuccessful response";
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

  void publish_clock(const rclcpp::Time & time)
  {
    if (!use_sim_time_) {
      return;
    }
    rosgraph_msgs::msg::Clock msg;
    msg.clock = time;
    clock_pub_->publish(msg);
    current_sim_time_ = time;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  void advance_sim_time(const rclcpp::Duration & delta)
  {
    if (!use_sim_time_) {
      return;
    }
    publish_clock(current_sim_time_ + delta);
  }

public:
  // Basic configuration
  const std::string recorder_name_ = "rosbag2_recorder_for_test_srvs";
  // Wait longer due to potential service latency after pause/resume operations
  const std::chrono::seconds service_wait_timeout_ {10};
  const std::chrono::seconds service_call_timeout_ {10};
  const std::string test_topic_ = kTestTopic;

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
  bool use_sim_time_;
  std::vector<std::string> topics_to_record_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::Time current_sim_time_{0, 0, RCL_ROS_TIME};
};

class RecordSrvsSnapshotTest : public RecordSrvsTest
{
protected:
  RecordSrvsSnapshotTest()
  : RecordSrvsTest(true /*snapshot_mode*/) {}
};

class RecordSrvsSimTimeTest : public RecordSrvsTest
{
protected:
  RecordSrvsSimTimeTest()
  : RecordSrvsTest(false /*snapshot_mode*/, false /*is_discovery_disabled*/, true /*use_sim_time*/)
  {}
};

class RecordSrvsPublishMultipleTopicsTest : public RecordSrvsTest
{
protected:
  RecordSrvsPublishMultipleTopicsTest()
  : RecordSrvsTest(false /*snapshot_mode*/, false /*is_discovery_disabled*/,
      true /*use_sim_time*/, {kTestTopic, kSplitOtherTopic})
  {}
};

class RecordSrvsDiscoveryTest : public RecordSrvsTest
{
protected:
  RecordSrvsDiscoveryTest()
  : RecordSrvsTest(false /*snapshot_mode*/, true /*discovery_mode*/) {}
};

TEST_F(RecordSrvsSnapshotTest, trigger_snapshot)
{
  rosbag2_test_common::PublicationManager pub_manager;
  auto string_message = get_messages_strings()[1];
  pub_manager.setup_publisher(test_topic_, string_message, 10);
  ASSERT_TRUE(pub_manager.wait_for_matched(test_topic_.c_str()));
  pub_manager.run_publishers();

  auto & writer = recorder_->get_writer_handle();
  auto & mock_writer = dynamic_cast<MockSequentialWriter &>(writer.get_implementation_handle());
  EXPECT_THAT(mock_writer.get_number_of_recorded_messages(), Eq(0u));

  // Wait for messages to be appeared in snapshot_buffer
  ASSERT_TRUE(
    rosbag2_test_common::wait_until_condition(
      [&mock_writer]() {return mock_writer.get_snapshot_buffer_size() > 0;},
      std::chrono::seconds(10))
  ) << "Failed to capture message in time";

  EXPECT_THAT(mock_writer.get_number_of_recorded_messages(), Eq(0u));

  EXPECT_TRUE(successful_service_request<Snapshot>(cli_snapshot_));
  EXPECT_THAT(mock_writer.get_number_of_recorded_messages(), Ne(0u));
}

TEST_F(RecordSrvsSnapshotTest, trigger_snapshot_ignored_when_not_recording)
{
  rosbag2_test_common::PublicationManager pub_manager;
  auto string_message = get_messages_strings()[1];
  pub_manager.setup_publisher(test_topic_, string_message, 10);
  ASSERT_TRUE(pub_manager.wait_for_matched(test_topic_.c_str()));
  pub_manager.run_publishers();

  auto & writer = recorder_->get_writer_handle();
  auto & mock_writer = dynamic_cast<MockSequentialWriter &>(writer.get_implementation_handle());
  EXPECT_THAT(mock_writer.get_number_of_recorded_messages(), Eq(0u));

  auto stop_response = std::make_shared<Stop::Response>();
  ASSERT_TRUE(successful_service_request<Stop>(cli_stop_, stop_response));
  EXPECT_EQ(stop_response->return_code, 0);
  ASSERT_TRUE(mock_writer.closed_was_called());

  EXPECT_TRUE(successful_service_request<Snapshot>(cli_snapshot_));
  EXPECT_THAT(mock_writer.get_number_of_recorded_messages(), Eq(0u));
}

TEST_F(RecordSrvsTest, split_bagfile_with_default_parameters)
{
  rosbag2_test_common::PublicationManager pub_manager;
  auto string_message = get_messages_strings()[1];
  pub_manager.setup_publisher(test_topic_, string_message, 1);
  ASSERT_TRUE(pub_manager.wait_for_matched(test_topic_.c_str()));
  pub_manager.run_publishers();

  auto & writer = recorder_->get_writer_handle();
  auto & mock_writer = dynamic_cast<MockSequentialWriter &>(writer.get_implementation_handle());
  std::atomic<bool> split_called{false};
  std::string closed_file, opened_file;
  rosbag2_cpp::bag_events::WriterEventCallbacks callbacks;
  callbacks.write_split_callback =
    [&split_called, &closed_file, &opened_file](rosbag2_cpp::bag_events::BagSplitInfo & info) {
      closed_file = info.closed_file;
      opened_file = info.opened_file;
      split_called.store(true);
    };
  mock_writer.add_event_callbacks(callbacks);

  // Wait for message to appear in writer buffer to make sure that recorder has received it
  ASSERT_TRUE(
    rosbag2_test_common::wait_until_condition(
      [&mock_writer]() {return mock_writer.get_number_of_recorded_messages() > 0;},
      std::chrono::seconds(10))
  ) << "Failed to capture message in time";

  ASSERT_FALSE(split_called.load());

  auto request = std::make_shared<SplitBagfile::Request>();
  auto response = std::make_shared<SplitBagfile::Response>();
  ASSERT_TRUE(successful_service_request<SplitBagfile>(cli_split_bagfile_, request, response));
  EXPECT_EQ(SplitBagfile::Response::RETURN_CODE_SUCCESS, response->return_code);
  EXPECT_TRUE(response->error_string.empty());

  // Confirm that the callback was called and the file names have been sent with the event
  ASSERT_TRUE(split_called.load());
  EXPECT_EQ(closed_file, "BagFile0");
  EXPECT_EQ(opened_file, "BagFile1");
}

TEST_F(RecordSrvsSimTimeTest, split_bagfile_by_receive_time_scheduled)
{
  auto string_message = get_messages_strings()[1];
  auto tracked_pub =
    client_node_->create_publisher<test_msgs::msg::Strings>(test_topic_,
                                                            Rosbag2QoS::UnitTestQoS());
  ASSERT_TRUE(
    rosbag2_test_common::wait_until_condition(
      [&tracked_pub]() {return tracked_pub->get_subscription_count() > 0;},
      std::chrono::seconds(10))
  );

  auto & writer = recorder_->get_writer_handle();
  auto & mock_writer = dynamic_cast<MockSequentialWriter &>(writer.get_implementation_handle());
  std::atomic<bool> split_called{false};
  rosbag2_cpp::bag_events::WriterEventCallbacks callbacks;
  callbacks.write_split_callback =
    [&split_called](rosbag2_cpp::bag_events::BagSplitInfo & /*info*/) {
      split_called.store(true);
    };
  mock_writer.add_event_callbacks(callbacks);

  auto split_time = current_sim_time_ + rclcpp::Duration(std::chrono::milliseconds(200));
  // Request a receive-time split using receive timestamps for comparison
  auto request = std::make_shared<SplitBagfile::Request>();
  request->split_time = split_time;
  request->split_mode = SplitBagfile::Request::SPLIT_MODE_RECEIVE_TIME;
  request->tracking_topic_name = test_topic_;
  auto response = std::make_shared<SplitBagfile::Response>();
  ASSERT_TRUE(successful_service_request<SplitBagfile>(cli_split_bagfile_, request, response));
  EXPECT_EQ(SplitBagfile::Response::RETURN_CODE_SUCCESS, response->return_code);
  EXPECT_TRUE(response->error_string.empty());

  EXPECT_FALSE(split_called.load());

  // Advance simulated time past the split time and publish a message to trigger the split from the
  // subscription callback
  publish_clock(split_time + rclcpp::Duration(std::chrono::milliseconds(10)));
  ASSERT_TRUE(
    rosbag2_test_common::wait_until_condition(
      [this]() {return recorder_->now() >= current_sim_time_;},
      std::chrono::seconds(10))
  ) << "Recorder node failed to advance to the current sim time";

  tracked_pub->publish(*string_message);

  const bool split_happened = rosbag2_test_common::wait_until_condition(
    [&split_called]() {return split_called.load();},
    std::chrono::seconds(10));
  ASSERT_TRUE(split_happened) << "Timed out waiting for receive-time split to occur.";
}

TEST_F(RecordSrvsSimTimeTest, split_bagfile_by_receive_time_immediate_due)
{
  auto string_message = get_messages_strings()[1];
  auto tracked_pub =
    client_node_->create_publisher<test_msgs::msg::Strings>(test_topic_,
                                                            Rosbag2QoS::UnitTestQoS());
  ASSERT_TRUE(
    rosbag2_test_common::wait_until_condition(
      [&tracked_pub]() {return tracked_pub->get_subscription_count() > 0;},
      std::chrono::seconds(10))
  );

  auto & writer = recorder_->get_writer_handle();
  auto & mock_writer = dynamic_cast<MockSequentialWriter &>(writer.get_implementation_handle());
  std::atomic<bool> split_called{false};
  rosbag2_cpp::bag_events::WriterEventCallbacks callbacks;
  callbacks.write_split_callback =
    [&split_called](rosbag2_cpp::bag_events::BagSplitInfo & /*info*/) {
      split_called.store(true);
    };
  mock_writer.add_event_callbacks(callbacks);

  ASSERT_TRUE(
    rosbag2_test_common::wait_until_condition(
      [this]() {return recorder_->now() >= current_sim_time_;},
      std::chrono::seconds(10))
  ) << "Recorder node failed to advance to the current sim time";

  // Publish a message to establish a receive timestamp baseline
  tracked_pub->publish(*string_message);
  // Wait for message to appear in writer buffer to make sure that recorder has received it
  ASSERT_TRUE(
    rosbag2_test_common::wait_until_condition(
      [&mock_writer]() {return mock_writer.get_number_of_recorded_messages() > 0;},
      std::chrono::seconds(10))
  ) << "Failed to capture message in time";

  // Request a receive-time split using a split time in the past
  // All the recorded messages have current_sim_time_ as their receive timestamp
  auto split_time = current_sim_time_ - rclcpp::Duration(std::chrono::milliseconds(100));

  // Request a receive-time split using receive timestamps for comparison
  auto request = std::make_shared<SplitBagfile::Request>();
  request->split_time = split_time;
  request->split_mode = SplitBagfile::Request::SPLIT_MODE_RECEIVE_TIME;
  request->tracking_topic_name = test_topic_;
  auto response = std::make_shared<SplitBagfile::Response>();
  ASSERT_TRUE(successful_service_request<SplitBagfile>(cli_split_bagfile_, request, response));
  EXPECT_EQ(SplitBagfile::Response::RETURN_CODE_SUCCESS, response->return_code);
  EXPECT_TRUE(response->error_string.empty());

  // Since the split time is in the past, the split should happen immediately before returning
  // response to the service call
  EXPECT_TRUE(split_called.load());
}

TEST_F(RecordSrvsTest, split_bagfile_by_publish_time_immediate_due)
{
#ifdef _WIN32
  if (std::string(rmw_get_implementation_identifier()).find("rmw_connextdds") !=
    std::string::npos)
  {
    GTEST_SKIP() << "Skipping test on Windows with Connext due to known issue with send "
      "timestamps being zero.";
  }
#endif
  rosbag2_test_common::PublicationManager pub_manager;
  auto string_message = get_messages_strings()[1];
  pub_manager.setup_publisher(test_topic_, string_message, 2);
  ASSERT_TRUE(pub_manager.wait_for_matched(test_topic_.c_str()));
  pub_manager.run_publishers();

  auto & writer = recorder_->get_writer_handle();
  auto & mock_writer = dynamic_cast<MockSequentialWriter &>(writer.get_implementation_handle());
  std::atomic<bool> split_called{false};
  rosbag2_cpp::bag_events::WriterEventCallbacks callbacks;
  callbacks.write_split_callback =
    [&split_called](rosbag2_cpp::bag_events::BagSplitInfo & /*info*/) {
      split_called.store(true);
    };
  mock_writer.add_event_callbacks(callbacks);

  // Wait for messages to be appeared in writer buffer
  ASSERT_TRUE(
    rosbag2_test_common::wait_until_condition(
      [&mock_writer]() {return mock_writer.get_number_of_recorded_messages() > 1;},
      std::chrono::seconds(10))
  ) << "Failed to capture message in time";

  const auto first_publish_timestamp = mock_writer.get_messages().front()->send_timestamp;
  // Request a publish-time split using a split time in the past. Note: we published at least two
  // messages, so adding 1 nanosecond to the first message's timestamp guarantees that there is at
  // least one message with a publish timestamp >= split_time
  auto split_time =
    rclcpp::Time(first_publish_timestamp + 1, client_node_->get_clock()->get_clock_type());

  auto request = std::make_shared<SplitBagfile::Request>();
  request->split_time = split_time;
  request->split_mode = SplitBagfile::Request::SPLIT_MODE_PUBLISH_TIME;
  request->tracking_topic_name = test_topic_;
  auto response = std::make_shared<SplitBagfile::Response>();
  ASSERT_TRUE(successful_service_request<SplitBagfile>(cli_split_bagfile_, request, response));
  EXPECT_EQ(SplitBagfile::Response::RETURN_CODE_SUCCESS, response->return_code);
  EXPECT_TRUE(response->error_string.empty());

  // Since the split time is in the past, the split should happen immediately before returning
  // response to the service call
  EXPECT_TRUE(split_called.load());
}

TEST_F(RecordSrvsTest, split_bagfile_by_publish_time_scheduled)
{
#ifdef _WIN32
  if (std::string(rmw_get_implementation_identifier()).find("rmw_connextdds") !=
    std::string::npos)
  {
    GTEST_SKIP() << "Skipping test on Windows with Connext due to known issue with send "
      "timestamps being zero.";
  }
#endif

  auto string_message = get_messages_strings()[1];
  auto tracked_pub =
    client_node_->create_publisher<test_msgs::msg::Strings>(test_topic_,
                                                            Rosbag2QoS::UnitTestQoS());
  ASSERT_TRUE(
    rosbag2_test_common::wait_until_condition(
      [&tracked_pub]() {return tracked_pub->get_subscription_count() > 0;},
      std::chrono::seconds(10)
    )
  );
  // Publish a message to establish a publish timestamp baseline
  tracked_pub->publish(*string_message);

  auto & writer = recorder_->get_writer_handle();
  auto & mock_writer = dynamic_cast<MockSequentialWriter &>(writer.get_implementation_handle());
  std::atomic<bool> split_called{false};
  rosbag2_cpp::bag_events::WriterEventCallbacks callbacks;
  callbacks.write_split_callback =
    [&split_called](rosbag2_cpp::bag_events::BagSplitInfo & /*info*/) {
      split_called.store(true);
    };
  mock_writer.add_event_callbacks(callbacks);

  // Wait for messages to be appeared in writer buffer
  ASSERT_TRUE(
    rosbag2_test_common::wait_until_condition(
      [&mock_writer]() {return mock_writer.get_number_of_recorded_messages() > 0;},
      std::chrono::seconds(10))
  ) << "Failed to capture message in time";

  const auto last_publish_timestamp = mock_writer.get_messages().back()->send_timestamp;
  auto split_time = rclcpp::Time(last_publish_timestamp + RCUTILS_MS_TO_NS(200), RCL_SYSTEM_TIME);

  auto request = std::make_shared<SplitBagfile::Request>();
  request->split_time = split_time;
  request->split_mode = SplitBagfile::Request::SPLIT_MODE_PUBLISH_TIME;
  request->tracking_topic_name = test_topic_;
  auto response = std::make_shared<SplitBagfile::Response>();
  ASSERT_TRUE(successful_service_request<SplitBagfile>(cli_split_bagfile_, request, response));
  EXPECT_EQ(SplitBagfile::Response::RETURN_CODE_SUCCESS, response->return_code);

  // Wait until split time is reached
  rclcpp::Clock system_clock(RCL_SYSTEM_TIME);
  ASSERT_TRUE(
    rosbag2_test_common::wait_until_condition(
      [split_time, &system_clock]() {return system_clock.now() >= split_time;},
      std::chrono::seconds(10))
  ) << "System clock did not reach split time in time";

  // Now send a message to trigger the split
  // The publish timestamp of this message will be >= split_time
  tracked_pub->publish(*string_message);

  const bool split_happened = rosbag2_test_common::wait_until_condition(
    [&split_called]() {return split_called.load();},
    std::chrono::seconds(10));
  ASSERT_TRUE(split_happened) << "Timed out waiting for split to occur.";
}

TEST_F(RecordSrvsPublishMultipleTopicsTest, split_bagfile_topic_filter_triggers_on_tracked_topic)
{
  auto & writer = recorder_->get_writer_handle();
  auto & mock_writer = dynamic_cast<MockSequentialWriter &>(writer.get_implementation_handle());
  std::atomic<bool> split_called{false};
  rosbag2_cpp::bag_events::WriterEventCallbacks callbacks;
  callbacks.write_split_callback =
    [&split_called](rosbag2_cpp::bag_events::BagSplitInfo & /*info*/) {
      split_called.store(true);
    };
  mock_writer.add_event_callbacks(callbacks);

  auto string_message = get_messages_strings()[1];
  auto tracked_pub =
    client_node_->create_publisher<test_msgs::msg::Strings>(test_topic_,
                                                            Rosbag2QoS::UnitTestQoS());
  auto other_pub =
    client_node_->create_publisher<test_msgs::msg::Strings>(kSplitOtherTopic,
                                                            Rosbag2QoS::UnitTestQoS());

  ASSERT_TRUE(
    rosbag2_test_common::wait_until_condition(
      [&tracked_pub]() {return tracked_pub->get_subscription_count() > 0;},
      std::chrono::seconds(10)));
  ASSERT_TRUE(
    rosbag2_test_common::wait_until_condition(
      [&other_pub]() {return other_pub->get_subscription_count() > 0;},
      std::chrono::seconds(10)));

  const auto initial_tracked_count = mock_writer.get_messages_per_topic(test_topic_);

  // Publish a message on each topic to establish a receive timestamp baseline
  other_pub->publish(*string_message);
  tracked_pub->publish(*string_message);
  // Wait for messages to appear in writer buffer to make sure that recorder has received them
  ASSERT_TRUE(
    rosbag2_test_common::wait_until_condition(
      [this, &mock_writer, initial_tracked_count]() {
        return  mock_writer.get_messages_per_topic(test_topic_) > initial_tracked_count &&
               mock_writer.get_messages_per_topic(kSplitOtherTopic) > 0;
      },
      std::chrono::seconds(10))
  ) << "Failed to capture messages in time";

  auto split_time = current_sim_time_ + rclcpp::Duration(std::chrono::milliseconds(200));

  auto request = std::make_shared<SplitBagfile::Request>();
  request->split_time = split_time;
  request->split_mode = SplitBagfile::Request::SPLIT_MODE_RECEIVE_TIME;
  request->tracking_topic_name = test_topic_;
  auto response = std::make_shared<SplitBagfile::Response>();
  ASSERT_TRUE(successful_service_request<SplitBagfile>(cli_split_bagfile_, request, response));
  EXPECT_EQ(SplitBagfile::Response::RETURN_CODE_SUCCESS, response->return_code);
  EXPECT_TRUE(response->error_string.empty());

  EXPECT_FALSE(split_called.load());

  // Advance simulated time past the split time and publish a message to trigger the split from the
  // subscription callback
  publish_clock(split_time + rclcpp::Duration(std::chrono::milliseconds(10)));
  ASSERT_TRUE(
    rosbag2_test_common::wait_until_condition(
      [this]() {return recorder_->now() >= current_sim_time_;},
      std::chrono::seconds(10))
  ) << "Recorder node failed to advance to the current sim time";

  // Publish on other topic first, should not trigger split
  other_pub->publish(*string_message);
  EXPECT_FALSE(
    rosbag2_test_common::wait_until_condition(
      [&split_called]() {return split_called.load();},
      std::chrono::milliseconds(500))
  ) << "Split should not trigger for messages on other topics.";

  // Now publish on tracked topic, should trigger split
  tracked_pub->publish(*string_message);

  const bool split_happened = rosbag2_test_common::wait_until_condition(
    [&split_called]() {return split_called.load();},
  std::chrono::seconds(10));
  ASSERT_TRUE(split_happened) << "Timed out waiting for split to occur.";
}

TEST_F(RecordSrvsSimTimeTest, split_bagfile_by_node_time_respects_future_timestamp)
{
  auto & writer = recorder_->get_writer_handle();
  auto & mock_writer = dynamic_cast<MockSequentialWriter &>(writer.get_implementation_handle());
  std::atomic<bool> callback_called{false};
  rosbag2_cpp::bag_events::WriterEventCallbacks callbacks;
  callbacks.write_split_callback =
    [&callback_called](rosbag2_cpp::bag_events::BagSplitInfo & /*info*/) {
      callback_called.store(true);
    };
  mock_writer.add_event_callbacks(callbacks);

  // Schedule a split in the future
  auto request = std::make_shared<SplitBagfile::Request>();
  auto target_split_time = current_sim_time_ + rclcpp::Duration(std::chrono::milliseconds(200));
  request->split_time = target_split_time;
  request->split_mode = SplitBagfile::Request::SPLIT_MODE_NODE_TIME;
  auto response = std::make_shared<SplitBagfile::Response>();
  ASSERT_TRUE(successful_service_request<SplitBagfile>(cli_split_bagfile_, request, response));
  EXPECT_EQ(SplitBagfile::Response::RETURN_CODE_SUCCESS, response->return_code);
  EXPECT_TRUE(response->error_string.empty());

  // Confirm that the callback has not yet been called
  EXPECT_FALSE(callback_called.load());

  // Advance simulated time to the target split time to trigger the scheduled split
  publish_clock(target_split_time);
  ASSERT_TRUE(
    rosbag2_test_common::wait_until_condition(
      [this]() {return recorder_->now() >= current_sim_time_;},
      std::chrono::seconds(10))
  ) << "Recorder node failed to advance to the current sim time";

  ASSERT_TRUE(
    rosbag2_test_common::wait_until_condition(
      [&callback_called]() {return callback_called.load();},
      std::chrono::seconds(10))
  ) << "Timed out waiting for scheduled split to occur.";
}

TEST_F(RecordSrvsTest, split_bagfile_ignored_when_not_recording)
{
  auto & writer = recorder_->get_writer_handle();
  auto & mock_writer = dynamic_cast<MockSequentialWriter &>(writer.get_implementation_handle());
  std::atomic<bool> callback_called{false};
  rosbag2_cpp::bag_events::WriterEventCallbacks callbacks;
  callbacks.write_split_callback =
    [&callback_called](rosbag2_cpp::bag_events::BagSplitInfo & /*info*/) {
      callback_called.store(true);
    };
  mock_writer.add_event_callbacks(callbacks);
  auto stop_response = std::make_shared<Stop::Response>();
  ASSERT_TRUE(successful_service_request<Stop>(cli_stop_, stop_response));
  EXPECT_EQ(stop_response->return_code, 0);
  ASSERT_TRUE(mock_writer.closed_was_called());
  EXPECT_FALSE(callback_called.load());
  auto request = std::make_shared<SplitBagfile::Request>();
  auto response = std::make_shared<SplitBagfile::Response>();
  ASSERT_TRUE(successful_service_request<SplitBagfile>(cli_split_bagfile_, request, response));
  EXPECT_EQ(SplitBagfile::Response::RETURN_CODE_NOT_RECORDING, response->return_code);
  EXPECT_FALSE(callback_called.load());
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

  auto request = std::make_shared<Resume::Request>();
  auto response = std::make_shared<Resume::Response>();
  EXPECT_TRUE(successful_service_request<Resume>(cli_resume_, request, response));
  EXPECT_EQ(Resume::Response::RETURN_CODE_SUCCESS, response->return_code);
  EXPECT_TRUE(response->error_string.empty());
  EXPECT_FALSE(recorder_->is_paused());
  is_paused_response = std::make_shared<IsPaused::Response>();
  EXPECT_TRUE(successful_service_request<IsPaused>(cli_is_paused_, is_paused_response));
  EXPECT_FALSE(is_paused_response->paused);
}

TEST_F(RecordSrvsSimTimeTest, resume_by_node_time_respects_future_timestamp)
{
  recorder_->pause();
  ASSERT_TRUE(recorder_->is_paused());

  // Schedule a resume in the future
  auto request = std::make_shared<Resume::Request>();
  auto target_resume_time = current_sim_time_ + rclcpp::Duration(std::chrono::milliseconds(200));
  request->resume_time = target_resume_time;
  request->resume_mode = Resume::Request::RESUME_MODE_NODE_TIME;
  auto response = std::make_shared<Resume::Response>();
  ASSERT_TRUE(successful_service_request<Resume>(cli_resume_, request, response));
  EXPECT_EQ(Resume::Response::RETURN_CODE_SUCCESS, response->return_code);
  EXPECT_TRUE(response->error_string.empty());

  // Confirm that the recorder is still paused
  EXPECT_TRUE(recorder_->is_paused());

  // Advance time to trigger the scheduled resume
  publish_clock(target_resume_time);
  ASSERT_TRUE(
    rosbag2_test_common::wait_until_condition(
      [this]() {return recorder_->now() >= current_sim_time_;},
      std::chrono::seconds(10))
  ) << "Recorder node failed to advance to the current sim time";

  ASSERT_TRUE(
    rosbag2_test_common::wait_until_condition(
      [this]() {return !recorder_->is_paused();},
      std::chrono::seconds(10))
  ) << "Timed out waiting for scheduled resume.";
}

TEST_F(RecordSrvsSimTimeTest, resume_by_receive_time_immediate_due)
{
  auto string_message = get_messages_strings()[1];
  auto tracked_pub =
    client_node_->create_publisher<test_msgs::msg::Strings>(test_topic_,
                                                            Rosbag2QoS::UnitTestQoS());
  ASSERT_TRUE(
    rosbag2_test_common::wait_until_condition(
      [&tracked_pub]() {return tracked_pub->get_subscription_count() > 0;},
      std::chrono::seconds(10))
  );

  auto & writer = recorder_->get_writer_handle();
  auto & mock_writer = dynamic_cast<MockSequentialWriter &>(writer.get_implementation_handle());

  ASSERT_TRUE(
    rosbag2_test_common::wait_until_condition(
      [this]() {return recorder_->now() >= current_sim_time_;},
      std::chrono::seconds(10))
  ) << "Recorder node failed to advance to the current sim time";

  // Publish a message to establish a receive timestamp baseline.
  tracked_pub->publish(*string_message);
  ASSERT_TRUE(
    rosbag2_test_common::wait_until_condition(
      [&mock_writer]() {return mock_writer.get_number_of_recorded_messages() > 0;},
      std::chrono::seconds(5))
  ) << "Failed to capture message in time";

  recorder_->pause();
  ASSERT_TRUE(recorder_->is_paused());

  // Request a receive-time resume using a resume time in the past.
  // Recorded messages have current_sim_time_ as their receive timestamp.
  auto resume_time = current_sim_time_ - rclcpp::Duration(std::chrono::milliseconds(100));

  auto request = std::make_shared<Resume::Request>();
  request->resume_time = resume_time;
  request->resume_mode = Resume::Request::RESUME_MODE_RECEIVE_TIME;
  request->tracking_topic_name.clear();
  auto response = std::make_shared<Resume::Response>();
  ASSERT_TRUE(successful_service_request<Resume>(cli_resume_, request, response));
  EXPECT_EQ(Resume::Response::RETURN_CODE_SUCCESS, response->return_code);
  EXPECT_TRUE(response->error_string.empty());

  // Since the resume time is in the past, resume should happen immediately before
  // returning response to the service call.
  EXPECT_FALSE(recorder_->is_paused());
}

TEST_F(RecordSrvsTest, resume_by_publish_time_immediate_due)
{
#ifdef _WIN32
  if (std::string(rmw_get_implementation_identifier()).find("rmw_connextdds") !=
    std::string::npos)
  {
    GTEST_SKIP() << "Skipping test on Windows with Connext due to known issue with send "
      "timestamps being zero.";
  }
#endif

  rosbag2_test_common::PublicationManager pub_manager;
  auto string_message = get_messages_strings()[1];
  pub_manager.setup_publisher(test_topic_, string_message, 2);
  ASSERT_TRUE(pub_manager.wait_for_matched(test_topic_.c_str()));
  pub_manager.run_publishers();

  auto & writer = recorder_->get_writer_handle();
  auto & mock_writer = dynamic_cast<MockSequentialWriter &>(writer.get_implementation_handle());
  ASSERT_TRUE(
    rosbag2_test_common::wait_until_condition(
      [&mock_writer]() {return mock_writer.get_number_of_recorded_messages() > 1;},
      std::chrono::seconds(10))
  ) << "Failed to capture published messages in time";

  recorder_->pause();
  ASSERT_TRUE(recorder_->is_paused());

  const auto first_publish_timestamp = mock_writer.get_messages().front()->send_timestamp;
  // Request a publish-time split using a split time in the past. Note: we published at least two
  // messages, so adding 1 nanosecond to the first message's timestamp guarantees that there is at
  // least one message with a publish timestamp >= split_time
  const auto resume_time =
    rclcpp::Time(first_publish_timestamp + 1, client_node_->get_clock()->get_clock_type());

  auto request = std::make_shared<Resume::Request>();
  request->resume_time = resume_time;
  request->resume_mode = Resume::Request::RESUME_MODE_PUBLISH_TIME;
  request->tracking_topic_name = test_topic_;
  auto response = std::make_shared<Resume::Response>();
  ASSERT_TRUE(successful_service_request<Resume>(cli_resume_, request, response));
  EXPECT_EQ(Resume::Response::RETURN_CODE_SUCCESS, response->return_code);
  EXPECT_TRUE(response->error_string.empty());

  // Since the resume time is in the past, resume should happen immediately before
  // returning response to the service call.
  EXPECT_FALSE(recorder_->is_paused());
}

TEST_F(RecordSrvsTest, resume_by_publish_time_scheduled)
{
#ifdef _WIN32
  if (std::string(rmw_get_implementation_identifier()).find("rmw_connextdds") !=
    std::string::npos)
  {
    GTEST_SKIP() << "Skipping test on Windows with Connext due to known issue with send "
      "timestamps being zero.";
  }
#endif

  auto string_message = get_messages_strings()[1];
  auto tracked_pub =
    client_node_->create_publisher<test_msgs::msg::Strings>(test_topic_,
                                                            Rosbag2QoS::UnitTestQoS());
  ASSERT_TRUE(
    rosbag2_test_common::wait_until_condition(
      [&tracked_pub]() {return tracked_pub->get_subscription_count() > 0;},
      std::chrono::seconds(10)
    )
  );

  // Publish a message to establish a publish timestamp baseline.
  tracked_pub->publish(*string_message);

  auto & writer = recorder_->get_writer_handle();
  auto & mock_writer = dynamic_cast<MockSequentialWriter &>(writer.get_implementation_handle());
  ASSERT_TRUE(
    rosbag2_test_common::wait_until_condition(
      [&mock_writer]() {return mock_writer.get_number_of_recorded_messages() > 0;},
      std::chrono::seconds(10))
  ) << "Failed to capture message in time";

  recorder_->pause();
  ASSERT_TRUE(recorder_->is_paused());

  const auto last_publish_timestamp = mock_writer.get_messages().back()->send_timestamp;
  auto resume_time = rclcpp::Time(last_publish_timestamp + RCUTILS_MS_TO_NS(200), RCL_SYSTEM_TIME);

  auto request = std::make_shared<Resume::Request>();
  request->resume_time = resume_time;
  request->resume_mode = Resume::Request::RESUME_MODE_PUBLISH_TIME;
  request->tracking_topic_name = test_topic_;
  auto response = std::make_shared<Resume::Response>();
  ASSERT_TRUE(successful_service_request<Resume>(cli_resume_, request, response));
  EXPECT_EQ(Resume::Response::RETURN_CODE_SUCCESS, response->return_code);
  EXPECT_TRUE(response->error_string.empty());

  // Confirm that recorder is still paused before reaching resume time.
  EXPECT_TRUE(recorder_->is_paused());

  // Wait until resume time is reached.
  rclcpp::Clock system_clock(RCL_SYSTEM_TIME);
  ASSERT_TRUE(
    rosbag2_test_common::wait_until_condition(
      [resume_time, &system_clock]() {return system_clock.now() >= resume_time;},
      std::chrono::seconds(10))
  ) << "System clock did not reach resume time in time";

  // Publish on tracked topic to trigger resume from subscription callback.
  // The publish timestamp of this message will be >= resume_time
  tracked_pub->publish(*string_message);

  ASSERT_TRUE(
    rosbag2_test_common::wait_until_condition(
      [this]() {return !recorder_->is_paused();},
      std::chrono::seconds(10))
  ) << "Timed out waiting for publish-time resume to occur.";
}

TEST_F(RecordSrvsPublishMultipleTopicsTest, resume_topic_filter_triggers_on_tracked_topic)
{
  auto string_message = get_messages_strings()[1];
  auto tracked_pub =
    client_node_->create_publisher<test_msgs::msg::Strings>(test_topic_,
                                                            Rosbag2QoS::UnitTestQoS());
  auto other_pub =
    client_node_->create_publisher<test_msgs::msg::Strings>(kSplitOtherTopic,
                                                            Rosbag2QoS::UnitTestQoS());

  ASSERT_TRUE(
    rosbag2_test_common::wait_until_condition(
      [&tracked_pub]() {return tracked_pub->get_subscription_count() > 0;},
      std::chrono::seconds(10)));
  ASSERT_TRUE(
    rosbag2_test_common::wait_until_condition(
      [&other_pub]() {return other_pub->get_subscription_count() > 0;},
      std::chrono::seconds(10)));

  auto & writer = recorder_->get_writer_handle();
  auto & mock_writer = dynamic_cast<MockSequentialWriter &>(writer.get_implementation_handle());
  const auto initial_tracked_count = mock_writer.get_messages_per_topic(test_topic_);

  // Publish a message on each topic to establish a receive timestamp baseline.
  other_pub->publish(*string_message);
  tracked_pub->publish(*string_message);
  ASSERT_TRUE(
    rosbag2_test_common::wait_until_condition(
      [this, &mock_writer, initial_tracked_count]() {
        return  mock_writer.get_messages_per_topic(test_topic_) > initial_tracked_count &&
               mock_writer.get_messages_per_topic(kSplitOtherTopic) > 0;
      },
      std::chrono::seconds(10))
  ) << "Failed to capture messages in time";

  // Pause the recorder
  recorder_->pause();
  ASSERT_TRUE(recorder_->is_paused());

  auto request = std::make_shared<Resume::Request>();
  const auto target_resume_time = current_sim_time_ +
    rclcpp::Duration(std::chrono::milliseconds(200));
  request->resume_time = target_resume_time;
  request->resume_mode = Resume::Request::RESUME_MODE_RECEIVE_TIME;
  request->tracking_topic_name = test_topic_;
  auto response = std::make_shared<Resume::Response>();
  ASSERT_TRUE(successful_service_request<Resume>(cli_resume_, request, response));
  EXPECT_EQ(Resume::Response::RETURN_CODE_SUCCESS, response->return_code);
  EXPECT_TRUE(response->error_string.empty());

  // Advance simulated time past the resume time and publish a message on other topic first.
  publish_clock(target_resume_time + rclcpp::Duration(std::chrono::milliseconds(10)));
  ASSERT_TRUE(
    rosbag2_test_common::wait_until_condition(
      [this]() {return recorder_->now() >= current_sim_time_;},
      std::chrono::seconds(10))
  ) << "Recorder node failed to advance to the current sim time";

  // Publish on other topic first, should not trigger resume.
  other_pub->publish(*string_message);
  EXPECT_FALSE(
    rosbag2_test_common::wait_until_condition(
      [this]() {return !recorder_->is_paused();},
      std::chrono::milliseconds(500))
  ) << "Resume should not trigger for messages on non-tracked topics.";

  // Now publish on tracked topic, should trigger resume.
  tracked_pub->publish(*string_message);
  ASSERT_TRUE(
    rosbag2_test_common::wait_until_condition(
      [this]() {return !recorder_->is_paused();},
      std::chrono::seconds(10))
  ) << "Timed out waiting for receive-time resume to occur on tracked topic.";
}

TEST_F(RecordSrvsTest, resume_returns_invalid_mode_for_bad_request)
{
  recorder_->pause();
  ASSERT_TRUE(recorder_->is_paused());

  auto request = std::make_shared<Resume::Request>();
  request->resume_mode = 99;  // Invalid resume mode
  auto response = std::make_shared<Resume::Response>();
  ASSERT_TRUE(successful_service_request<Resume>(cli_resume_, request, response));

  EXPECT_EQ(Resume::Response::RETURN_CODE_INVALID_RESUME_MODE, response->return_code);
  EXPECT_FALSE(response->error_string.empty());
  EXPECT_TRUE(recorder_->is_paused());
}

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

  auto stop_response = std::make_shared<Stop::Response>();
  EXPECT_TRUE(successful_service_request<Stop>(cli_stop_, stop_response));
  EXPECT_EQ(stop_response->return_code, 0);
  EXPECT_TRUE(mock_writer.closed_was_called());

  // second stop should return unsuccessful return code since it's already stopped
  EXPECT_TRUE(successful_service_request<Stop>(cli_stop_, stop_response));
  EXPECT_EQ(stop_response->return_code, 1);

  auto record_response = std::make_shared<Record::Response>();
  EXPECT_TRUE(successful_service_request<Record>(cli_record_, record_response));
  EXPECT_EQ(0, record_response->return_code);
  EXPECT_TRUE(record_response->error_string.empty());
  EXPECT_FALSE(mock_writer.closed_was_called());

  EXPECT_TRUE(successful_service_request<Stop>(cli_stop_, stop_response));
  EXPECT_EQ(stop_response->return_code, 0);
  EXPECT_TRUE(mock_writer.closed_was_called());
}

TEST_F(RecordSrvsTest, record_with_uri)
{
  static const std::string test_uri = "file:///tmp/test_bag_with_uri";

  auto & writer = recorder_->get_writer_handle();
  auto & mock_writer = dynamic_cast<MockSequentialWriter &>(writer.get_implementation_handle());

  EXPECT_NE(mock_writer.get_storage_options().uri, test_uri);

  auto stop_response = std::make_shared<Stop::Response>();
  ASSERT_TRUE(successful_service_request<Stop>(cli_stop_, stop_response));
  EXPECT_EQ(stop_response->return_code, 0);

  auto record_request = std::make_shared<Record::Request>();
  record_request->uri = test_uri;
  auto record_response = std::make_shared<Record::Response>();
  EXPECT_TRUE(successful_service_request<Record>(cli_record_, record_request, record_response));
  EXPECT_EQ(0, record_response->return_code);
  EXPECT_TRUE(record_response->error_string.empty());

  EXPECT_EQ(mock_writer.get_storage_options().uri, test_uri);
}

TEST_F(RecordSrvsSimTimeTest, record_can_be_scheduled_in_future)
{
  auto & writer = recorder_->get_writer_handle();
  auto & mock_writer = dynamic_cast<MockSequentialWriter &>(writer.get_implementation_handle());

  auto stop_response = std::make_shared<Stop::Response>();
  ASSERT_TRUE(successful_service_request<Stop>(cli_stop_, stop_response));
  EXPECT_EQ(stop_response->return_code, 0);
  ASSERT_TRUE(mock_writer.closed_was_called());

  // Schedule a record in the future
  auto request = std::make_shared<Record::Request>();
  auto target_time = current_sim_time_ + rclcpp::Duration(std::chrono::milliseconds(200));
  request->start_time = target_time;
  auto response = std::make_shared<Record::Response>();
  ASSERT_TRUE(successful_service_request<Record>(cli_record_, request, response));
  EXPECT_EQ(SplitBagfile::Response::RETURN_CODE_SUCCESS, response->return_code);
  EXPECT_TRUE(response->error_string.empty());

  // Confirm that the writer is still closed
  EXPECT_TRUE(mock_writer.closed_was_called());

  // Advance time to trigger the scheduled record start
  publish_clock(target_time);
  ASSERT_TRUE(
    rosbag2_test_common::wait_until_condition(
      [this]() {return recorder_->now() >= current_sim_time_;},
      std::chrono::seconds(10))
  ) << "Recorder node failed to advance to the current sim time";

  // Writer should now be re-opened
  ASSERT_TRUE(
    rosbag2_test_common::wait_until_condition(
      [&mock_writer]() {return !mock_writer.closed_was_called();},
      std::chrono::seconds(10))
  ) << "Timed out waiting for scheduled record start.";
}
