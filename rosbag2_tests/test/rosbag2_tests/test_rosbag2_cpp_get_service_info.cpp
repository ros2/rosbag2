// Copyright 2023 Sony Group Corporation and Apex.AI, Inc.
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
#include <string>

#include "rosbag2_cpp/info.hpp"
#include "rosbag2_cpp/writer.hpp"

#include "rosbag2_test_common/client_manager.hpp"
#include "rosbag2_test_common/publication_manager.hpp"
#include "rosbag2_test_common/temporary_directory_fixture.hpp"
#include "rosbag2_test_common/tested_storage_ids.hpp"
#include "rosbag2_test_common/wait_for.hpp"

#include "rosbag2_transport/recorder.hpp"

#include "test_msgs/message_fixtures.hpp"
#include "test_msgs/msg/basic_types.hpp"
#include "test_msgs/srv/basic_types.hpp"

class SequentialWriterForTest : public rosbag2_cpp::writers::SequentialWriter
{
public:
  size_t get_number_of_written_messages()
  {
    size_t num_messages = 0;
    for (const auto & file : metadata_.files) {
      num_messages += file.message_count;
    }
    return num_messages;
  }
};

class Rosbag2CPPGetServiceInfoTest
  : public rosbag2_test_common::ParametrizedTemporaryDirectoryFixture
{
public:
  void SetUp() override
  {
    auto bag_name = get_test_name() + "_" + GetParam();
    root_bag_path_ = std::filesystem::path(temporary_dir_path_) / bag_name;

    // Clean up potentially leftover bag files.
    // There may be leftovers if the system reallocates a temp directory
    // used by a previous test execution and the test did not have a clean exit.
    std::filesystem::remove_all(root_bag_path_);
  }

  void TearDown() override
  {
    std::filesystem::remove_all(root_bag_path_);
  }

  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  template<class T>
  void start_async_spin(T node)
  {
    node_spinner_future_ = std::async(
      std::launch::async,
      [node, this]() -> void {
        rclcpp::executors::SingleThreadedExecutor exec;
        exec.add_node(node);
        while (!exit_from_node_spinner_) {
          exec.spin_some();
        }
      });
  }

  void stop_spinning()
  {
    exit_from_node_spinner_ = true;
    if (node_spinner_future_.valid()) {
      node_spinner_future_.wait();
    }
  }

  std::string get_test_name() const
  {
    const auto * test_info = UnitTest::GetInstance()->current_test_info();
    std::string test_name = test_info->name();
    // Replace any slashes in the test name, since it is used in paths
    std::replace(test_name.begin(), test_name.end(), '/', '_');
    return test_name;
  }

  std::string get_bag_file_name(int split_index = 0) const
  {
    const auto storage_id = GetParam();
    std::stringstream bag_file_name;
    bag_file_name << get_test_name() << "_" << storage_id << "_" << split_index;
    return rosbag2_test_common::bag_filename_for_storage_id(bag_file_name.str(), storage_id);
  }

  std::string get_bag_path_str() const
  {
    return root_bag_path_.generic_string();
  }

  bool wait_for_subscriptions(
    const rosbag2_transport::Recorder & recorder,
    const std::vector<std::string> && topic_names,
    std::chrono::duration<double> timeout = std::chrono::seconds(5))
  {
    using clock = std::chrono::system_clock;
    auto start = clock::now();
    bool ready = false;
    while (!ready && (clock::now() - start) < timeout) {
      const auto & subscriptions = recorder.subscriptions();
      ready = true;
      for (const auto & topic_name : topic_names) {
        if (subscriptions.find(topic_name) == subscriptions.end()) {
          ready = false;
          break;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
    return ready;
  }

  // relative path to the root of the bag file.
  std::filesystem::path root_bag_path_;
  std::future<void> node_spinner_future_;
  std::atomic_bool exit_from_node_spinner_{false};
};

TEST_P(Rosbag2CPPGetServiceInfoTest, get_service_info_for_bag_with_topics_only) {
  const std::string storage_id = GetParam();
  const auto bag_path_str = get_bag_path_str();
  {
    // Create an empty bag with default storage
    rosbag2_cpp::Writer writer;
    rosbag2_storage::StorageOptions storage_options;
    storage_options.storage_id = storage_id;
    storage_options.uri = bag_path_str;
    writer.open(storage_options);
    test_msgs::msg::BasicTypes msg;
    writer.write(msg, "test_topic", rclcpp::Time{});
  }

  rosbag2_cpp::Info info;
  std::vector<std::shared_ptr<rosbag2_cpp::rosbag2_service_info_t>> ret_service_infos;
  const std::string recorded_bag_uri = bag_path_str + "/" + get_bag_file_name();
  ASSERT_NO_THROW(ret_service_infos = info.read_service_info(recorded_bag_uri, storage_id)) <<
    recorded_bag_uri;

  EXPECT_TRUE(ret_service_infos.empty());
}

TEST_P(Rosbag2CPPGetServiceInfoTest, get_service_info_for_bag_with_services_only) {
  const std::string storage_id = GetParam();
  const std::string bag_path_str = get_bag_path_str();

  auto service_client_manager =
    std::make_shared<rosbag2_test_common::ClientManager<test_msgs::srv::BasicTypes>>(
    "test_service");

  std::unique_ptr<rosbag2_cpp::writer_interfaces::BaseWriterInterface> writer_impl =
    std::make_unique<SequentialWriterForTest>();
  auto writer = std::make_unique<rosbag2_cpp::Writer>(std::move(writer_impl));

  rosbag2_storage::StorageOptions storage_options;
  storage_options.storage_id = storage_id;
  storage_options.uri = bag_path_str;
  rosbag2_transport::RecordOptions record_options =
  {false, true, false, {}, {}, {}, {"/rosout"}, {}, {}, "cdr", 100ms};
  auto recorder = std::make_shared<rosbag2_transport::Recorder>(
    std::move(writer), storage_options, record_options);
  recorder->record();

  start_async_spin(recorder);
  auto cleanup_process_handle = rcpputils::make_scope_exit(
    [&]() {stop_spinning();});

  ASSERT_TRUE(service_client_manager->wait_for_srvice_to_be_ready());
  ASSERT_TRUE(wait_for_subscriptions(*recorder, {"/test_service/_service_event"}));

  constexpr size_t num_service_requests = 3;
  for (size_t i = 0; i < num_service_requests; i++) {
    ASSERT_TRUE(service_client_manager->send_request());
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  auto & writer_ref = recorder->get_writer_handle();
  auto & recorder_writer =
    dynamic_cast<SequentialWriterForTest &>(writer_ref.get_implementation_handle());

  // By default, only client introspection is enabled.
  // For one request, service event topic get 2 messages.
  size_t expected_messages = num_service_requests * 2;
  auto ret = rosbag2_test_common::wait_until_condition(
    [&recorder_writer, &expected_messages]() {
      return recorder_writer.get_number_of_written_messages() >= expected_messages;
    },
    std::chrono::seconds(5));
  EXPECT_TRUE(ret) << "Failed to capture " << expected_messages << " expected messages in time";

  recorder->stop();
  stop_spinning();
  cleanup_process_handle.cancel();

  rosbag2_cpp::Info info;
  std::vector<std::shared_ptr<rosbag2_cpp::rosbag2_service_info_t>> ret_service_infos;

  const std::string recorded_bag_uri = bag_path_str + "/" + get_bag_file_name();
  ASSERT_NO_THROW(ret_service_infos = info.read_service_info(recorded_bag_uri, storage_id)) <<
    recorded_bag_uri;

  ASSERT_EQ(ret_service_infos.size(), 1);
  EXPECT_EQ(ret_service_infos[0]->name, "/test_service");
  EXPECT_EQ(ret_service_infos[0]->type, "test_msgs/srv/BasicTypes");
  EXPECT_EQ(ret_service_infos[0]->request_count, num_service_requests);
  EXPECT_EQ(ret_service_infos[0]->response_count, num_service_requests);
  EXPECT_EQ(ret_service_infos[0]->serialization_format, "cdr");
}

TEST_P(Rosbag2CPPGetServiceInfoTest, get_service_info_for_bag_with_topics_and_services) {
  const std::string storage_id = GetParam();
  const std::string bag_path_str = get_bag_path_str();

  // Prepare service/client
  auto service_client_manager1 =
    std::make_shared<rosbag2_test_common::ClientManager<test_msgs::srv::BasicTypes>>(
    "test_service1");
  auto service_client_manager2 =
    std::make_shared<rosbag2_test_common::ClientManager<test_msgs::srv::BasicTypes>>(
    "test_service2");

  ASSERT_TRUE(service_client_manager1->wait_for_srvice_to_be_ready());
  ASSERT_TRUE(service_client_manager2->wait_for_srvice_to_be_ready());

  rosbag2_test_common::PublicationManager pub_manager;
  auto message = get_messages_strings()[0];
  pub_manager.setup_publisher("test_topic1", message, 1);
  pub_manager.setup_publisher("test_topic2", message, 1);

  std::unique_ptr<rosbag2_cpp::writer_interfaces::BaseWriterInterface> writer_impl =
    std::make_unique<SequentialWriterForTest>();
  auto writer = std::make_unique<rosbag2_cpp::Writer>(std::move(writer_impl));
  rosbag2_storage::StorageOptions storage_options;
  storage_options.storage_id = storage_id;
  storage_options.uri = bag_path_str;
  rosbag2_transport::RecordOptions record_options =
  {true, true, false, {}, {}, {}, {"/rosout"}, {}, {}, "cdr", 100ms};
  auto recorder = std::make_shared<rosbag2_transport::Recorder>(
    std::move(writer), storage_options, record_options);
  recorder->record();

  start_async_spin(recorder);
  auto cleanup_process_handle = rcpputils::make_scope_exit(
    [&]() {stop_spinning();});

  ASSERT_TRUE(
    wait_for_subscriptions(
      *recorder,
      {"/test_service1/_service_event",
        "/test_service2/_service_event",
        "/test_topic1",
        "/test_topic2"}
    )
  );

  constexpr size_t num_service_requests = 2;
  for (size_t i = 0; i < num_service_requests; i++) {
    ASSERT_TRUE(service_client_manager1->send_request());
    ASSERT_TRUE(service_client_manager2->send_request());
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  pub_manager.run_publishers();

  auto & writer_ref = recorder->get_writer_handle();
  auto & recorder_writer =
    dynamic_cast<SequentialWriterForTest &>(writer_ref.get_implementation_handle());

  // By default, only client introspection is enabled.
  // For one request, service event topic get 2 messages.
  size_t expected_messages = num_service_requests * 2 + 2;
  auto ret = rosbag2_test_common::wait_until_condition(
    [&recorder_writer, &expected_messages]() {
      return recorder_writer.get_number_of_written_messages() >= expected_messages;
    },
    std::chrono::seconds(5));
  EXPECT_TRUE(ret) << "Failed to capture " << expected_messages << " expected messages in time";

  recorder->stop();
  stop_spinning();
  cleanup_process_handle.cancel();

  rosbag2_cpp::Info info;
  std::vector<std::shared_ptr<rosbag2_cpp::rosbag2_service_info_t>> ret_service_infos;

  const std::string recorded_bag_uri = bag_path_str + "/" + get_bag_file_name();
  ASSERT_NO_THROW(ret_service_infos = info.read_service_info(recorded_bag_uri, storage_id)) <<
    recorded_bag_uri;
  ASSERT_EQ(ret_service_infos.size(), 2);
  if (ret_service_infos[0]->name == "/test_service2") {
    EXPECT_EQ(ret_service_infos[1]->name, "/test_service1");
  } else {
    EXPECT_EQ(ret_service_infos[0]->name, "/test_service1");
    EXPECT_EQ(ret_service_infos[1]->name, "/test_service2");
  }
  for (const auto & service_info : ret_service_infos) {
    EXPECT_EQ(service_info->request_count, num_service_requests);
    EXPECT_EQ(service_info->response_count, num_service_requests);
    EXPECT_EQ(service_info->type, "test_msgs/srv/BasicTypes");
    EXPECT_EQ(service_info->serialization_format, "cdr");
  }
}

INSTANTIATE_TEST_SUITE_P(
  TestInfoGetServiceInfo,
  Rosbag2CPPGetServiceInfoTest,
  ::testing::ValuesIn(rosbag2_test_common::kTestedStorageIDs)
);
