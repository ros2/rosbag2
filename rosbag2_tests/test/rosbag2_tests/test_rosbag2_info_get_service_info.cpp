// Copyright 2023 Sony Group Corporation.
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
#include <string>

#include "rosbag2_cpp/info.hpp"
#include "rosbag2_cpp/writer.hpp"

#include "rosbag2_test_common/client_manager.hpp"
#include "rosbag2_test_common/process_execution_helpers.hpp"
#include "rosbag2_test_common/publication_manager.hpp"
#include "rosbag2_test_common/temporary_directory_fixture.hpp"

#include "test_msgs/message_fixtures.hpp"
#include "test_msgs/msg/basic_types.hpp"
#include "test_msgs/srv/basic_types.hpp"

class GetServiceInfoTest : public rosbag2_test_common::TemporaryDirectoryFixture
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

  rcpputils::fs::path get_bag_path()
  {
    return rcpputils::fs::path(temporary_dir_path_) /
           UnitTest::GetInstance()->current_test_info()->name();
  }

  const std::string get_bag_file_name(int split_index, std::string storage_id)
  {
    if (storage_id == "mcap") {
      return std::string(UnitTest::GetInstance()->current_test_info()->name()) +
             "_" + std::to_string(split_index) + ".mcap";
    } else if (storage_id == "sqlite3") {
      return std::string(UnitTest::GetInstance()->current_test_info()->name()) +
             "_" + std::to_string(split_index) + ".db3";
    } else {
      throw std::runtime_error("Invalid storage id : \"" + storage_id + "\"");
    }
  }

  void wait_for_storage_file(
    std::string file_path,
    std::chrono::duration<float> timeout = std::chrono::seconds(10))
  {
    auto check_file_path = rcpputils::fs::path(file_path);
    const auto start_time = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start_time < timeout) {
      if (check_file_path.exists()) {
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    ASSERT_EQ(check_file_path.exists(), true)
      << "Could not find storage file: \"" << check_file_path.string() << "\"";
  }

  void topics_and_services_bag_test(std::string storage_id);
};

TEST_F(GetServiceInfoTest, only_topics_bag_test) {
  const std::string storage_id = "mcap";
  const auto bag_path = get_bag_path();

  {
    // Create an empty bag with default storage
    rosbag2_cpp::Writer writer;
    rosbag2_storage::StorageOptions storage_options;
    storage_options.storage_id = storage_id;
    storage_options.uri = bag_path.string();
    writer.open(storage_options);
    test_msgs::msg::BasicTypes msg;
    writer.write(msg, "test_topic", rclcpp::Time{});
  }
  std::string first_storage_file_path;
  {
    rosbag2_storage::MetadataIo metadata_io;
    auto metadata = metadata_io.read_metadata(bag_path.string());
    first_storage_file_path = (bag_path / metadata.relative_file_paths[0]).string();
  }

  rosbag2_cpp::Info info;
  std::vector<std::shared_ptr<rosbag2_cpp::rosbag2_service_info_t>> ret_service_info;

  ASSERT_NO_THROW(ret_service_info = info.read_service_info(first_storage_file_path, storage_id));

  EXPECT_TRUE(ret_service_info.empty());
}

TEST_F(GetServiceInfoTest, only_services_bag_test) {
  const std::string storage_id = "mcap";
  const auto bag_path = get_bag_path();
  const std::string record_cmd = "ros2 bag record --all-services -o " + bag_path.string();
  std::string bag_filename;

  ASSERT_NO_THROW(bag_filename = get_bag_file_name(0, storage_id));

  auto service_client_manager =
    std::make_shared<rosbag2_test_common::ClientManager<test_msgs::srv::BasicTypes>>(
    "test_service");

  if (!service_client_manager->check_service_ready()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    ASSERT_TRUE(service_client_manager->check_service_ready());
  }

  auto record_process = start_execution(record_cmd);
  auto cleanup_process_handle = rcpputils::make_scope_exit(
    [record_process]() {
      stop_execution(record_process);
    });

  wait_for_storage_file(bag_path.string() + "/" + bag_filename);

  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  ASSERT_TRUE(service_client_manager->send_request());
  ASSERT_TRUE(service_client_manager->send_request());
  ASSERT_TRUE(service_client_manager->send_request());
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  stop_execution(record_process);
  cleanup_process_handle.cancel();

  rosbag2_cpp::Info info;
  std::vector<std::shared_ptr<rosbag2_cpp::rosbag2_service_info_t>> ret_service_info;

  ASSERT_NO_THROW(
    ret_service_info =
    info.read_service_info(bag_path.string() + "/" + bag_filename, storage_id));

  ASSERT_EQ(ret_service_info.size(), 1);
  EXPECT_EQ(ret_service_info[0]->name, "/test_service");
  EXPECT_EQ(ret_service_info[0]->type, "test_msgs/srv/BasicTypes");
  EXPECT_EQ(ret_service_info[0]->request_count, 3);
  EXPECT_EQ(ret_service_info[0]->response_count, 3);
  EXPECT_EQ(ret_service_info[0]->serialization_format, "cdr");
}

void GetServiceInfoTest::topics_and_services_bag_test(std::string storage_id)
{
  const auto bag_path = get_bag_path();
  const std::string record_cmd =
    "ros2 bag record --all-topics --all-services -s " + storage_id + " -o " + bag_path.string();
  std::string bag_filename;

  ASSERT_NO_THROW(bag_filename = get_bag_file_name(0, storage_id));

  // Prepare service/client
  auto service_client_manager1 =
    std::make_shared<rosbag2_test_common::ClientManager<test_msgs::srv::BasicTypes>>(
    "test_service1");

  if (!service_client_manager1->check_service_ready()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    ASSERT_TRUE(service_client_manager1->check_service_ready());
  }

  auto service_client_manager2 =
    std::make_shared<rosbag2_test_common::ClientManager<test_msgs::srv::BasicTypes>>(
    "test_service2");

  if (!service_client_manager2->check_service_ready()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    ASSERT_TRUE(service_client_manager2->check_service_ready());
  }

  rosbag2_test_common::PublicationManager pub_manager;
  auto message = get_messages_strings()[0];
  pub_manager.setup_publisher("test_topic1", message, 1);
  pub_manager.setup_publisher("test_topic2", message, 1);

  auto record_process = start_execution(record_cmd);
  auto cleanup_process_handle = rcpputils::make_scope_exit(
    [record_process]() {
      stop_execution(record_process);
    });

  wait_for_storage_file(bag_path.string() + "/" + bag_filename);

  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  ASSERT_TRUE(service_client_manager1->send_request());
  ASSERT_TRUE(service_client_manager1->send_request());
  ASSERT_TRUE(service_client_manager2->send_request());
  pub_manager.run_publishers();
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  stop_execution(record_process);
  cleanup_process_handle.cancel();

  rosbag2_cpp::Info info;
  std::vector<std::shared_ptr<rosbag2_cpp::rosbag2_service_info_t>> ret_service_info;

  ASSERT_NO_THROW(
    ret_service_info =
    info.read_service_info(bag_path.string() + "/" + bag_filename, storage_id));

  ASSERT_EQ(ret_service_info.size(), 2);
  if (ret_service_info[0]->name == "/test_service2") {
    EXPECT_EQ(ret_service_info[0]->request_count, 1);
    EXPECT_EQ(ret_service_info[0]->response_count, 1);
    EXPECT_EQ(ret_service_info[1]->name, "/test_service1");
    EXPECT_EQ(ret_service_info[1]->request_count, 2);
    EXPECT_EQ(ret_service_info[1]->response_count, 2);
  } else {
    EXPECT_EQ(ret_service_info[0]->name, "/test_service1");
    EXPECT_EQ(ret_service_info[0]->request_count, 2);
    EXPECT_EQ(ret_service_info[0]->response_count, 2);
    EXPECT_EQ(ret_service_info[1]->name, "/test_service2");
    EXPECT_EQ(ret_service_info[1]->request_count, 1);
    EXPECT_EQ(ret_service_info[1]->response_count, 1);
  }
  EXPECT_EQ(ret_service_info[0]->type, "test_msgs/srv/BasicTypes");
  EXPECT_EQ(ret_service_info[0]->serialization_format, "cdr");
  EXPECT_EQ(ret_service_info[1]->type, "test_msgs/srv/BasicTypes");
  EXPECT_EQ(ret_service_info[1]->serialization_format, "cdr");
}

TEST_F(GetServiceInfoTest, topics_and_services_mcap_bag_test) {
  topics_and_services_bag_test("mcap");
}

TEST_F(GetServiceInfoTest, topics_and_services_sqlite3_bag_test) {
  topics_and_services_bag_test("sqlite3");
}
