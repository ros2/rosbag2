// Copyright 2023, Apex.AI.
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

#include <filesystem>
#include <memory>
#include <string>

#include "rosbag2_storage/storage_factory.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "rosbag2_test_common/memory_management.hpp"
#include "rosbag2_test_common/tested_storage_ids.hpp"
#include "rosbag2_test_common/temporary_directory_fixture.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;  // NOLINT
using namespace ::testing;  // NOLINT
using namespace rosbag2_test_common;  // NOLINT

namespace fs = std::filesystem;

class Rosbag2StorageAPITests : public rosbag2_test_common::ParametrizedTemporaryDirectoryFixture
{
public:
  Rosbag2StorageAPITests()
  {
    memory_management_ = std::make_unique<MemoryManagement>();
  }

  void SetUp() override
  {
    auto bag_name = get_test_name() + "_" + GetParam();
    root_bag_path_ = fs::path(temporary_dir_path_) / bag_name;

    // Clean up potentially leftover bag files.
    // There may be leftovers if the system reallocates a temp directory
    // used by a previous test execution and the test did not have a clean exit.
    fs::remove_all(root_bag_path_);
  }

  void TearDown() override
  {
    fs::remove_all(root_bag_path_);
  }

  static std::string get_test_name()
  {
    const auto * test_info = UnitTest::GetInstance()->current_test_info();
    std::string test_name = test_info->name();
    // Replace any slashes in the test name, since it is used in paths
    std::replace(test_name.begin(), test_name.end(), '/', '_');
    return test_name;
  }

  std::vector<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>>
  prepare_serialized_messages(
    const std::vector<std::string> & topics, const size_t num_msgs_per_topic = 5,
    std::chrono::duration<double> dt = 0.01s) const
  {
    std::vector<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>> serialized_messages;
    for (const auto & topic : topics) {
      std::chrono::duration<double> timestamp{dt};
      for (size_t msg_idx = 0; msg_idx < num_msgs_per_topic; msg_idx++) {
        auto std_string_msg = std::make_shared<std_msgs::msg::String>();
        std::stringstream ss;
        for (int i = 0; i < 5; i++) {
          ss << topic << "_long_string_message_" << msg_idx + 1 << "_repeat_";
        }
        std_string_msg->data = ss.str();

        auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
        bag_message->serialized_data = memory_management_->serialize_message(std_string_msg);
        bag_message->recv_timestamp =
          std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp).count();
        bag_message->send_timestamp = bag_message->recv_timestamp;
        bag_message->topic_name = topic;
        timestamp += dt;
        serialized_messages.push_back(bag_message);
      }
    }
    return serialized_messages;
  }

  static void create_topics(
    std::shared_ptr<rosbag2_storage::storage_interfaces::ReadWriteInterface> & rw_storage,
    const std::vector<std::string> & topics)
  {
    for (size_t topic_idx = 0; topic_idx < topics.size(); topic_idx++) {
      const std::string type_description_hash = "type_hash_" + std::to_string(topic_idx);
      const rosbag2_storage::MessageDefinition msg_definition = {
        "std_msgs/msg/String", "ros2msg", "string data", type_description_hash};

      rosbag2_storage::TopicMetadata topic_metadata = {
        0u,
        topics[topic_idx],
        "std_msgs/msg/String",
        "cdr",
        {rclcpp::QoS(1)},
        type_description_hash
      };
      rw_storage->create_topic(topic_metadata, msg_definition);
    }
  }

  fs::path root_bag_path_;
  std::unique_ptr<MemoryManagement> memory_management_;
};

TEST_P(Rosbag2StorageAPITests, get_bagfile_size_read_write_interface)
{
  const std::string FILE_EXTENSION = (GetParam() == "mcap") ? ".mcap" : ".db3";
  fs::path full_bagfile_path = root_bag_path_;
  full_bagfile_path.replace_extension(FILE_EXTENSION);

  rosbag2_storage::StorageFactory factory{};
  rosbag2_storage::StorageOptions options{};
  options.uri = root_bag_path_.generic_string();
  options.storage_id = GetParam();

  std::shared_ptr<rosbag2_storage::storage_interfaces::ReadWriteInterface> rw_storage =
    factory.open_read_write(options);

  std::vector<std::string> topics = {"topic1_topic1", "topic2_topic2"};
  auto serialized_messages = prepare_serialized_messages(topics, 500 * 20);
  create_topics(rw_storage, topics);

  rw_storage->write(serialized_messages);
  uint64_t storage_bagfile_size = rw_storage->get_bagfile_size();

  size_t fs_bagfile_size = fs::file_size(full_bagfile_path);
  auto tolerance = static_cast<size_t>(fs_bagfile_size * 0.001);  // tolerance = 0.1%

  size_t filesize_difference =
    std::abs(static_cast<int64_t>(storage_bagfile_size) - static_cast<int64_t>(fs_bagfile_size));

  EXPECT_LE(filesize_difference, tolerance) << " tolerance = " << tolerance << std::endl <<
    " filesize_difference = " << filesize_difference << std::endl <<
    " bagfile_size_from_storage = " << storage_bagfile_size;

  // Write messages one more time to make sure that storage_bagfile_size updating with each write
  rw_storage->write(serialized_messages);
  storage_bagfile_size = rw_storage->get_bagfile_size();

  fs_bagfile_size = fs::file_size(full_bagfile_path);
  tolerance = static_cast<size_t>(fs_bagfile_size * 0.001);  // tolerance = 0.1%

  filesize_difference =
    std::abs(static_cast<int64_t>(storage_bagfile_size) - static_cast<int64_t>(fs_bagfile_size));

  EXPECT_LE(filesize_difference, tolerance) << " tolerance = " << tolerance << std::endl <<
    " filesize_difference = " << filesize_difference << std::endl <<
    " bagfile_size_from_storage = " << storage_bagfile_size;
}

INSTANTIATE_TEST_SUITE_P(
  ParametrizedStorageAPITests,
  Rosbag2StorageAPITests,
  ValuesIn(rosbag2_test_common::kTestedStorageIDs)
);
