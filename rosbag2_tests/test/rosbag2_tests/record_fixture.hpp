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

#ifndef ROSBAG2_TESTS__RECORD_FIXTURE_HPP_
#define ROSBAG2_TESTS__RECORD_FIXTURE_HPP_

#include <gmock/gmock.h>

#include <future>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "rcpputils/filesystem_helper.hpp"

#include "rclcpp/rclcpp.hpp"

#include "rosbag2_compression/sequential_compression_reader.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_storage/default_storage_id.hpp"
#include "rosbag2_storage/storage_filter.hpp"
#include "rosbag2_test_common/memory_management.hpp"
#include "rosbag2_test_common/temporary_directory_fixture.hpp"
#include "rosbag2_test_common/tested_storage_ids.hpp"

#include "test_msgs/msg/arrays.hpp"
#include "test_msgs/msg/basic_types.hpp"
#include "test_msgs/message_fixtures.hpp"

using namespace ::testing;  // NOLINT
using namespace std::chrono_literals;  // NOLINT
using namespace rosbag2_test_common;  // NOLINT


class RecordFixture : public ParametrizedTemporaryDirectoryFixture
{
public:
  void SetUp() override
  {
    auto bag_name = get_test_name() + "_" + GetParam();
    root_bag_path_ = rcpputils::fs::path(temporary_dir_path_) / bag_name;

    // Clean up potentially leftover bag files.
    // There may be leftovers if the system reallocates a temp directory
    // used by a previous test execution and the test did not have a clean exit.
    rcpputils::fs::remove_all(root_bag_path_);
  }

  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown() override
  {
    rcpputils::fs::remove_all(root_bag_path_);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  std::string get_base_record_command() const
  {
    return "ros2 bag record --storage " + GetParam() + " --output " + root_bag_path_.string();
  }

  std::string get_test_name() const
  {
    const auto * test_info = UnitTest::GetInstance()->current_test_info();
    std::string test_name = test_info->name();
    // Replace any slashes in the test name, since it is used in paths
    std::replace(test_name.begin(), test_name.end(), '/', '_');
    return test_name;
  }

  std::string get_bag_file_name(int split_index) const
  {
    std::stringstream bag_file_name;
    bag_file_name << get_test_name() << "_" << GetParam() << "_" << split_index;

    return bag_file_name.str();
  }

  rcpputils::fs::path get_compressed_bag_file_path(int split_index)
  {
    return rcpputils::fs::path(get_bag_file_path(split_index).string() + ".zstd");
  }

  rcpputils::fs::path get_bag_file_path(int split_index)
  {
    return root_bag_path_ / get_relative_bag_file_path(split_index);
  }

  rcpputils::fs::path get_relative_bag_file_path(int split_index)
  {
    const auto storage_id = GetParam();
    return rcpputils::fs::path(
      rosbag2_test_common::bag_filename_for_storage_id(
        get_bag_file_name(split_index), storage_id));
  }

  void wait_for_metadata(std::chrono::duration<float> timeout = std::chrono::seconds(5)) const
  {
    rosbag2_storage::MetadataIo metadata_io;
    const auto start_time = std::chrono::steady_clock::now();
    const auto bag_path = root_bag_path_.string();

    while (std::chrono::steady_clock::now() - start_time < timeout && rclcpp::ok()) {
      if (metadata_io.metadata_file_exists(bag_path)) {
        return;
      }
      std::this_thread::sleep_for(50ms);
    }
    ASSERT_EQ(metadata_io.metadata_file_exists(bag_path), true)
      << "Could not find metadata file.";
  }

  void wait_for_storage_file(std::chrono::duration<float> timeout = std::chrono::seconds(10))
  {
    const auto storage_path = get_bag_file_path(0);
    const auto start_time = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start_time < timeout && rclcpp::ok()) {
      if (storage_path.exists()) {
        return;
      }
      std::this_thread::sleep_for(50ms);  // wait a bit to not query constantly
    }
    ASSERT_EQ(storage_path.exists(), true)
      << "Could not find storage file: \"" << storage_path.string() << "\"";
  }

  template<typename MessageT>
  std::vector<std::shared_ptr<MessageT>> get_messages_for_topic(
    const std::string & topic, const std::string & compression_plugin = "")
  {
    auto filter = rosbag2_storage::StorageFilter{};
    filter.topics.push_back(topic);

    std::unique_ptr<rosbag2_cpp::Reader> reader;
    if (compression_plugin.empty()) {
      reader = std::make_unique<rosbag2_cpp::Reader>();
    } else {
      reader = std::make_unique<rosbag2_cpp::Reader>(
        std::make_unique<rosbag2_compression::SequentialCompressionReader>());
    }
    reader->open(root_bag_path_.string());
    reader->set_filter(filter);

    auto messages = std::vector<std::shared_ptr<MessageT>>{};
    while (reader->has_next()) {
      auto msg = reader->read_next();
      messages.push_back(memory_management_.deserialize_message<MessageT>(msg->serialized_data));
    }
    return messages;
  }

  std::string get_serialization_format_for_topic(const std::string & topic_name)
  {
    auto reader = rosbag2_cpp::Reader{};
    reader.open(root_bag_path_.string());
    auto topics_and_types = reader.get_all_topics_and_types();
    auto topic_it = std::find_if(
      topics_and_types.begin(), topics_and_types.end(),
      [&topic_name](const auto & tm) {
        return topic_name == tm.first.name;
      });
    return topic_it->serialization_format;
  }

  void finalize_metadata_kludge(
    int expected_splits = 0,
    const std::string & compression_format = "",
    const std::string & compression_mode = "")
  {
    // TODO(ros-tooling): Find out how to correctly send a Ctrl-C signal on Windows
    // This is necessary as the process is killed hard on Windows and doesn't write a metadata file
  #ifdef _WIN32
    rosbag2_storage::BagMetadata metadata{};
    metadata.storage_identifier = rosbag2_storage::get_default_storage_id();
    for (int i = 0; i <= expected_splits; i++) {
      rcpputils::fs::path bag_file_path;
      if (!compression_format.empty()) {
        bag_file_path = get_bag_file_path(i);
      } else {
        bag_file_path = get_compressed_bag_file_path(i);
      }

      if (rcpputils::fs::exists(bag_file_path)) {
        metadata.relative_file_paths.push_back(bag_file_path.string());
      }
    }
    metadata.duration = std::chrono::nanoseconds(0);
    metadata.starting_time =
      std::chrono::time_point<std::chrono::high_resolution_clock>(std::chrono::nanoseconds(0));
    metadata.message_count = 0;
    metadata.compression_mode = compression_mode;
    metadata.compression_format = compression_format;

    rosbag2_storage::MetadataIo metadata_io;
    metadata_io.write_metadata(root_bag_path_.string(), metadata);
  #else
    (void)expected_splits;
    (void)compression_format;
    (void)compression_mode;
  #endif
  }

  // relative path to the root of the bag file.
  rcpputils::fs::path root_bag_path_;

  MemoryManagement memory_management_;
};

#endif  // ROSBAG2_TESTS__RECORD_FIXTURE_HPP_
