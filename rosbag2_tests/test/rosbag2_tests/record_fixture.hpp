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

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/utilities.hpp"

#include "rosbag2_compression/sequential_compression_reader.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_storage/metadata_io.hpp"
#include "rosbag2_storage/storage_filter.hpp"
#include "rosbag2_test_common/bag_files_helpers.hpp"
#include "rosbag2_test_common/memory_management.hpp"
#include "rosbag2_test_common/temporary_directory_fixture.hpp"
#include "rosbag2_test_common/tested_storage_ids.hpp"

using namespace ::testing;  // NOLINT
using namespace std::chrono_literals;  // NOLINT
using namespace rosbag2_test_common;  // NOLINT

namespace fs = std::filesystem;

class RecordFixture : public ParametrizedTemporaryDirectoryFixture
{
public:
  void SetUp() override
  {
    auto bag_name = get_test_name() + "_" + GetParam();
    root_bag_path_ = std::filesystem::path(temporary_dir_path_) / bag_name;

    // Clean up potentially leftover bag files.
    // There may be leftovers if the system reallocates a temp directory
    // used by a previous test execution and the test did not have a clean exit.
    std::error_code ec;
    std::filesystem::remove_all(root_bag_path_, ec);
  }

  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown() override
  {
    // Do not throw out of TearDown: an exception here is followed by the fixture
    // destructor, and the two together have taken down the whole test binary.
    std::error_code ec;
    std::filesystem::remove_all(root_bag_path_, ec);
    EXPECT_FALSE(ec) << "Failed to remove " << root_bag_path_.generic_string() << ": " <<
      ec.message();
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  [[nodiscard]] std::string get_base_record_command() const
  {
    return "ros2 bag record --storage " + GetParam() + " --output " +
           root_bag_path_.generic_string();
  }

  static std::string get_test_name()
  {
    const auto * test_info = UnitTest::GetInstance()->current_test_info();
    std::string test_name = test_info->name();
    // Replace any slashes in the test name, since it is used in paths
    std::replace(test_name.begin(), test_name.end(), '/', '_');
    // The recorder names each file "<split>_<bag name>_<timestamp>.<ext>" inside the
    // bag directory, so the test name occurs twice in a bag file's path.  With the
    // longest test names that exceeds MAX_PATH (260) on Windows, where the recorder
    // then creates files the test process cannot see.  Keep the name short and unique.
    constexpr size_t max_length = 32;
    if (test_name.size() > max_length) {
      std::stringstream hash;
      hash << std::hex << std::setw(8) << std::setfill('0') <<
        static_cast<uint32_t>(std::hash<std::string>{}(test_name));
      test_name = test_name.substr(0, max_length) + "_" + hash.str();
    }
    return test_name;
  }

  [[nodiscard]] std::filesystem::path get_compressed_bag_file_path(int split_index) const
  {
    return load_metadata_and_get_bag_file_path(split_index).generic_string() + ".zstd";
  }

  void wait_for_metadata(std::chrono::duration<float> timeout = std::chrono::seconds(10)) const
  {
    rosbag2_test_common::wait_for_metadata(root_bag_path_, timeout);
  }

  [[nodiscard]] std::filesystem::path load_metadata_and_get_bag_file_path(int split_index = 0) const
  {
    // Read metadata to get actual file name (timestamped format)
    rosbag2_storage::MetadataIo metadata_io;
    const auto bag_path = root_bag_path_.generic_string();
    rosbag2_test_common::wait_for_metadata(root_bag_path_, std::chrono::seconds(10));
    auto metadata = metadata_io.read_metadata(bag_path);
    return get_bag_file_path_from_metadata(root_bag_path_, metadata, split_index);
  }

  void wait_for_storage_file(std::chrono::duration<float> timeout = std::chrono::seconds(10)) const
  {
    rosbag2_test_common::wait_for_storage_files(root_bag_path_, 1, timeout);
  }

  void wait_for_storage_files(
    size_t count, std::chrono::duration<float> timeout = std::chrono::seconds(30)) const
  {
    rosbag2_test_common::wait_for_storage_files(root_bag_path_, count, timeout);
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
    reader->open(root_bag_path_.generic_string());
    reader->set_filter(filter);

    auto messages = std::vector<std::shared_ptr<MessageT>>{};
    while (reader->has_next()) {
      auto msg = reader->read_next();
      messages.push_back(memory_management_.deserialize_message<MessageT>(msg->serialized_data));
    }
    return messages;
  }

  [[nodiscard]] std::string get_serialization_format_for_topic(const std::string & topic_name) const
  {
    auto reader = rosbag2_cpp::Reader{};
    reader.open(root_bag_path_.generic_string());
    auto topics_and_types = reader.get_all_topics_and_types();
    auto topic_it = std::find_if(
      topics_and_types.begin(), topics_and_types.end(),
      [&topic_name](const auto & tm) {
        return topic_name == tm.name;
      });
    if (topic_it == topics_and_types.end()) {
      ADD_FAILURE() << "Topic " << topic_name << " not found in bag metadata";
      return "";
    }
    return topic_it->serialization_format;
  }

  // relative path to the root of the bag file.
  std::filesystem::path root_bag_path_;

  MemoryManagement memory_management_;
};

#endif  // ROSBAG2_TESTS__RECORD_FIXTURE_HPP_
