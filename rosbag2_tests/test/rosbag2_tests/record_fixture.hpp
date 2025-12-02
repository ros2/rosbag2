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

#include <filesystem>
#include <future>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

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
    std::filesystem::remove_all(root_bag_path_);
  }

  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown() override
  {
    std::filesystem::remove_all(root_bag_path_);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  std::string get_base_record_command() const
  {
    return "ros2 bag record --storage " + GetParam() + " --output " +
           root_bag_path_.generic_string();
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

  std::filesystem::path get_compressed_bag_file_path(int split_index)
  {
    // For timestamped filename format, get the actual bag file path and add compression extension
    try {
      auto actual_path = get_actual_bag_file_path(split_index);
      return std::filesystem::path(actual_path.generic_string() + ".zstd");
    } catch (const std::runtime_error &) {
      // Fallback to old method if metadata is not available
      return std::filesystem::path(get_bag_file_path(split_index).generic_string() + ".zstd");
    }
  }

  std::filesystem::path get_bag_file_path(int split_index)
  {
    return root_bag_path_ / get_relative_bag_file_path(split_index);
  }

  std::filesystem::path get_relative_bag_file_path(int split_index)
  {
    const auto storage_id = GetParam();
    return std::filesystem::path(
      rosbag2_test_common::bag_filename_for_storage_id(
        get_bag_file_name(split_index), storage_id));
  }

  void wait_for_metadata(std::chrono::duration<float> timeout = std::chrono::seconds(5)) const
  {
    rosbag2_storage::MetadataIo metadata_io;
    const auto start_time = std::chrono::steady_clock::now();
    const auto bag_path = root_bag_path_.generic_string();

    while (std::chrono::steady_clock::now() - start_time < timeout && rclcpp::ok()) {
      if (metadata_io.metadata_file_exists(bag_path)) {
        return;
      }
      std::this_thread::sleep_for(50ms);
    }
    ASSERT_EQ(metadata_io.metadata_file_exists(bag_path), true)
      << "Could not find metadata file.";
  }

  std::filesystem::path get_actual_bag_file_path(int split_index = 0) const
  {
    // Read metadata to get actual file name (timestamped format)
    rosbag2_storage::MetadataIo metadata_io;
    const auto bag_path = root_bag_path_.generic_string();

    // Wait for metadata file to exist
    const auto start_time = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(10)) {
      if (metadata_io.metadata_file_exists(bag_path)) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    if (!metadata_io.metadata_file_exists(bag_path)) {
      throw std::runtime_error("Metadata file not found for bag: " + bag_path);
    }

    auto metadata = metadata_io.read_metadata(bag_path);
    if (metadata.files.empty() || split_index >= static_cast<int>(metadata.files.size())) {
      throw std::runtime_error("No file found at split_index " + std::to_string(split_index));
    }

    // Return full path to the actual file
    return fs::path(bag_path) / metadata.files[split_index].path;
  }

  void wait_for_storage_file(std::chrono::duration<float> timeout = std::chrono::seconds(10))
  {
    // For timestamped filename format, wait for any bag file (.db3 or .mcap) to appear in directory
    const auto start_time = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start_time < timeout && rclcpp::ok()) {
      if (std::filesystem::exists(root_bag_path_) &&
        std::filesystem::is_directory(root_bag_path_))
      {
        for (const auto & entry : std::filesystem::directory_iterator(root_bag_path_)) {
          if (entry.is_regular_file()) {
            const auto & path = entry.path();
            const auto extension = path.extension();
            // Check for bag files (.db3, .mcap) or compressed files
            if (extension == ".db3" || extension == ".mcap" ||
              path.filename().generic_string().find(".db3.") != std::string::npos ||
              path.filename().generic_string().find(".mcap.") != std::string::npos)
            {
              return;  // Found a bag file
            }
          }
        }
      }
      std::this_thread::sleep_for(50ms);
    }

    // If we get here, no bag file was found
    ASSERT_TRUE(false) << "Could not find any storage file in directory: \"" <<
      root_bag_path_.generic_string() << "\" within " << timeout.count() << " seconds";
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

  std::string get_serialization_format_for_topic(const std::string & topic_name)
  {
    auto reader = rosbag2_cpp::Reader{};
    reader.open(root_bag_path_.generic_string());
    auto topics_and_types = reader.get_all_topics_and_types();
    auto topic_it = std::find_if(
      topics_and_types.begin(), topics_and_types.end(),
      [&topic_name](const auto & tm) {
        return topic_name == tm.name;
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

    // For timestamped filename format, scan directory for actual files instead of guessing paths
    if (std::filesystem::exists(root_bag_path_) && std::filesystem::is_directory(root_bag_path_)) {
      for (const auto & entry : std::filesystem::directory_iterator(root_bag_path_)) {
        if (entry.is_regular_file()) {
          const auto & path = entry.path();
          const auto extension = path.extension();
          // Include db3, mcap files and compressed files
          if (extension == ".db3" || extension == ".mcap" ||
            (!compression_format.empty() &&
            path.string().find(compression_format) != std::string::npos))
          {
            metadata.relative_file_paths.push_back(path.filename().generic_string());

            // Create file info
            rosbag2_storage::FileInformation file_info;
            file_info.path = path.filename().generic_string();
            file_info.starting_time = std::chrono::time_point<std::chrono::high_resolution_clock>(
              std::chrono::nanoseconds(0));
            file_info.duration = std::chrono::nanoseconds(0);
            file_info.message_count = 0;
            metadata.files.push_back(file_info);
          }
        }
      }
    } else {
      // Fallback to old logic if directory doesn't exist
      for (int i = 0; i <= expected_splits; i++) {
        try {
          std::filesystem::path bag_file_path = get_actual_bag_file_path(i);
          if (!compression_format.empty()) {
            bag_file_path = std::filesystem::path(bag_file_path.generic_string() + ".zstd");
          }

          if (std::filesystem::exists(bag_file_path)) {
            metadata.relative_file_paths.push_back(bag_file_path.generic_string());
          }
        } catch (const std::runtime_error &) {
          // Skip if file path cannot be determined
          continue;
        }
      }
    }
    metadata.duration = std::chrono::nanoseconds(0);
    metadata.starting_time =
      std::chrono::time_point<std::chrono::high_resolution_clock>(std::chrono::nanoseconds(0));
    metadata.message_count = 0;
    metadata.compression_mode = compression_mode;
    metadata.compression_format = compression_format;

    rosbag2_storage::MetadataIo metadata_io;
    metadata_io.write_metadata(root_bag_path_.generic_string(), metadata);
  #else
    (void)expected_splits;
    (void)compression_format;
    (void)compression_mode;
  #endif
  }

  // relative path to the root of the bag file.
  std::filesystem::path root_bag_path_;

  MemoryManagement memory_management_;
};

#endif  // ROSBAG2_TESTS__RECORD_FIXTURE_HPP_
