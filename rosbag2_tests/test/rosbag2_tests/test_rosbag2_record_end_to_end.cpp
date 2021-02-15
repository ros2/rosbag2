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

#include <memory>
#include <string>
#include <unordered_set>

#include "rclcpp/rclcpp.hpp"
#include "rcpputils/filesystem_helper.hpp"
#include "rcutils/filesystem.h"
#include "rosbag2_compression/zstd_decompressor.hpp"
#include "rosbag2_storage/metadata_io.hpp"
#include "rosbag2_test_common/subscription_manager.hpp"
#include "rosbag2_test_common/process_execution_helpers.hpp"

#include "record_fixture.hpp"

namespace
{
/**
 * Construct an instance of test_msgs::msg::Strings populated with the base_message repeated until it fits the requested size.
 *
 * \param base_message is the String to repeat.
 * \param max_message_size_bytes is the size of the message in bytes.
 * \return an instance of test_msgs::msg::Strings that is the requested size.
 */
std::shared_ptr<test_msgs::msg::Strings> create_string_message(
  const std::string & base_message,
  const int max_message_size_bytes)
{
  const auto base_message_size = base_message.size() * sizeof(std::string::value_type);
  const auto iterations = max_message_size_bytes / static_cast<int>(base_message_size);

  std::stringstream message_str;
  for (int i = 0; i < iterations; ++i) {
    message_str << base_message;
  }

  auto message = get_messages_strings()[0];
  message->string_value = message_str.str();

  return message;
}
}  // namespace

#ifndef _WIN32
TEST_F(RecordFixture, record_end_to_end_test_with_zstd_file_compression) {
  constexpr const char topic_name[] = "/test_topic";

  auto message = get_messages_strings()[0];
  message->string_value = "test";
  size_t expected_test_messages = 100;

  std::stringstream cmd;
  cmd << "ros2 bag record" <<
    " --compression-mode file" <<
    " --compression-format zstd" <<
    " --output " << root_bag_path_.string() <<
    " " << topic_name;

  auto process_handle = start_execution(cmd.str());
  wait_for_db();

  pub_man_.run_scoped_publisher(
    topic_name,
    message,
    50ms,
    expected_test_messages);

  stop_execution(process_handle);

  wait_for_metadata();

  const auto compressed_bag_file_path = get_compressed_bag_file_path(0);

  ASSERT_TRUE(compressed_bag_file_path.exists()) <<
    "Expected compressed bag file path: \"" <<
    compressed_bag_file_path.string() << "\" to exist!";

  rosbag2_compression::ZstdDecompressor decompressor;

  const auto decompressed_uri = decompressor.decompress_uri(compressed_bag_file_path.string());
  const auto database_path = get_bag_file_path(0).string();

  ASSERT_EQ(decompressed_uri, database_path) <<
    "Expected decompressed URI to be same as uncompressed bag file path!";
  ASSERT_TRUE(rcpputils::fs::exists(database_path)) <<
    "Expected decompressed first bag file to exist!";

  auto test_topic_messages = get_messages_for_topic<test_msgs::msg::Strings>(topic_name);
  EXPECT_GT(test_topic_messages.size(), 0u);

  for (const auto & message : test_topic_messages) {
    EXPECT_EQ(message->string_value, "test");
  }
}
#endif

TEST_F(RecordFixture, record_end_to_end_test) {
  auto message = get_messages_strings()[0];
  message->string_value = "test";
  size_t expected_test_messages = 3;

  auto wrong_message = get_messages_strings()[0];
  wrong_message->string_value = "wrong_content";

  auto process_handle = start_execution(
    "ros2 bag record --output " + root_bag_path_.string() + " /test_topic");
  wait_for_db();

  pub_man_.add_publisher("/test_topic", message, expected_test_messages);
  pub_man_.add_publisher("/wrong_topic", wrong_message);

  const auto database_path = get_bag_file_path(0).string();

  rosbag2_storage_plugins::SqliteWrapper db{
    database_path, rosbag2_storage::storage_interfaces::IOFlag::READ_ONLY};
  pub_man_.run_publishers(
    [this, &db](const std::string & topic_name) {
      return count_stored_messages(db, topic_name);
    });

  stop_execution(process_handle);

  // TODO(Martin-Idel-SI): Find out how to correctly send a Ctrl-C signal on Windows
  // This is necessary as the process is killed hard on Windows and doesn't write a metadata file
#ifdef _WIN32
  rosbag2_storage::BagMetadata metadata{};
  metadata.version = 1;
  metadata.storage_identifier = "sqlite3";
  metadata.relative_file_paths = {get_bag_file_path(0).string()};
  metadata.duration = std::chrono::nanoseconds(0);
  metadata.starting_time =
    std::chrono::time_point<std::chrono::high_resolution_clock>(std::chrono::nanoseconds(0));
  metadata.message_count = 0;
  rosbag2_storage::MetadataIo metadata_io;
  metadata_io.write_metadata(root_bag_path_.string(), metadata);
#endif

  wait_for_metadata();
  auto test_topic_messages = get_messages_for_topic<test_msgs::msg::Strings>("/test_topic");
  EXPECT_THAT(test_topic_messages, SizeIs(Ge(expected_test_messages)));

  for (const auto & message : test_topic_messages) {
    EXPECT_EQ(message->string_value, "test");
  }

  EXPECT_THAT(get_rwm_format_for_topic("/test_topic", db), Eq(rmw_get_serialization_format()));

  auto wrong_topic_messages = get_messages_for_topic<test_msgs::msg::BasicTypes>("/wrong_topic");
  EXPECT_THAT(wrong_topic_messages, IsEmpty());
}

// TODO(zmichaels11): Fix and enable this test on Windows.
// This tests depends on the ability to read the metadata file.
// Stopping the process on Windows does a hard kill and the metadata file is not written.
#ifndef _WIN32
TEST_F(RecordFixture, record_end_to_end_with_splitting_metadata_contains_all_topics) {
  constexpr const int bagfile_split_size = 4 * 1024 * 1024;  // 4MB.
  std::stringstream command;
  command << "ros2 bag record" <<
    " --output " << root_bag_path_.string() <<
    " --max-bag-size " << bagfile_split_size <<
    " -a";
  auto process_handle = start_execution(command.str());
  wait_for_db();

  constexpr const char first_topic_name[] = "/test_topic0";
  constexpr const char second_topic_name[] = "/test_topic1";
  constexpr const int expected_splits = 4;
  {
    constexpr const char message_str[] = "Test";
    constexpr const int message_size = 1024 * 1024;  // 1MB
    const auto message = create_string_message(message_str, message_size);
    constexpr const int message_count = bagfile_split_size * expected_splits / message_size;
    constexpr const int message_batch_size = message_count / 2;

    pub_man_.run_scoped_publisher(
      first_topic_name,
      message,
      50ms,
      message_batch_size);

    pub_man_.run_scoped_publisher(
      second_topic_name,
      message,
      50ms,
      message_batch_size);
  }

  stop_execution(process_handle);

  wait_for_metadata();
  rosbag2_storage::MetadataIo metadataIo;
  const auto metadata = metadataIo.read_metadata(root_bag_path_.string());
  // Verify at least 2 topics are in the metadata.
  // There may be more if the test system is noisy.
  EXPECT_GT(metadata.topics_with_message_count.size(), 1u);

  // Transform the topic_names_with_message_count into an unordered set of topic_names.
  std::unordered_set<std::string> topic_names;
  for (const auto & topic : metadata.topics_with_message_count) {
    topic_names.insert(topic.topic_metadata.name);
  }

  // Verify that first_topic_name and second_topic_name are both in the metadata.
  EXPECT_NE(topic_names.end(), topic_names.find(first_topic_name));
  EXPECT_NE(topic_names.end(), topic_names.find(second_topic_name));
}
#endif

TEST_F(RecordFixture, record_end_to_end_with_splitting_bagsize_split_is_at_least_specified_size) {
  constexpr const char topic_name[] = "/test_topic";
  constexpr const int bagfile_split_size = 4 * 1024 * 1024;  // 4MB.
  std::stringstream command;
  command << "ros2 bag record " <<
    " --output " << root_bag_path_.string() <<
    " --max-bag-size " << bagfile_split_size <<
    " " << topic_name;
  auto process_handle = start_execution(command.str());
  wait_for_db();

  constexpr const int expected_splits = 4;
  {
    constexpr const char message_str[] = "Test";
    constexpr const int message_size = 512 * 1024;  // 512KB
    const auto message = create_string_message(message_str, message_size);
    constexpr const int message_count = bagfile_split_size * expected_splits / message_size;

    pub_man_.run_scoped_publisher(
      topic_name,
      message,
      50ms,
      message_count);
  }

  stop_execution(process_handle);

  rosbag2_storage::MetadataIo metadata_io;

#ifdef _WIN32
  {
    rosbag2_storage::BagMetadata metadata;
    metadata.version = 4;
    metadata.storage_identifier = "sqlite3";

    // Loop until expected_splits in case it split or the bagfile doesn't exist.
    for (int i = 0; i < expected_splits; ++i) {
      const auto bag_file_path = get_relative_bag_file_path(i);
      if (rcpputils::fs::exists(root_bag_path_ / bag_file_path)) {
        metadata.relative_file_paths.push_back(bag_file_path.string());
      }
    }

    metadata_io.write_metadata(root_bag_path_.string(), metadata);
  }
#endif

  wait_for_metadata();
  const auto metadata = metadata_io.read_metadata(root_bag_path_.string());
  const auto actual_splits = static_cast<int>(metadata.relative_file_paths.size());

  // TODO(zmichaels11): Support reliable sync-to-disk for more accurate splits.
  // The only guarantee with splits right now is that they will not occur until
  // a bagfile is at least the specified max_bagfile_size.
  EXPECT_GT(actual_splits, 0);

  // Don't include the last bagfile since it won't be full
  for (int i = 0; i < actual_splits - 1; ++i) {
    const auto bagfile_path = root_bag_path_ / rcpputils::fs::path{metadata.relative_file_paths[i]};
    ASSERT_TRUE(bagfile_path.exists()) <<
      "Expected bag file: \"" << bagfile_path.string() << "\" to exist.";

    const auto actual_split_size = static_cast<int>(bagfile_path.file_size());
    // Actual size is guaranteed to be >= bagfile_split size
    EXPECT_LT(bagfile_split_size, actual_split_size);
  }
}

TEST_F(RecordFixture, record_end_to_end_with_splitting_max_size_not_reached) {
  constexpr const char topic_name[] = "/test_topic";
  constexpr const int bagfile_split_size = 4 * 1024 * 1024;  // 4MB.
  std::stringstream command;
  command << "ros2 bag record " <<
    " --output " << root_bag_path_.string() <<
    " --max-bag-size " << bagfile_split_size <<
    " " << topic_name;
  auto process_handle = start_execution(command.str());
  wait_for_db();

  {
    constexpr const int message_size = 512 * 1024;  // 512KB
    constexpr const char message_str[] = "Test";
    const auto message = create_string_message(message_str, message_size);
    // only fill the bagfile halfway
    constexpr const int message_count = bagfile_split_size / message_size / 2;

    pub_man_.run_scoped_publisher(
      topic_name,
      message,
      50ms,
      message_count);
  }

  stop_execution(process_handle);

  rosbag2_storage::MetadataIo metadata_io;

// TODO(zmichaels11): Remove when stop_execution properly SIGINT on Windows.
// This is required since stop_execution hard kills the proces on Windows,
// which prevents the metadata from being written.
#ifdef _WIN32
  {
    rosbag2_storage::BagMetadata metadata;
    metadata.version = 4;
    metadata.storage_identifier = "sqlite3";
    metadata.relative_file_paths = {get_bag_file_name(0) + ".db3"};
    metadata_io.write_metadata(root_bag_path_.string(), metadata);
  }
#endif

  wait_for_metadata();
  const auto metadata = metadata_io.read_metadata(root_bag_path_.string());

  // Check that there's only 1 bagfile and that it exists.
  ASSERT_EQ(1u, metadata.relative_file_paths.size());
  const auto bagfile_path = root_bag_path_ / rcpputils::fs::path{metadata.relative_file_paths[0]};
  ASSERT_TRUE(bagfile_path.exists()) <<
    "Expected bag file: \"" << bagfile_path.string() << "\" to exist.";

  // Check that the next bagfile does not exist.
  const auto next_bag_file = get_bag_file_path(1);
  EXPECT_FALSE(next_bag_file.exists()) << "Expected next bag file: \"" <<
    next_bag_file.string() << "\" to not exist!";
}

TEST_F(RecordFixture, record_end_to_end_with_splitting_splits_bagfile) {
  constexpr const char topic_name[] = "/test_topic";
  constexpr const int bagfile_split_size = 4 * 1024 * 1024;  // 4MB.

  std::stringstream command;
  command << "ros2 bag record" <<
    " --output " << root_bag_path_.string() <<
    " --max-bag-size " << bagfile_split_size <<
    " " << topic_name;
  auto process_handle = start_execution(command.str());
  wait_for_db();

  constexpr const int expected_splits = 4;
  {
    constexpr const char message_str[] = "Test";
    constexpr const int message_size = 1024 * 1024;  // 1MB
    // string message from test_msgs
    const auto message = create_string_message(message_str, message_size);
    constexpr const int message_count = bagfile_split_size * expected_splits / message_size;

    pub_man_.run_scoped_publisher(
      topic_name,
      message,
      50ms,
      message_count);
  }

  stop_execution(process_handle);

  rosbag2_storage::MetadataIo metadata_io;

// TODO(zmichaels11): Remove when stop_execution properly SIGINT on Windows.
// This is required since stop_execution hard kills the proces on Windows,
// which prevents the metadata from being written.
#ifdef _WIN32
  {
    rosbag2_storage::BagMetadata metadata;
    metadata.version = 4;
    metadata.storage_identifier = "sqlite3";

    for (int i = 0; i < expected_splits; ++i) {
      const auto rel_bag_file_path = get_relative_bag_file_path(i);

      // There is no guarantee that the bagfile split expected_split times
      // due to possible io sync delays. Instead, assert that the bagfile split
      // at least once
      if (rcpputils::fs::exists(root_bag_path_ / rel_bag_file_path)) {
        metadata.relative_file_paths.push_back(rel_bag_file_path.string());
      }
    }

    ASSERT_GE(metadata.relative_file_paths.size(), 1) << "Bagfile never split!";
    metadata_io.write_metadata(root_bag_path_.string(), metadata);
  }
#endif

  wait_for_metadata();
  const auto metadata = metadata_io.read_metadata(root_bag_path_.string());

  for (const auto & rel_path : metadata.relative_file_paths) {
    auto path = root_bag_path_ / rcpputils::fs::path(rel_path);
    EXPECT_TRUE(rcpputils::fs::exists(path));
  }
}

TEST_F(RecordFixture, record_end_to_end_test_with_zstd_file_compression_compresses_files) {
  constexpr const char topic_name[] = "/test_topic";
  constexpr const int bagfile_split_size = 4 * 1024 * 1024;  // 4MB.

  std::stringstream command;
  command << "ros2 bag record" <<
    " --output " << root_bag_path_.string() <<
    " --max-bag-size " << bagfile_split_size <<
    " --compression-mode file" <<
    " --compression-format zstd"
    " " << topic_name;

  auto process_handle = start_execution(command.str());
  wait_for_db();

  constexpr const int expected_splits = 4;
  {
    constexpr const char message_str[] = "Test";
    constexpr const int message_size = 1024 * 1024;  // 1MB
    // string message from test_msgs
    const auto message = create_string_message(message_str, message_size);
    constexpr const int message_count = bagfile_split_size * expected_splits / message_size;

    pub_man_.run_scoped_publisher(
      topic_name,
      message,
      50ms,
      message_count);
  }

  stop_execution(process_handle);

  rosbag2_storage::MetadataIo metadata_io;

  // TODO(zmichaels11): Remove when stop_execution properly SIGINT on Windows.
  // This is required since stop_execution hard kills the proces on Windows,
  // which prevents the metadata from being written.
  #ifdef _WIN32
  {
    rosbag2_storage::BagMetadata metadata;
    metadata.version = 3;
    metadata.storage_identifier = "sqlite3";
    metadata.compression_mode = "file";
    metadata.compression_format = "zstd";

    for (int i = 0; i < expected_splits; ++i) {
      const auto compressed_bag_path = get_compressed_bag_file_path(i);

      // There is no guarantee that the bagfile split expected_split times
      // due to possible io sync delays. Instead, assert that the bagfile
      // split at least once.
      if (compressed_bag_path.exists()) {
        metadata.relative_file_paths.push_back(compressed_bag_path.string());
      }
    }

    ASSERT_GE(metadata.relative_file_paths.size(), 1) << "Bagfile never split!";
    metadata_io.write_metadata(root_bag_path_.string(), metadata);
  }
  #endif

  wait_for_metadata();
  const auto metadata = metadata_io.read_metadata(root_bag_path_.string());

  for (const auto & path : metadata.relative_file_paths) {
    const auto file_path = rcpputils::fs::path{path};

    EXPECT_TRUE(file_path.exists()) << "File: \"" <<
      file_path.string() << "\" does not exist!";
    EXPECT_EQ(file_path.extension().string(), ".zstd") << "File :\"" <<
      file_path.string() << "\" does not have proper \".zstd\" extension!";
  }
}

TEST_F(RecordFixture, record_fails_gracefully_if_bag_already_exists) {
  auto database_path = _SRC_RESOURCES_DIR_PATH;  // variable defined in CMakeLists.txt

  internal::CaptureStderr();
  auto exit_code =
    execute_and_wait_until_completion("ros2 bag record --output cdr_test -a", database_path);
  auto error_output = internal::GetCapturedStderr();

  EXPECT_THAT(exit_code, Eq(EXIT_FAILURE));
  EXPECT_THAT(error_output, HasSubstr("Output folder 'cdr_test' already exists"));
}

TEST_F(RecordFixture, record_fails_if_both_all_and_topic_list_is_specified) {
  internal::CaptureStderr();
  auto exit_code =
    execute_and_wait_until_completion("ros2 bag record -a /some_topic", temporary_dir_path_);
  auto error_output = internal::GetCapturedStderr();

  EXPECT_THAT(exit_code, Eq(EXIT_FAILURE));
  EXPECT_THAT(error_output, HasSubstr("Can not specify topics and -a at the same time."));
}

TEST_F(RecordFixture, record_fails_gracefully_if_plugin_for_given_encoding_does_not_exist) {
  internal::CaptureStderr();
  auto exit_code =
    execute_and_wait_until_completion("ros2 bag record -a -f some_rmw", temporary_dir_path_);
  auto error_output = internal::GetCapturedStderr();

  EXPECT_THAT(exit_code, Eq(EXIT_SUCCESS));
  EXPECT_THAT(
    error_output, HasSubstr("Requested converter for format 'some_rmw' does not exist"));
}

TEST_F(RecordFixture, record_end_to_end_test_with_cache) {
  auto max_cache_size = 10;
  auto topic_name = "/rosbag2_cache_test_topic";

  auto message = get_messages_strings()[0];
  message->string_value = "test";
  size_t expected_test_messages = 33;

  auto process_handle = start_execution(
    "ros2 bag record --output " + root_bag_path_.string() + " " + topic_name + " " +
    "--max-cache-size " + std::to_string(max_cache_size));
  wait_for_db();

  pub_man_.add_publisher(topic_name, message, expected_test_messages);

  const auto database_path = get_bag_file_path(0).string();

  rosbag2_storage_plugins::SqliteWrapper db{
    database_path, rosbag2_storage::storage_interfaces::IOFlag::READ_ONLY};
  pub_man_.run_publishers(
    [this, &db](const std::string & topic_name) {
      return count_stored_messages(db, topic_name);
    });

  stop_execution(process_handle);

  // TODO(Martin-Idel-SI): Find out how to correctly send a Ctrl-C signal on Windows
  // This is necessary as the process is killed hard on Windows and doesn't write a metadata file
#ifdef _WIN32
  rosbag2_storage::BagMetadata metadata{};
  metadata.version = 1;
  metadata.storage_identifier = "sqlite3";
  metadata.relative_file_paths = {get_bag_file_path(0).string()};
  metadata.duration = std::chrono::nanoseconds(0);
  metadata.starting_time =
    std::chrono::time_point<std::chrono::high_resolution_clock>(std::chrono::nanoseconds(0));
  metadata.message_count = 0;
  rosbag2_storage::MetadataIo metadata_io;
  metadata_io.write_metadata(root_bag_path_.string(), metadata);
#endif

  wait_for_metadata();
  auto test_topic_messages =
    get_messages_for_topic<test_msgs::msg::Strings>(topic_name);
  EXPECT_THAT(test_topic_messages, SizeIs(Ge(expected_test_messages)));
}

#ifndef _WIN32
TEST_F(RecordFixture, rosbag2_record_and_play_multiple_topics_with_filter) {
  constexpr const int bagfile_split_size = 4 * 1024 * 1024;  // 4MB.

  std::stringstream command_record;
  command_record << "ros2 bag record" <<
    " --output " << root_bag_path_.string() <<
    " --max-bag-size " << bagfile_split_size <<
    " -a";
  auto process_handle = start_execution(command_record.str());

  wait_for_db();

  constexpr const char first_topic_name[] = "/test_topic0";
  constexpr const char second_topic_name[] = "/test_topic1";
  constexpr const int expected_splits = 4;
  constexpr const char message_str[] = "Test";
  constexpr const int message_size = 1024 * 1024;  // 1MB
  constexpr const int message_count = bagfile_split_size * expected_splits / message_size;
  const auto message = create_string_message(message_str, message_size);
  constexpr const int message_batch_size = message_count / 2;
  {
    pub_man_.run_scoped_publisher(
      first_topic_name,
      message,
      50ms,
      message_batch_size);

    pub_man_.run_scoped_publisher(
      second_topic_name,
      message,
      50ms,
      message_batch_size);
  }

  stop_execution(process_handle);

  wait_for_metadata();

  auto sub = std::make_unique<SubscriptionManager>();
  sub->add_subscription<test_msgs::msg::Strings>(
    first_topic_name,
    message_batch_size);
  auto sub_future = sub->spin_subscriptions();

  std::stringstream command_play;
  command_play << "ros2 bag play " << root_bag_path_.string() << " --topics " <<
    second_topic_name;

  int exit_code = execute_and_wait_until_completion(command_play.str(), ".");
  EXPECT_THAT(exit_code, Eq(EXIT_SUCCESS));

  sub_future.wait_for(1s);

  auto first_topic_msgs = sub->get_received_messages<test_msgs::msg::Strings>(first_topic_name);

  EXPECT_THAT(first_topic_msgs, SizeIs(Eq(0u)));

  // stops thread
  sub->add_subscription<test_msgs::msg::Strings>(first_topic_name, 0);
}
#endif
