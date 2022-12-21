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
#include "rcpputils/scope_exit.hpp"
#include "rcutils/filesystem.h"
#include "rosbag2_compression_zstd/zstd_decompressor.hpp"
#include "rosbag2_storage/metadata_io.hpp"
#include "rosbag2_test_common/publication_manager.hpp"
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
TEST_P(RecordFixture, record_end_to_end_test_with_zstd_file_compression) {
  constexpr const char topic_name[] = "/test_topic";
  const auto compression_format = "zstd";

  auto message = get_messages_strings()[0];
  message->string_value = "test";
  size_t expected_test_messages = 100;

  rosbag2_test_common::PublicationManager pub_manager;
  pub_manager.setup_publisher(topic_name, message, expected_test_messages);

  std::stringstream cmd;
  cmd << get_base_record_command() <<
    " --compression-mode file" <<
    " --compression-format " << compression_format <<
    " --max-cache-size 0" <<
    " " << topic_name;

  auto process_handle = start_execution(cmd.str());
  auto cleanup_process_handle = rcpputils::make_scope_exit(
    [process_handle]() {
      stop_execution(process_handle);
    });

  ASSERT_TRUE(pub_manager.wait_for_matched(topic_name)) <<
    "Expected find rosbag subscription";
  wait_for_storage_file();

  pub_manager.run_publishers();

  stop_execution(process_handle);
  cleanup_process_handle.cancel();

  wait_for_metadata();

  const auto compressed_bag_file_path = get_compressed_bag_file_path(0);

  ASSERT_TRUE(compressed_bag_file_path.exists()) <<
    "Expected compressed bag file path: \"" <<
    compressed_bag_file_path.string() << "\" to exist!";

  rosbag2_compression_zstd::ZstdDecompressor decompressor;

  const auto decompressed_uri = decompressor.decompress_uri(compressed_bag_file_path.string());
  const auto bag_path = get_bag_file_path(0).string();

  ASSERT_EQ(decompressed_uri, bag_path) <<
    "Expected decompressed URI to be same as uncompressed bag file path!";
  ASSERT_TRUE(rcpputils::fs::exists(bag_path)) <<
    "Expected decompressed first bag file to exist!";

  auto test_topic_messages = get_messages_for_topic<test_msgs::msg::Strings>(
    topic_name, compression_format);
  EXPECT_GT(test_topic_messages.size(), 0u);

  for (const auto & message : test_topic_messages) {
    EXPECT_EQ(message->string_value, "test");
  }
}
#endif

TEST_P(RecordFixture, record_end_to_end_test) {
  auto message = get_messages_strings()[0];
  message->string_value = "test";
  size_t expected_test_messages = 3;

  auto unrecorded_message = get_messages_strings()[0];
  unrecorded_message->string_value = "unrecorded_content";

  rosbag2_test_common::PublicationManager pub_manager;
  pub_manager.setup_publisher("/test_topic", message, expected_test_messages);
  pub_manager.setup_publisher("/unrecorded_topic", unrecorded_message, 3);

  auto process_handle = start_execution(
    get_base_record_command() + " --max-cache-size 0 /test_topic");
  auto cleanup_process_handle = rcpputils::make_scope_exit(
    [process_handle]() {
      stop_execution(process_handle);
    });

  ASSERT_TRUE(pub_manager.wait_for_matched("/test_topic")) <<
    "Expected find rosbag subscription";

  wait_for_storage_file();

  pub_manager.run_publishers();

  stop_execution(process_handle);
  cleanup_process_handle.cancel();

  finalize_metadata_kludge();
  wait_for_metadata();
  auto test_topic_messages = get_messages_for_topic<test_msgs::msg::Strings>("/test_topic");
  EXPECT_THAT(test_topic_messages, SizeIs(Ge(expected_test_messages)));

  for (const auto & message : test_topic_messages) {
    EXPECT_EQ(message->string_value, "test");
  }

  EXPECT_THAT(
    get_serialization_format_for_topic("/test_topic"),
    Eq(rmw_get_serialization_format()));

  auto unrecorded_topic_messages = get_messages_for_topic<test_msgs::msg::BasicTypes>(
    "/unrecorded_topic");
  EXPECT_THAT(unrecorded_topic_messages, IsEmpty());
}

TEST_P(RecordFixture, record_end_to_end_test_start_paused) {
  auto message = get_messages_strings()[0];
  message->string_value = "test";

  rosbag2_test_common::PublicationManager pub_manager;
  pub_manager.setup_publisher("/test_topic", message, 10);

  auto process_handle = start_execution(
    get_base_record_command() + " --max-cache-size 0 --start-paused /test_topic");
  auto cleanup_process_handle = rcpputils::make_scope_exit(
    [process_handle]() {
      stop_execution(process_handle);
    });

  ASSERT_TRUE(pub_manager.wait_for_matched("/test_topic")) <<
    "Expected find rosbag subscription";

  wait_for_storage_file();

  pub_manager.run_publishers();

  stop_execution(process_handle);
  cleanup_process_handle.cancel();

  finalize_metadata_kludge();
  wait_for_metadata();
  auto test_topic_messages = get_messages_for_topic<test_msgs::msg::Strings>("/test_topic");
  EXPECT_THAT(test_topic_messages, IsEmpty());
}

TEST_P(RecordFixture, record_end_to_end_exits_gracefully_on_sigterm) {
  const std::string topic_name = "/test_sigterm";
  auto message = get_messages_strings()[0];
  message->string_value = "test";
  rosbag2_test_common::PublicationManager pub_manager;
  pub_manager.setup_publisher(topic_name, message, 10);
  auto process_handle = start_execution(
    get_base_record_command() + " " + topic_name);
  wait_for_storage_file();
  pub_manager.run_publishers();
  stop_execution(process_handle, SIGTERM);
  wait_for_metadata();
}

// TODO(zmichaels11): Fix and enable this test on Windows.
// This tests depends on the ability to read the metadata file.
// Stopping the process on Windows does a hard kill and the metadata file is not written.
#ifndef _WIN32
TEST_P(RecordFixture, record_end_to_end_with_splitting_metadata_contains_all_topics) {
  constexpr const int bagfile_split_size = 4 * 1024 * 1024;  // 4MB.
  constexpr const char first_topic_name[] = "/test_topic0";
  constexpr const char second_topic_name[] = "/test_topic1";
  constexpr const int expected_splits = 4;
  constexpr const char message_str[] = "Test";
  constexpr const int message_size = 1024 * 1024;  // 1MB
  const auto message = create_string_message(message_str, message_size);
  constexpr const int message_count = bagfile_split_size * expected_splits / message_size;
  constexpr const int message_batch_size = message_count / 2;

  rosbag2_test_common::PublicationManager pub_manager;
  pub_manager.setup_publisher(first_topic_name, message, message_batch_size);
  pub_manager.setup_publisher(second_topic_name, message, message_batch_size);

  std::stringstream command;
  command << get_base_record_command() <<
    " --max-bag-size " << bagfile_split_size <<
    " -a";
  auto process_handle = start_execution(command.str());
  auto cleanup_process_handle = rcpputils::make_scope_exit(
    [process_handle]() {
      stop_execution(process_handle);
    });

  ASSERT_TRUE(pub_manager.wait_for_matched("/test_topic0")) <<
    "Expected find rosbag subscription";
  ASSERT_TRUE(pub_manager.wait_for_matched("/test_topic1")) <<
    "Expected find rosbag subscription";

  wait_for_storage_file();

  pub_manager.run_publishers();

  stop_execution(process_handle);
  cleanup_process_handle.cancel();

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

TEST_P(RecordFixture, record_end_to_end_with_splitting_bagsize_split_is_at_least_specified_size) {
  constexpr const char topic_name[] = "/test_topic";
  constexpr const int bagfile_split_size = 4 * 1024 * 1024;  // 4MB.
  constexpr const int expected_splits = 3;
  constexpr const char message_str[] = "Test";
  constexpr const int message_size = 512 * 1024;  // 512KB
  const auto message = create_string_message(message_str, message_size);
  constexpr const int message_count = bagfile_split_size * expected_splits / message_size;

  rosbag2_test_common::PublicationManager pub_manager;
  pub_manager.setup_publisher(topic_name, message, message_count);

  std::stringstream command;
  command << get_base_record_command() <<
    " --max-bag-size " << bagfile_split_size <<
    " " << topic_name;
  auto process_handle = start_execution(command.str());
  auto cleanup_process_handle = rcpputils::make_scope_exit(
    [process_handle]() {
      stop_execution(process_handle);
    });

  ASSERT_TRUE(pub_manager.wait_for_matched(topic_name)) <<
    "Expected find rosbag subscription";

  wait_for_storage_file();

  stop_execution(process_handle);
  cleanup_process_handle.cancel();

  pub_manager.run_publishers();

  finalize_metadata_kludge(expected_splits);
  wait_for_metadata();
  rosbag2_storage::MetadataIo metadata_io;
  const auto metadata = metadata_io.read_metadata(root_bag_path_.string());
  const auto actual_splits = static_cast<int>(metadata.files.size());

  // TODO(zmichaels11): Support reliable sync-to-disk for more accurate splits.
  // The only guarantee with splits right now is that they will not occur until
  // a bagfile is at least the specified max_bagfile_size.
  EXPECT_GT(actual_splits, 0);

  // Don't include the last bagfile since it won't be full
  for (int i = 0; i < actual_splits - 1; ++i) {
    const auto bagfile_path = root_bag_path_ / rcpputils::fs::path{metadata.files[i].path};
    ASSERT_TRUE(bagfile_path.exists()) <<
      "Expected bag file: \"" << bagfile_path.string() << "\" to exist.";

    const auto actual_split_size = static_cast<int>(bagfile_path.file_size());
    // Actual size is guaranteed to be >= bagfile_split size
    EXPECT_LT(bagfile_split_size, actual_split_size);
  }
}

TEST_P(RecordFixture, record_end_to_end_with_splitting_max_size_not_reached) {
  constexpr const char topic_name[] = "/test_topic";
  constexpr const int bagfile_split_size = 4 * 1024 * 1024;  // 4MB.
  constexpr const int message_size = 512 * 1024;  // 512KB
  constexpr const char message_str[] = "Test";
  const auto message = create_string_message(message_str, message_size);
  // only fill the bagfile halfway
  constexpr const int message_count = bagfile_split_size / message_size / 2;

  rosbag2_test_common::PublicationManager pub_manager;
  pub_manager.setup_publisher(topic_name, message, message_count);

  std::stringstream command;
  command << get_base_record_command() <<
    " --max-bag-size " << bagfile_split_size <<
    " " << topic_name;
  auto process_handle = start_execution(command.str());
  auto cleanup_process_handle = rcpputils::make_scope_exit(
    [process_handle]() {
      stop_execution(process_handle);
    });

  ASSERT_TRUE(pub_manager.wait_for_matched(topic_name)) <<
    "Expected find rosbag subscription";

  wait_for_storage_file();

  pub_manager.run_publishers();

  stop_execution(process_handle);
  cleanup_process_handle.cancel();

  finalize_metadata_kludge();
  wait_for_metadata();
  rosbag2_storage::MetadataIo metadata_io;
  const auto metadata = metadata_io.read_metadata(root_bag_path_.string());

  // Check that there's only 1 bagfile and that it exists.
  ASSERT_EQ(1u, metadata.files.size());
  const auto bagfile_path = root_bag_path_ / rcpputils::fs::path{metadata.files[0].path};
  ASSERT_TRUE(bagfile_path.exists()) <<
    "Expected bag file: \"" << bagfile_path.string() << "\" to exist.";

  // Check that the next bagfile does not exist.
  const auto next_bag_file = get_bag_file_path(1);
  EXPECT_FALSE(next_bag_file.exists()) << "Expected next bag file: \"" <<
    next_bag_file.string() << "\" to not exist!";
}

TEST_P(RecordFixture, record_end_to_end_with_splitting_splits_bagfile) {
  constexpr const char topic_name[] = "/test_topic";
  constexpr const int bagfile_split_size = 4 * 1024 * 1024;  // 4MB.
  constexpr const int expected_splits = 4;
  constexpr const char message_str[] = "Test";
  constexpr const int message_size = 1024 * 1024;  // 1MB
  // string message from test_msgs
  const auto message = create_string_message(message_str, message_size);
  constexpr const int message_count = bagfile_split_size * expected_splits / message_size;

  rosbag2_test_common::PublicationManager pub_manager;
  pub_manager.setup_publisher(topic_name, message, message_count);

  std::stringstream command;
  command << get_base_record_command() <<
    " --max-bag-size " << bagfile_split_size <<
    " " << topic_name;
  auto process_handle = start_execution(command.str());
  auto cleanup_process_handle = rcpputils::make_scope_exit(
    [process_handle]() {
      stop_execution(process_handle);
    });

  ASSERT_TRUE(pub_manager.wait_for_matched(topic_name)) <<
    "Expected find rosbag subscription";

  wait_for_storage_file();

  pub_manager.run_publishers();

  stop_execution(process_handle);
  cleanup_process_handle.cancel();

  wait_for_metadata();
  finalize_metadata_kludge(expected_splits);
  rosbag2_storage::MetadataIo metadata_io;
  const auto metadata = metadata_io.read_metadata(root_bag_path_.string());

  ASSERT_GE(metadata.relative_file_paths.size(), 1u) << "Bagfile never split!";

  for (const auto & file : metadata.files) {
    auto path = root_bag_path_ / rcpputils::fs::path(file.path);
    EXPECT_TRUE(rcpputils::fs::exists(path));
  }
}

TEST_P(RecordFixture, record_end_to_end_with_duration_splitting_splits_bagfile) {
  constexpr const char topic_name[] = "/test_topic";
  constexpr const int bagfile_split_duration = 1000;   // 1 second
  constexpr const int expected_splits = 4;
  constexpr const char message_str[] = "Test";
  constexpr const int message_size = 1024 * 1024;  // 1MB
  constexpr const int message_time = 500;  // 500ms
  // string message from test_msgs
  const auto message = create_string_message(message_str, message_size);
  constexpr const int message_count = (bagfile_split_duration * expected_splits) / message_time;

  rosbag2_test_common::PublicationManager pub_manager;
  pub_manager.setup_publisher(topic_name, message, message_count);

  std::stringstream command;
  command << get_base_record_command() <<
    " -d " << bagfile_split_duration <<
    " " << topic_name;
  auto process_handle = start_execution(command.str());
  auto cleanup_process_handle = rcpputils::make_scope_exit(
    [process_handle]() {
      stop_execution(process_handle);
    });

  ASSERT_TRUE(pub_manager.wait_for_matched(topic_name)) <<
    "Expected find rosbag subscription";

  wait_for_storage_file();

  pub_manager.run_publishers();

  stop_execution(process_handle);
  cleanup_process_handle.cancel();

  finalize_metadata_kludge();
  wait_for_metadata();
  rosbag2_storage::MetadataIo metadata_io;
  const auto metadata = metadata_io.read_metadata(root_bag_path_.string());

  for (const auto & file : metadata.files) {
    auto path = root_bag_path_ / rcpputils::fs::path(file.path);
    EXPECT_TRUE(rcpputils::fs::exists(path));
  }
}

TEST_P(RecordFixture, record_end_to_end_test_with_zstd_file_compression_compresses_files) {
  constexpr const char topic_name[] = "/test_topic";
  constexpr const int bagfile_split_size = 4 * 1024 * 1024;  // 4MB.
  constexpr const int expected_splits = 4;
  constexpr const char message_str[] = "Test";
  constexpr const int message_size = 1024 * 1024;  // 1MB
  // string message from test_msgs
  const auto message = create_string_message(message_str, message_size);
  constexpr const int message_count = bagfile_split_size * expected_splits / message_size;

  rosbag2_test_common::PublicationManager pub_manager;
  pub_manager.setup_publisher(topic_name, message, message_count);

  std::stringstream command;
  command << get_base_record_command() <<
    " --max-bag-size " << bagfile_split_size <<
    " --compression-mode file" <<
    " --compression-format zstd"
    " " << topic_name;

  auto process_handle = start_execution(command.str());
  auto cleanup_process_handle = rcpputils::make_scope_exit(
    [process_handle]() {
      stop_execution(process_handle);
    });

  ASSERT_TRUE(pub_manager.wait_for_matched(topic_name)) <<
    "Expected find rosbag subscription";

  wait_for_storage_file();

  pub_manager.run_publishers();

  stop_execution(process_handle);
  cleanup_process_handle.cancel();

  finalize_metadata_kludge(expected_splits);
  wait_for_metadata();
  rosbag2_storage::MetadataIo metadata_io;
  const auto metadata = metadata_io.read_metadata(root_bag_path_.string());

  for (const auto & path : metadata.relative_file_paths) {
    const auto file_path = root_bag_path_ / rcpputils::fs::path{path};

    EXPECT_TRUE(file_path.exists()) << "File: \"" <<
      file_path.string() << "\" does not exist!";
    EXPECT_EQ(file_path.extension().string(), ".zstd") << "File :\"" <<
      file_path.string() << "\" does not have proper \".zstd\" extension!";
  }
}

TEST_P(RecordFixture, record_fails_gracefully_if_bag_already_exists) {
  auto bag_path = _SRC_RESOURCES_DIR_PATH;  // variable defined in CMakeLists.txt

  internal::CaptureStderr();
  auto exit_code = execute_and_wait_until_completion(
    "ros2 bag record --output empty_dir -a --storage " + GetParam(), bag_path);
  auto error_output = internal::GetCapturedStderr();

  EXPECT_THAT(exit_code, Eq(EXIT_FAILURE));
  EXPECT_THAT(error_output, HasSubstr("Output folder 'empty_dir' already exists"));
}

TEST_P(RecordFixture, record_fails_if_both_all_and_topic_list_is_specified) {
  internal::CaptureStderr();
  auto exit_code = execute_and_wait_until_completion(
    get_base_record_command() + " -a /some_topic", temporary_dir_path_);
  auto error_output = internal::GetCapturedStderr();

  EXPECT_THAT(exit_code, Eq(EXIT_FAILURE));
  EXPECT_FALSE(error_output.empty());
}

TEST_P(RecordFixture, record_fails_if_neither_all_nor_topic_list_are_specified) {
  internal::CaptureStderr();
  auto exit_code =
    execute_and_wait_until_completion(get_base_record_command(), temporary_dir_path_);
  auto output = internal::GetCapturedStderr();

  EXPECT_THAT(exit_code, Eq(EXIT_FAILURE));
  EXPECT_FALSE(output.empty());
}

TEST_P(RecordFixture, record_fails_gracefully_if_plugin_for_given_encoding_does_not_exist) {
  internal::CaptureStderr();
  auto exit_code = execute_and_wait_until_completion(
    get_base_record_command() + " -a -f some_rmw", temporary_dir_path_);
  auto error_output = internal::GetCapturedStderr();

  EXPECT_THAT(exit_code, Ne(EXIT_SUCCESS));
  EXPECT_THAT(
    error_output, HasSubstr("invalid choice: 'some_rmw'"));
}

TEST_P(RecordFixture, record_end_to_end_test_with_cache) {
  auto max_cache_size = 10;
  auto topic_name = "/rosbag2_cache_test_topic";

  auto message = get_messages_strings()[0];
  message->string_value = "test";
  size_t expected_test_messages = 33;

  rosbag2_test_common::PublicationManager pub_manager;
  pub_manager.setup_publisher(topic_name, message, expected_test_messages);

  auto process_handle = start_execution(
    get_base_record_command() +
    " " + topic_name + " " +
    "--max-cache-size " + std::to_string(max_cache_size));
  auto cleanup_process_handle = rcpputils::make_scope_exit(
    [process_handle]() {
      stop_execution(process_handle);
    });

  ASSERT_TRUE(pub_manager.wait_for_matched(topic_name)) <<
    "Expected find rosbag subscription";

  wait_for_storage_file();

  pub_manager.run_publishers();

  stop_execution(process_handle);
  cleanup_process_handle.cancel();

  finalize_metadata_kludge();
  wait_for_metadata();
  auto test_topic_messages =
    get_messages_for_topic<test_msgs::msg::Strings>(topic_name);
  EXPECT_THAT(test_topic_messages, SizeIs(Ge(expected_test_messages)));
}

TEST_P(RecordFixture, rosbag2_record_and_play_multiple_topics_with_filter) {
  constexpr const int bagfile_split_size = 4 * 1024 * 1024;  // 4MB.
  constexpr const char first_topic_name[] = "/test_topic0";
  constexpr const char second_topic_name[] = "/test_topic1";
  constexpr const int expected_splits = 4;
  constexpr const char message_str[] = "Test";
  constexpr const int message_size = 1024 * 1024;  // 1MB
  constexpr const int message_count = bagfile_split_size * expected_splits / message_size;
  const auto message = create_string_message(message_str, message_size);
  constexpr const int message_batch_size = message_count / 2;
  rosbag2_test_common::PublicationManager pub_manager;
  pub_manager.setup_publisher(first_topic_name, message, message_batch_size);
  pub_manager.setup_publisher(second_topic_name, message, message_batch_size);

  std::stringstream command_record;
  command_record << get_base_record_command() <<
    " --max-bag-size " << bagfile_split_size <<
    " -a";
  auto process_handle = start_execution(command_record.str());
  auto cleanup_process_handle = rcpputils::make_scope_exit(
    [process_handle]() {
      stop_execution(process_handle);
    });

  ASSERT_TRUE(pub_manager.wait_for_matched(first_topic_name)) <<
    "Expected find rosbag subscription";
  ASSERT_TRUE(pub_manager.wait_for_matched(second_topic_name)) <<
    "Expected find rosbag subscription";
  wait_for_storage_file();

  pub_manager.run_publishers();

  stop_execution(process_handle);
  cleanup_process_handle.cancel();

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

INSTANTIATE_TEST_SUITE_P(
  TestRecordEndToEnd,
  RecordFixture,
  ::testing::ValuesIn(rosbag2_test_common::kTestedStorageIDs)
);
