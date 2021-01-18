// Copyright (c) 2020 DCS Corporation
// All Rights Reserved
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    Caution-http://www.apache.org/licenses/LICENSE-2.0
//
// The software/firmware is provided to you on an As-Is basis.
//
// DISTRIBUTION A. Approved for public release; distribution unlimited.
// OPSEC #4584.
//
// Delivered to the U.S. Government with Unlimited Rights, as defined in DFARS
// Part 252.227-7013 or 7014 (Feb 2014).
//
// This notice must appear in all copies of this file and its derivatives.

#include <gmock/gmock.h>

#include <errno.h>

#include <cstdlib>
#include <future>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

// rclcpp must be included before process_execution_helpers.hpp
#include "rclcpp/rclcpp.hpp"

#include "rosbag2_storage_default_plugins/sqlite/sqlite_storage.hpp"

#include "rosbag2_test_common/subscription_manager.hpp"
#include "rosbag2_test_common/process_execution_helpers.hpp"

// #include "test_msgs/msg/arrays.hpp"
// #include "test_msgs/msg/basic_types.hpp"
// #include "test_msgs/message_fixtures.hpp"

#include "yaml-cpp/yaml.h"

using namespace ::testing;  // NOLINT
using namespace rosbag2_test_common;  // NOLINT

class ReindexEndToEndTestFixture : public Test
{
public:
  ReindexEndToEndTestFixture()
  {
    database_path_ = _SRC_RESOURCES_DIR_PATH;  // variable defined in CMakeLists.txt
  }

  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  std::string database_path_;
  std::unique_ptr<SubscriptionManager> sub_;

  // Declare keys to avoid potential typos
  const char * const ROOT_KEY = "rosbag2_bagfile_information";
  const char * const VERSION_KEY = "version";
  const char * const SI_KEY = "storage_identifier";
  const char * const RFP_KEY = "relative_file_paths";
  const char * const DURATION_KEY = "duration";
  const char * const NS_KEY = "nanoseconds";
  const char * const ST_KEY = "starting_time";
  const char * const NSSE_KEY = "nanoseconds_since_epoch";
  const char * const COUNT_KEY = "message_count";

  // Topics With Message Count section
  const char * const TWMC_KEY = "topics_with_message_count";
  const char * const TOPIC_KEY = "topic_metadata";
  const char * const NAME_KEY = "name";
  const char * const TYPE_KEY = "type";
  const char * const SF_KEY = "serialization_format";
  const char * const QOS_KEY = "offered_qos_profiles";

  // Compression section
  const char * const CFORMAT_KEY = "compression_format";
  const char * const MODE_KEY = "compression_mode";
};

/** FOR REFERENCE, A MAP OF A METADATA.YAML FILE
 * <FILE BEGIN>
 * rosbag2_bagfile_information:
 *   version: <number>
 *   storage_identifier: <string>
 *   relative_file_paths: <sequence>
 *     - <string>
 *   duration:
 *     nanoseconds: <number>
 *   starting_time:
 *     nanoseconds_since_epoch: <number>
 *   message_count: <number>
 *   topics_with_message_count: <sequence>
 *     - topic_metadata:
 *         name: <string>
 *         type: <string>
 *         serialization_format: <string>
 *         offered_qos_profiles: <string>
 *       message_count: <number>
 *   compression_format: <string>
 *   compression_mode: <string>
 * <EOF>
 * */
#ifndef _WIN32
TEST_F(ReindexEndToEndTestFixture, reindex_end_to_end_test) {
  // We will be creating a new metadata file (hopefully), so preserve the old one
  std::string old_name = database_path_ + "/cdr_test/metadata.yaml";
  std::string new_name = database_path_ + "/cdr_test/metadata_old.yaml";

  bool rename_success = std::rename(old_name.c_str(), new_name.c_str()) == 0;
  EXPECT_EQ(rename_success, true);
  if (!rename_success) {
    std::perror("Error occurred during rename: ");
    return;
  }

  // Try to create the metadata file
  auto exit_code = execute_and_wait_until_completion("ros2 bag reindex cdr_test", database_path_);

  // First check to see if we could create a file at all
  EXPECT_EQ(exit_code, EXIT_SUCCESS);
  if (exit_code != EXIT_SUCCESS) {
    // Delete any created metadata file (just in case)
    std::remove(old_name.c_str());
    std::rename(new_name.c_str(), old_name.c_str());
    return;
  }

  // If we're at this point, that means a metadata file was cretaed.
  // Read both the old and new metadata files in.
  YAML::Node original_metadata = YAML::LoadFile(new_name.c_str())[ROOT_KEY];
  YAML::Node new_metadata = YAML::LoadFile(old_name.c_str());

  // Easy test first - check for the root node
  /**
   * <FILE BEGIN>
   * rosbag2_bagfile_information:   <- WE ARE HERE
   *  version: <number>
   *  storage_identifier: <string>
   * ...
   * */
  EXPECT_TRUE(new_metadata[ROOT_KEY]);
  if (!new_metadata[ROOT_KEY]) {
    // Something went wrong. Abort.
    std::remove(old_name.c_str());
    std::rename(new_name.c_str(), old_name.c_str());
    return;
  }

  // All further tests use the root node
  YAML::Node root_node = new_metadata[ROOT_KEY];

  // Check the version
  /**
   * <FILE BEGIN>>
   * rosbag2_bagfile_information:
   *  version: <number>   <- WE ARE HERE
   *  storage_identifier: <string>
   *  relative_file_paths: <sequence>
   * ...
   * */
  std::cout << "Checking version" << std::endl;
  EXPECT_TRUE(root_node[VERSION_KEY]);
  if (root_node[VERSION_KEY]) {
    int new_version = root_node[VERSION_KEY].as<int>();
    int original_version = original_metadata[VERSION_KEY].as<int>();
    EXPECT_EQ(new_version, original_version);
  }

  // Test storage identifier
  /**
   * ...
   * rosbag2_bagfile_information:
   *  version: <number>
   *  storage_identifier: <string>    <- WE ARE HERE
   *  relative_file_paths: <sequence>
   *    - <string>
   * ...
   * */
  std::cout << "Checking storage identifier" << std::endl;
  EXPECT_TRUE(root_node[SI_KEY]);
  if (root_node[SI_KEY]) {
    std::string new_si = root_node[SI_KEY].as<std::string>();
    std::string old_si = original_metadata[SI_KEY].as<std::string>();
    EXPECT_EQ(new_si, old_si);
  }

  // Test relative file paths
  /**
   * ...
   *  version: <number>
   *  storage_identifier: <string>
   *  relative_file_paths: <sequence>   <- WE ARE HERE
   *    - <string>                      <- AND HERE TOO
   *  duration:
   * ...
   * */
  std::cout << "Checking relative file paths" << std::endl;
  EXPECT_TRUE(root_node[RFP_KEY]);
  if (root_node[RFP_KEY]) {
    std::vector<std::string> new_filepaths =
      root_node[RFP_KEY].as<std::vector<std::string>>();
    std::vector<std::string> original_filepaths =
      original_metadata[RFP_KEY].as<std::vector<std::string>>();
    EXPECT_EQ(new_filepaths, original_filepaths);
  }

  // Test duration
  /**
   * ...
   *  relative_file_paths: <sequence>
   *    - <string>
   *  duration:                 <- WE ARE HERE
   *    nanoseconds: <number>   <- AND HERE TOO
   *  starting_time:
   *    nanoseconds_since_epoch: <number>
   * ...
   * */
  std::cout << "Checking duration" << std::endl;
  EXPECT_TRUE(root_node[DURATION_KEY]);
  if (root_node[DURATION_KEY]) {
    YAML::Node duration_node = root_node[DURATION_KEY];
    YAML::Node original_duration_node = original_metadata[DURATION_KEY];
    EXPECT_TRUE(duration_node[NS_KEY]);
    if (duration_node[NS_KEY]) {
      int64_t new_duration = duration_node[NS_KEY].as<int64_t>();
      int64_t original_duration = original_duration_node[NS_KEY].as<int64_t>();
      EXPECT_EQ(new_duration, original_duration);
    }
  }

  // Test starting time
  /**
   * ...
   *  duration:
   *    nanoseconds: <number>
   *  starting_time:                          <- WE ARE HERE
   *    nanoseconds_since_epoch: <number>     <- AND HERE TOO
   *  message_count: <number>
   *  topics_with_message_count:  <sequence>
   * ...
   * */
  std::cout << "Checking starting_time" << std::endl;
  EXPECT_TRUE(root_node[ST_KEY]);
  if (root_node[ST_KEY]) {
    YAML::Node starting_time = root_node[ST_KEY];
    YAML::Node original_starting_time_node = original_metadata[ST_KEY];
    EXPECT_TRUE(starting_time[NSSE_KEY]);
    if (starting_time[NSSE_KEY]) {
      int64_t new_start = starting_time[NSSE_KEY].as<int64_t>();
      int64_t original_start = original_starting_time_node[NSSE_KEY].as<int64_t>();
      EXPECT_EQ(new_start, original_start);
    }
  }

  // Test message count
  /**
   * ...
   *  starting_time:
   *    nanoseconds_since_epoch: <number>
   *  message_count: <number>               <- WE ARE HERE
   *  topics_with_message_count: <sequence>
   *...
   * */
  std::cout << "Checking message count" << std::endl;
  EXPECT_TRUE(root_node[COUNT_KEY]);
  if (root_node[COUNT_KEY]) {
    int new_count = root_node[COUNT_KEY].as<int>();
    int original_count = original_metadata[COUNT_KEY].as<int>();
    EXPECT_EQ(new_count, original_count);
  }

  // Test topics
  /**
   * ...
   *  message_count: <number>
   *  topics_with_message_count: <sequence>   <- WE ARE HERE
   *    - topic_metadata:                     <- AND HERE TOO
   *        name: <string>                    <- AND HERE AS WELL
   *        type: <string>                    <- AND ALSO HERE
   *        serialization_format: <string>    <- AND EVEN HERE
   *        offered_qos_profiles: <string>    <- AND HERE
   *      message_count:  <number>            <- AND HERE
   *  compression_format: <string>
   * ...
   * */
  std::cout << "Checking topics" << std::endl;
  EXPECT_TRUE(root_node[TWMC_KEY]);
  if (root_node[TWMC_KEY]) {
    YAML::Node new_twmc_node = root_node[TWMC_KEY];
    YAML::Node old_twmc_node = original_metadata[TWMC_KEY];

    // The ordering of topics is not guaranteed to be preserved during
    // the reindexing process. So we have to iterate over each topic
    // in the new metadata, and see if it exists in the original
    for (YAML::const_iterator new_metadata_iter = new_twmc_node.begin();
      new_metadata_iter != new_twmc_node.end();
      ++new_metadata_iter)
    {
      bool found_match = false;
      YAML::Node new_topic = *new_metadata_iter;
      YAML::Node new_topic_metadata = new_topic[TOPIC_KEY];
      std::string new_topic_name = new_topic_metadata[NAME_KEY].as<std::string>();

      for (YAML::const_iterator orig_metadata_iter = old_twmc_node.begin();
        orig_metadata_iter != old_twmc_node.end();
        ++orig_metadata_iter)
      {
        YAML::Node old_topic = *orig_metadata_iter;
        YAML::Node old_topic_metadata = old_topic[TOPIC_KEY];
        std::string old_topic_name = old_topic_metadata[NAME_KEY].as<std::string>();

        if (new_topic_name == old_topic_name) {
          // A match! The testing can now continue
          found_match = true;
          // Check that the other topic metadata matches...
          // Check type:
          std::string new_topic_type = new_topic_metadata[TYPE_KEY].as<std::string>();
          std::string old_topic_type = old_topic_metadata[TYPE_KEY].as<std::string>();
          EXPECT_EQ(new_topic_type, old_topic_type);

          // Check serialization format
          std::string new_s_fmt = new_topic_metadata[SF_KEY].as<std::string>();
          std::string old_s_fmt = old_topic_metadata[SF_KEY].as<std::string>();
          EXPECT_EQ(new_s_fmt, old_s_fmt);

          // Check qos profiles
          std::string new_qos = new_topic_metadata[QOS_KEY].as<std::string>();
          std::string old_qos = old_topic_metadata[QOS_KEY].as<std::string>();
          EXPECT_EQ(new_qos, old_qos);

          // Check message count
          int64_t new_count = new_topic[COUNT_KEY].as<int64_t>();
          int64_t old_count = old_topic[COUNT_KEY].as<int64_t>();
          EXPECT_EQ(new_count, old_count);
          break;
        }
      }

      EXPECT_TRUE(found_match);   // We expected to have a name match
    }
  }

  // Test compression format
  /**
   * ...
   *        serialization_format: <string>
   *        offered_qos_profiles: <string>
   *      message_count:  <number>
   *  compression_format: <string>  <- WE ARE HERE
   *  compression_mode: <string>
   * <EOF>
   * */
  std::cout << "Checking compression format" << std::endl;
  EXPECT_TRUE(root_node[CFORMAT_KEY]);
  if (root_node[CFORMAT_KEY]) {
    std::string new_format = root_node[CFORMAT_KEY].as<std::string>();
    std::string old_format = original_metadata[CFORMAT_KEY].as<std::string>();
    EXPECT_EQ(new_format, old_format);
  }

  // Test compression mode
  /**
   * ...
   *  compression_format: <string>
   *  compression_mode: <string>    <- WE ARE HERE
   * <EOF>
   * */
  std::cout << "Checking compression mode" << std::endl;
  EXPECT_TRUE(root_node[MODE_KEY]);
  if (root_node[MODE_KEY]) {
    std::string new_mode = root_node[MODE_KEY].as<std::string>();
    std::string old_mode = original_metadata[MODE_KEY].as<std::string>();
    EXPECT_EQ(new_mode, old_mode);
  }

  // Cleanup
  std::remove(old_name.c_str());
  std::rename(new_name.c_str(), old_name.c_str());
}
#endif

TEST_F(ReindexEndToEndTestFixture, reindex_fails_gracefully_if_bag_does_not_exist) {
  internal::CaptureStderr();
  auto exit_code =
    execute_and_wait_until_completion("ros2 bag reindex does_not_exist", database_path_);
  auto error_output = internal::GetCapturedStderr();

  // Exit code could be EXIT_FAILURE (1) or 2 (no such file or directory)
  EXPECT_NE(exit_code, EXIT_SUCCESS);
  EXPECT_THAT(error_output, HasSubstr("'does_not_exist' does not exist"));
}
