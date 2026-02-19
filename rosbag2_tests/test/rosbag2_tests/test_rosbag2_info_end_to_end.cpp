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

#include <cstdlib>
#include <filesystem>
#include <regex>
#include <string>
#include <thread>

#include "rosbag2_test_common/process_execution_helpers.hpp"
#include "rosbag2_test_common/tested_storage_ids.hpp"

using namespace ::testing;  // NOLINT

namespace fs = std::filesystem;

class InfoEndToEndTestFixture : public Test, public WithParamInterface<std::string>
{
public:
  InfoEndToEndTestFixture()
  {
    // _SRC_RESOURCES_DIR_PATH defined in CMakeLists.txt
    bags_path_ = fs::absolute(fs::path(_SRC_RESOURCES_DIR_PATH) / GetParam()).generic_string();
  }

  std::string bags_path_;
};

TEST_P(InfoEndToEndTestFixture, info_end_to_end_test) {
  internal::CaptureStdout();
  auto exit_code = execute_and_wait_until_completion("ros2 bag info cdr_test", bags_path_);
  std::string output = internal::GetCapturedStdout();
  auto expected_storage = GetParam();
  auto expected_file = rosbag2_test_common::bag_filename_for_storage_id("cdr_test_0", GetParam());
  std::string expected_ros_distro = "unknown";

  EXPECT_THAT(exit_code, Eq(EXIT_SUCCESS));
  // The bag size depends on the os and is not asserted, the time is asserted time zone independent
  EXPECT_THAT(
    output, ContainsRegex(
      "\nFiles:             " + expected_file +
      "\nBag size:          .*B"
      "\nStorage id:        " + expected_storage +
      "\nROS Distro:        " + expected_ros_distro +
      "\nDuration:          0.151137181s"
      "\nStart:             Apr  .+ 2020 .*:.*:36.763032325 \\(1586406456.763032325\\)"
      "\nEnd:               Apr  .+ 2020 .*:.*:36.914169506 \\(1586406456.914169506\\)"
      "\nMessages:          7"
      "\nTopic information: "));
  EXPECT_THAT(
    output, HasSubstr(
      "Topic: /test_topic | Type: test_msgs/msg/BasicTypes | Count: 3 | "
      "Serialization Format: cdr\n"));
  EXPECT_THAT(
    output, HasSubstr(
      "Topic: /array_topic | Type: test_msgs/msg/Arrays | Count: 4 | "
      "Serialization Format: cdr"));
}

TEST_P(InfoEndToEndTestFixture, info_with_verbose_option_and_topic_name_option) {
  internal::CaptureStdout();
  auto exit_code = execute_and_wait_until_completion(
    "ros2 bag info bag_with_topics_and_service_events_and_action --verbose --topic-name",
    bags_path_);
  std::string output = internal::GetCapturedStdout();
  auto expected_storage = GetParam();
  auto expected_file = rosbag2_test_common::bag_filename_for_storage_id(
    "bag_with_topics_and_service_events_and_action", GetParam());
  std::string expected_ros_distro = "rolling";

  EXPECT_THAT(exit_code, Eq(EXIT_SUCCESS));

  EXPECT_THAT(
    output, HasSubstr(
      "Warning! You have set both the '-t' and '-v' parameters. The '-t' parameter "
      "will be ignored.\n"));

  // The bag size depends on the os and is not asserted, the time is asserted time zone independent
  EXPECT_THAT(
    output, ContainsRegex(
      "\nFiles:             " + expected_file +
      "\nBag size:          .* KiB"
      "\nStorage id:        " + expected_storage +
      "\nROS Distro:        " + expected_ros_distro +
      "\nDuration:          .*s"
      "\nStart:             Mar 14 2025 .*:.*:.*\\..* \\(.*\\..*\\)"
      "\nEnd:               Mar 14 2025 .*:.*:.*\\..* \\(.*\\..*\\)"
      "\nMessages:          .*"
      "\nTopic information: "));

  EXPECT_THAT(output, HasSubstr("Services:          2\n"));
}

TEST_P(InfoEndToEndTestFixture, info_with_verbose_option_end_to_end_test) {
  internal::CaptureStdout();
  auto exit_code = execute_and_wait_until_completion(
    "ros2 bag info bag_with_topics_and_service_events_and_action --verbose",
    bags_path_);
  std::string output = internal::GetCapturedStdout();
  auto expected_storage = GetParam();
  auto expected_file = rosbag2_test_common::bag_filename_for_storage_id(
    "bag_with_topics_and_service_events_and_action", GetParam());
  std::string expected_ros_distro = "rolling";

  EXPECT_THAT(exit_code, Eq(EXIT_SUCCESS));
  // The bag size depends on the os and is not asserted, the time is asserted time zone independent
  EXPECT_THAT(
    output, ContainsRegex(
      "\nFiles:             " + expected_file +
      "\nBag size:          .* KiB"
      "\nStorage id:        " + expected_storage +
      "\nROS Distro:        " + expected_ros_distro +
      "\nDuration:          .*s"
      "\nStart:             Mar 14 2025 .*:.*:.*\\..* \\(.*\\..*\\)"
      "\nEnd:               Mar 14 2025 .*:.*:.*\\..* \\(.*\\..*\\)"
      "\nMessages:          .*"
      "\nTopic information: "));

  // On the Windows platform, ContainsRegex cannot correctly handle the "|" character,
  // so replace it with std::regex to check.
  std::regex topic_1_info_pattern(
    R"(Topic: /test_topic1 | Type: std_msgs/msg/String | Count: \d+"
    " | Size Contribution: \d+ B | Serialization Format: cdr\n)");
  EXPECT_TRUE(std::regex_search(output, topic_1_info_pattern));

  std::regex topic_2_info_pattern(
    R"(Topic: /test_topic2 | Type: std_msgs/msg/String | Count: \d+"
    " | Size Contribution: \\+d B \\| Serialization Format: cdr\n)");
  EXPECT_TRUE(std::regex_search(output, topic_2_info_pattern));

  EXPECT_THAT(output, HasSubstr("Services:          2\n"));

  std::regex service_1_info_pattern(
    R"(Service: /test_service1 | Type: example_interfaces/srv/AddTwoInts | Request Count: \d+ "
    "| Response Count: \d+ | Size Contribution: \d+ B | Serialization Format: cdr\n)");
  EXPECT_TRUE(std::regex_search(output, service_1_info_pattern));

  std::regex service_2_info_pattern(
    R"(Service: /test_service2 | Type: example_interfaces/srv/AddTwoInts | Request Count: \d+ "
    "| Response Count: \d+ | Size Contribution: \d+ B | Serialization Format: cdr\n)");
  EXPECT_TRUE(std::regex_search(output, service_2_info_pattern));

  EXPECT_THAT(output, HasSubstr("Actions:           2\nAction information:"));

  std::regex action_1_info_pattern(
    R"(Action: /test_action1 | Type: example_interfaces/action/Fibonacci | Topics: 2 | Service: 3"
    " | Size Contribution: \d+ B | Serialization Format: cdr\n"
    "    Topic: feedback | Count: \d+\n"
    "    Topic: status | Count: \d+\n"
    "    Service: send_goal | Request Count: \d+ | Response Count: \d+\n"
    "    Service: cancel_goal | Request Count: \d+ | Response Count: \d+\n"
    "    Service: get_result | Request Count: \d+ | Response Count: \d+)");
  EXPECT_TRUE(std::regex_search(output, action_1_info_pattern));

  std::regex action_2_info_pattern(
    R"(Action: /test_action2 | Type: example_interfaces/action/Fibonacci | Topics: 2 | Service: 3"
    " | Size Contribution: \d+ B | Serialization Format: cdr\n"
    "    Topic: feedback | Count: \d+\n"
    "    Topic: status | Count: \d+\n"
    "    Service: send_goal | Request Count: \d+ | Response Count: \d+\n"
    "    Service: cancel_goal | Request Count: \d+ | Response Count: \d+\n"
    "    Service: get_result | Request Count: \d+ | Response Count: \d+)");
  EXPECT_TRUE(std::regex_search(output, action_2_info_pattern));
}

TEST_P(InfoEndToEndTestFixture, info_basic_types_and_arrays_with_verbose_option_end_to_end_test) {
  internal::CaptureStdout();
  auto exit_code = execute_and_wait_until_completion(
    "ros2 bag info cdr_test --verbose", bags_path_);
  std::string output = internal::GetCapturedStdout();
  auto expected_storage = GetParam();
  auto expected_file = rosbag2_test_common::bag_filename_for_storage_id("cdr_test_0", GetParam());
  std::string expected_ros_distro = "unknown";

  EXPECT_THAT(exit_code, Eq(EXIT_SUCCESS));
  // The bag size depends on the os and is not asserted, the time is asserted time zone independent
  EXPECT_THAT(
    output, ContainsRegex(
      "\nFiles:             " + expected_file +
      "\nBag size:          .*B"
      "\nStorage id:        " + expected_storage +
      "\nROS Distro:        " + expected_ros_distro +
      "\nDuration:          0.151137181s"
      "\nStart:             Apr  .+ 2020 .*:.*:36.763032325 \\(1586406456.763032325\\)"
      "\nEnd:               Apr  .+ 2020 .*:.*:36.914169506 \\(1586406456.914169506\\)"
      "\nMessages:          7"
      "\nTopic information: "));
  EXPECT_THAT(
    output, HasSubstr(
      "Topic: /test_topic | Type: test_msgs/msg/BasicTypes | Count: 3 | "
      "Size Contribution: 156 B | Serialization Format: cdr\n"));
  EXPECT_THAT(
    output, HasSubstr(
      "Topic: /array_topic | Type: test_msgs/msg/Arrays | Count: 4 | "
      "Size Contribution: 2.7 KiB | Serialization Format: cdr"));
}

TEST_P(InfoEndToEndTestFixture, info_output_default_sorted_by_name_test) {
  internal::CaptureStdout();
  auto exit_code = execute_and_wait_until_completion("ros2 bag info cdr_test", bags_path_);
  std::string output = internal::GetCapturedStdout();

  EXPECT_THAT(exit_code, Eq(EXIT_SUCCESS));
  EXPECT_THAT(
    output, HasSubstr(
      "Topic: /array_topic | Type: test_msgs/msg/Arrays | Count: 4 | "
      "Serialization Format: cdr\n"
      "                   Topic: /test_topic | Type: test_msgs/msg/BasicTypes | Count: 3 | "
      "Serialization Format: cdr"));
}

TEST_P(InfoEndToEndTestFixture, info_output_sorted_by_name_test) {
  internal::CaptureStdout();
  auto exit_code = execute_and_wait_until_completion(
    "ros2 bag info cdr_test --sort name", bags_path_);
  std::string output = internal::GetCapturedStdout();

  EXPECT_THAT(exit_code, Eq(EXIT_SUCCESS));
  EXPECT_THAT(
    output, HasSubstr(
      "Topic: /array_topic | Type: test_msgs/msg/Arrays | Count: 4 | "
      "Serialization Format: cdr\n"
      "                   Topic: /test_topic | Type: test_msgs/msg/BasicTypes | Count: 3 | "
      "Serialization Format: cdr"));
}

TEST_P(InfoEndToEndTestFixture, info_output_sorted_by_type_test) {
  internal::CaptureStdout();
  auto exit_code = execute_and_wait_until_completion(
    "ros2 bag info cdr_test --sort type", bags_path_);
  std::string output = internal::GetCapturedStdout();

  EXPECT_THAT(exit_code, Eq(EXIT_SUCCESS));
  EXPECT_THAT(
    output, HasSubstr(
      "Topic: /array_topic | Type: test_msgs/msg/Arrays | Count: 4 | "
      "Serialization Format: cdr\n"
      "                   Topic: /test_topic | Type: test_msgs/msg/BasicTypes | Count: 3 | "
      "Serialization Format: cdr"));
}

TEST_P(InfoEndToEndTestFixture, info_output_sorted_by_count_test) {
  internal::CaptureStdout();
  auto exit_code = execute_and_wait_until_completion(
    "ros2 bag info cdr_test --sort count", bags_path_);
  std::string output = internal::GetCapturedStdout();

  EXPECT_THAT(exit_code, Eq(EXIT_SUCCESS));
  EXPECT_THAT(
    output, HasSubstr(
      "Topic: /test_topic | Type: test_msgs/msg/BasicTypes | Count: 3 | "
      "Serialization Format: cdr\n"
      "                   Topic: /array_topic | Type: test_msgs/msg/Arrays | Count: 4 | "
      "Serialization Format: cdr"));
}

TEST_P(InfoEndToEndTestFixture, info_output_topics_only_sorted_by_count_test) {
  internal::CaptureStdout();
  auto exit_code = execute_and_wait_until_completion(
    "ros2 bag info -t cdr_test --sort count", bags_path_);
  std::string output = internal::GetCapturedStdout();

  EXPECT_THAT(exit_code, Eq(EXIT_SUCCESS));
  EXPECT_THAT(
    output, HasSubstr(
      "/test_topic\n"
      "/array_topic"));
}

TEST_P(InfoEndToEndTestFixture, info_output_topics_only_default_sorted_by_name_test) {
  internal::CaptureStdout();
  auto exit_code = execute_and_wait_until_completion(
    "ros2 bag info -t cdr_test", bags_path_);
  std::string output = internal::GetCapturedStdout();

  EXPECT_THAT(exit_code, Eq(EXIT_SUCCESS));
  EXPECT_THAT(
    output, HasSubstr(
      "/array_topic\n"
      "/test_topic"));
}

TEST_P(InfoEndToEndTestFixture, info_fails_gracefully_sort_method_does_not_exist_test) {
  internal::CaptureStderr();
  auto exit_code = execute_and_wait_until_completion(
    "ros2 bag info cdr_test --sort non_existent", bags_path_);
  auto error_output = internal::GetCapturedStderr();

  EXPECT_THAT(exit_code, Ne(EXIT_SUCCESS));
  EXPECT_THAT(
    error_output, HasSubstr("--sort: invalid choice: 'non_existent'"));
}

// TODO(Martin-Idel-SI): Revisit exit code non-zero here, gracefully should be exit code zero
TEST_P(InfoEndToEndTestFixture, info_fails_gracefully_if_bag_does_not_exist) {
  internal::CaptureStderr();
  auto exit_code =
    execute_and_wait_until_completion("ros2 bag info does_not_exist", bags_path_);
  auto error_output = internal::GetCapturedStderr();

  EXPECT_THAT(exit_code, Ne(EXIT_SUCCESS));
  EXPECT_THAT(error_output, HasSubstr("'does_not_exist' does not exist"));
}

TEST_P(InfoEndToEndTestFixture, info_fails_gracefully_if_metadata_yaml_file_does_not_exist) {
  internal::CaptureStderr();
  auto exit_code =
    execute_and_wait_until_completion("ros2 bag info " + bags_path_, bags_path_);
  auto error_output = internal::GetCapturedStderr();

  EXPECT_THAT(exit_code, Eq(EXIT_FAILURE));
  EXPECT_THAT(error_output, HasSubstr("Could not find metadata"));
}

INSTANTIATE_TEST_SUITE_P(
  TestInfoEndToEnd,
  InfoEndToEndTestFixture,
  ::testing::ValuesIn(rosbag2_test_common::kTestedStorageIDs)
);
