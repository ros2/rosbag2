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

#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "rosbag2/formatter.hpp"

using namespace ::testing;  // NOLINT
using namespace std::chrono_literals;  // NOLINT

class FormatterTestFixture : public Test
{
public:
  FormatterTestFixture()
  : formatter_(std::make_unique<rosbag2::Formatter>()) {}

  std::unique_ptr<rosbag2::Formatter> formatter_;
};

TEST_F(FormatterTestFixture, format_file_size_returns_correct_format) {
  size_t zero_bytes = 0;
  size_t thirty_bytes = 30;
  size_t two_kilobytes = 2048;
  size_t two_point_twelve_kilobytes = 3195;
  size_t one_and_a_half_megabytes = 1536 * 1024;
  size_t one_terabite = static_cast<size_t>(pow(1024, 4));
  size_t one_petabyte = static_cast<size_t>(pow(1024, 5));

  EXPECT_THAT(formatter_->format_file_size(zero_bytes), Eq("0 B"));
  EXPECT_THAT(formatter_->format_file_size(thirty_bytes), Eq("30.0 B"));
  EXPECT_THAT(formatter_->format_file_size(two_kilobytes), Eq("2.0 KB"));
  EXPECT_THAT(formatter_->format_file_size(two_point_twelve_kilobytes), Eq("3.1 KB"));
  EXPECT_THAT(formatter_->format_file_size(one_and_a_half_megabytes), Eq("1.5 MB"));
  EXPECT_THAT(formatter_->format_file_size(one_terabite), Eq("1.0 TB"));
  EXPECT_THAT(formatter_->format_file_size(one_petabyte), Eq("1024.0 TB"));
}

TEST_F(FormatterTestFixture, format_files_correctly_layouts_more_paths) {
  std::vector<std::string> paths = {"first/file/path", "second/file", "third/path/"};
  std::stringstream formatted_output;

  formatter_->format_file_paths(paths, formatted_output);
  EXPECT_THAT(formatted_output.str(), Eq(
      "first/file/path\n"
      "                  second/file\n"
      "                  third/path/\n"));
}

TEST_F(FormatterTestFixture, format_files_prints_newline_if_there_are_no_paths) {
  std::vector<std::string> paths = {};
  std::stringstream formatted_output;

  formatter_->format_file_paths(paths, formatted_output);
  EXPECT_THAT(formatted_output.str(), Eq("\n"));
}

TEST_F(FormatterTestFixture, format_topics_with_type_correctly_layouts_more_topics) {
  std::vector<rosbag2::TopicMetadata> topics;
  topics.push_back({{"topic1", "type1"}, 100});
  topics.push_back({{"topic2", "type2"}, 200});
  std::stringstream formatted_output;

  formatter_->format_topics_with_type(topics, formatted_output);
  EXPECT_THAT(formatted_output.str(), Eq("topic1; type1; 100 msgs\n"
    "                  topic2; type2; 200 msgs\n"));
}

TEST_F(FormatterTestFixture, format_topics_with_type_prints_newline_if_there_are_no_topics) {
  std::vector<rosbag2::TopicMetadata> topics = {};
  std::stringstream formatted_output;

  formatter_->format_topics_with_type(topics, formatted_output);
  EXPECT_THAT(formatted_output.str(), Eq("\n"));
}
