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
#include <sstream>
#include <string>
#include <vector>

#include "../../src/rosbag2_transport/formatter.hpp"

using namespace ::testing;  // NOLINT
using namespace std::chrono_literals;  // NOLINT

class FormatterTestFixture : public Test
{
public:
  FormatterTestFixture()
  : formatter_(std::make_unique<rosbag2_transport::Formatter>()), indentation_spaces_(19) {}

  std::unique_ptr<rosbag2_transport::Formatter> formatter_;
  int indentation_spaces_;
};

TEST_F(FormatterTestFixture, format_file_size_returns_correct_format) {
  size_t zero_bytes = 0;
  size_t thirty_bytes = 30;
  size_t two_kibibytes = 2048;
  size_t two_point_twelve_kibibytes = 3195;
  size_t one_and_a_half_mebibytes = 1536 * 1024;
  size_t one_tebibite = static_cast<size_t>(pow(1024, 4));
  size_t one_pebibyte = static_cast<size_t>(pow(1024, 5));

  EXPECT_THAT(formatter_->format_file_size(zero_bytes), Eq("0 B"));
  EXPECT_THAT(formatter_->format_file_size(thirty_bytes), Eq("30 B"));
  EXPECT_THAT(formatter_->format_file_size(two_kibibytes), Eq("2.0 KiB"));
  EXPECT_THAT(formatter_->format_file_size(two_point_twelve_kibibytes), Eq("3.1 KiB"));
  EXPECT_THAT(formatter_->format_file_size(one_and_a_half_mebibytes), Eq("1.5 MiB"));
  EXPECT_THAT(formatter_->format_file_size(one_tebibite), Eq("1.0 TiB"));
  EXPECT_THAT(formatter_->format_file_size(one_pebibyte), Eq("1024.0 TiB"));
}

TEST_F(FormatterTestFixture, format_files_correctly_layouts_more_paths) {
  std::vector<std::string> paths = {"first/file/path", "second/file", "third/path/"};
  std::stringstream formatted_output;

  formatter_->format_file_paths(paths, formatted_output, indentation_spaces_);

  auto expected = std::string("first/file/path\n") +
    std::string(indentation_spaces_, ' ') + "second/file\n" +
    std::string(indentation_spaces_, ' ') + "third/path/\n";
  EXPECT_EQ(expected, formatted_output.str());
}

TEST_F(FormatterTestFixture, format_files_prints_newline_if_there_are_no_paths) {
  std::vector<std::string> paths = {};
  std::stringstream formatted_output;

  formatter_->format_file_paths(paths, formatted_output, 0);
  EXPECT_THAT(formatted_output.str(), Eq("\n"));
}

TEST_F(FormatterTestFixture, format_topics_with_type_correctly_layouts_more_topics) {
  std::vector<rosbag2::TopicInformation> topics;
  topics.push_back({{"topic1", "type1", "rmw1"}, 100});
  topics.push_back({{"topic2", "type2", "rmw2"}, 200});
  std::stringstream formatted_output;

  formatter_->format_topics_with_type(topics, formatted_output, indentation_spaces_);
  auto expected =
    std::string("Topic: topic1 | Type: type1 | Count: 100 | Serialization Format: rmw1\n") +
    std::string(indentation_spaces_, ' ') +
    std::string("Topic: topic2 | Type: type2 | Count: 200 | Serialization Format: rmw2\n");
  EXPECT_EQ(expected, formatted_output.str());
}

TEST_F(FormatterTestFixture, format_topics_with_type_prints_newline_if_there_are_no_topics) {
  std::vector<rosbag2::TopicInformation> topics = {};
  std::stringstream formatted_output;

  formatter_->format_topics_with_type(topics, formatted_output, 0);
  EXPECT_THAT(formatted_output.str(), Eq("\n"));
}
