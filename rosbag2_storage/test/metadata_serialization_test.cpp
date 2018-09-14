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

#include <chrono>

#include "rosbag2_storage/bag_metadata.hpp"
#include "rosbag2_storage/metadata_io.hpp"

using namespace ::testing;  // NOLINT

class MetadataFixture : public Test
{
public:
  MetadataFixture()
  : yaml_file_name_(std::string(UnitTest::GetInstance()->current_test_info()->name()) + ".yaml")
  {
    std::string system_separator = "/";
#ifdef _WIN32
    system_separator = "\\";
#endif
    yaml_file_name_ = temporary_dir_path_ + system_separator + yaml_file_name_;
    std::cout << "Yaml name: " << yaml_file_name_ << std::endl;
  }

  ~MetadataFixture() override
  {
#ifdef _WIN32
    DeleteFileA(yaml_file_name_.c_str());
#else
    // TODO(botteroa-si): once filesystem::remove_all() can be used, this line can be removed and
    // the ful directory can be deleted in remove_temporary_dir()
    remove(yaml_file_name_.c_str());
#endif
  }

  static void SetUpTestCase()
  {
    char template_char[] = "tmp_test_dir.XXXXXX";
#ifdef _WIN32
    char temp_path[255];
    GetTempPathA(255, temp_path);
    _mktemp_s(template_char, strnlen(template_char, 20) + 1);
    temporary_dir_path_ = std::string(temp_path) + std::string(template_char);
    _mkdir(temporary_dir_path_.c_str());
#else
    char * dir_name = mkdtemp(template_char);
    temporary_dir_path_ = dir_name;
#endif
  }

  static void TearDownTestCase()
  {
    remove_temporary_dir();
  }

  static void remove_temporary_dir()
  {
#ifdef _WIN32
    // TODO(botteroa-si): find a way to delete a not empty directory in Windows, so that we don't
    // need the Windows line in the fixture destructor anymore.
    RemoveDirectoryA(temporary_dir_path_.c_str());
#else
    remove(temporary_dir_path_.c_str());
#endif
  }


  std::string yaml_file_name_;
  static std::string temporary_dir_path_;
};

std::string MetadataFixture::temporary_dir_path_ = "";  // NOLINT

TEST_F(MetadataFixture, test_writing_and_reading_yaml)
{
  rosbag2_storage::BagMetadata metadata{};
  metadata.storage_identifier = "sqlite3";
  metadata.encoding = "cdr";
  metadata.relative_file_paths.emplace_back("some_relative_path");
  metadata.relative_file_paths.emplace_back("some_other_relative_path");
  metadata.combined_bag_size = 10;
  metadata.duration_in_nanoseconds = std::chrono::nanoseconds(100);
  metadata.time_start_in_nanoseconds = std::chrono::nanoseconds(10000);
  metadata.message_count = 50;
  metadata.topics_with_message_count.push_back({{"topic1", "type1"}, 100});
  metadata.topics_with_message_count.push_back({{"topic2", "type2"}, 200});

  rosbag2_storage::write_metadata(yaml_file_name_, metadata);
  auto read_metadata = rosbag2_storage::read_metadata(yaml_file_name_);

  EXPECT_THAT(read_metadata.storage_identifier, Eq(metadata.storage_identifier));
  EXPECT_THAT(read_metadata.encoding, Eq(metadata.encoding));
  EXPECT_THAT(read_metadata.relative_file_paths, Eq(metadata.relative_file_paths));
  EXPECT_THAT(read_metadata.combined_bag_size, Eq(metadata.combined_bag_size));
  EXPECT_THAT(read_metadata.duration_in_nanoseconds, Eq(metadata.duration_in_nanoseconds));
  EXPECT_THAT(read_metadata.time_start_in_nanoseconds, Eq(metadata.time_start_in_nanoseconds));
  EXPECT_THAT(read_metadata.message_count, Eq(metadata.message_count));
  EXPECT_THAT(read_metadata.topics_with_message_count,
    SizeIs(metadata.topics_with_message_count.size()));
  auto actual_first_topic = read_metadata.topics_with_message_count[0];
  auto expected_first_topic = read_metadata.topics_with_message_count[0];
  EXPECT_THAT(actual_first_topic.topic_with_type.name,
    Eq(expected_first_topic.topic_with_type.name));
  EXPECT_THAT(actual_first_topic.topic_with_type.type,
    Eq(expected_first_topic.topic_with_type.type));
  EXPECT_THAT(actual_first_topic.message_count, Eq(expected_first_topic.message_count));
  auto actual_second_topic = read_metadata.topics_with_message_count[1];
  auto expected_second_topic = read_metadata.topics_with_message_count[1];
  EXPECT_THAT(actual_second_topic.topic_with_type.name,
    Eq(expected_second_topic.topic_with_type.name));
  EXPECT_THAT(actual_second_topic.topic_with_type.type,
    Eq(expected_second_topic.topic_with_type.type));
  EXPECT_THAT(actual_second_topic.message_count, Eq(expected_second_topic.message_count));
}
