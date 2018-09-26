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

#include <future>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rosbag2_transport/rosbag2_transport.hpp"
#include "rosbag2_transport_test_fixture.hpp"

using namespace ::testing;  // NOLINT

TEST_F(Rosbag2TransportTestFixture, info_pretty_prints_information_from_bagfile)
{
  internal::CaptureStdout();

  rosbag2::BagMetadata bagfile;
  bagfile.storage_identifier = "sqlite3";
  bagfile.encoding = "cdr";
  bagfile.relative_file_paths.emplace_back("some_relative_path");
  bagfile.relative_file_paths.emplace_back("some_other_relative_path");
  bagfile.duration = std::chrono::nanoseconds(100);
  bagfile.starting_time =
    std::chrono::time_point<std::chrono::high_resolution_clock>(std::chrono::nanoseconds(1000000));
  bagfile.message_count = 50;
  bagfile.topics_with_message_count.push_back({{"topic1", "type1"}, 100});
  bagfile.topics_with_message_count.push_back({{"topic2", "type2"}, 200});
  EXPECT_CALL(*factory_->info_, read_metadata(_)).WillOnce(Return(bagfile));

  auto transport = rosbag2_transport::Rosbag2Transport(factory_);
  transport.print_bag_info("test");
  std::string expected_output(
    "Storage identifier:  sqlite3\n"
    "File encoding:       cdr\n"
    "Associated files (relative paths):\n"
    "        - some_relative_path\n"
    "        - some_other_relative_path\n"
    "Starting time:       1000000\n"
    "End time:            1000100\n"
    "Duration:            100\n"
    "Total message count: 50\n"
    "Topics with Type and message count:\n"
    "        - topic1 ; type1 ; 100\n"
    "        - topic2 ; type2 ; 200\n");

  std::string output = internal::GetCapturedStdout();
  EXPECT_THAT(output, HasSubstr(expected_output));
}
