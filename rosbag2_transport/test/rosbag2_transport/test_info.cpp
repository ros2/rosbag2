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

#include "rosbag2/types.hpp"
#include "rosbag2_transport/rosbag2_transport.hpp"
#include "rosbag2_transport_test_fixture.hpp"

using namespace ::testing;  // NOLINT

TEST_F(Rosbag2TransportTestFixture, info_pretty_prints_information_from_bagfile)
{
  internal::CaptureStdout();

  rosbag2::BagMetadata bagfile;
  bagfile.storage_identifier = "sqlite3";
  bagfile.relative_file_paths.emplace_back("some_relative_path");
  bagfile.relative_file_paths.emplace_back("some_other_relative_path");
  bagfile.starting_time = std::chrono::time_point<std::chrono::high_resolution_clock>(
    std::chrono::nanoseconds(1538051985348887471));    // corresponds to Sept 27 14:39:45.348
  bagfile.duration = std::chrono::nanoseconds(50000000);
  bagfile.message_count = 50;
  bagfile.topics_with_message_count.push_back({{"topic1", "type1", "rmw1"}, 100});
  bagfile.topics_with_message_count.push_back({{"topic2", "type2", "rmw2"}, 200});
  EXPECT_CALL(*info_, read_metadata(_, _)).WillOnce(Return(bagfile));

  // the expected output uses a regex to handle different time zones.
  rosbag2_transport::Rosbag2Transport transport(reader_, writer_, info_);
  transport.print_bag_info("test");
  std::string expected_output(
    "\nFiles:            some_relative_path\n"
    "                  some_other_relative_path\n"
    "Bag size:         0 B\n"
    "Storage id:       sqlite3\n"
    "Duration:         0\\.50s\n"
    "Start:            Sep .+ 2018 .+:.+:45\\.348 \\(1538051985\\.348\\)\n"
    "End               Sep .+ 2018 .+:.+:45\\.398 \\(1538051985\\.398\\)\n"
    "Messages:         50\n"
    "Topics with Type: topic1; type1; 100 msgs; rmw1\n"
    "                  topic2; type2; 200 msgs; rmw2\n\n");

  std::string output = internal::GetCapturedStdout();
  EXPECT_THAT(output, ContainsRegex(expected_output));
}
