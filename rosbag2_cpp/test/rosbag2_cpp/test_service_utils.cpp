// Copyright 2023 Sony Group Corporation.
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
#include  <gmock/gmock.h>

#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "rosbag2_cpp/service_utils.hpp"
#include "rosbag2_test_common/memory_management.hpp"
#include "test_msgs/srv/basic_types.hpp"

using namespace ::testing;  // NOLINT
using namespace rosbag2_test_common; // NOLINT

class ServiceUtilsTest : public Test
{
public:
  MemoryManagement memory_management_;
};

TEST_F(ServiceUtilsTest, check_is_service_event_topic)
{
  std::vector<std::pair<std::tuple<std::string, std::string>, bool>> all_test_data =
  {
    {{"/abc/_service_event", "package/srv/xyz_Event"}, true},
    {{"/_service_event", "package/srv/xyz_Event"}, false},
    {{"/abc/service_event", "package/srv/xyz_Event"}, false},
    {{"/abc/_service_event", "package/xyz_Event"}, false},
    {{"/abc/_service_event", "package/srv/xyz"}, false}
  };

  for (const auto & test_data : all_test_data) {
    EXPECT_TRUE(
      rosbag2_cpp::is_service_event_topic(
        std::get<0>(test_data.first), std::get<1>(test_data.first)) == test_data.second);
  }
}

TEST_F(ServiceUtilsTest, check_service_event_topic_name_to_service_name)
{
  std::vector<std::pair<std::string, std::string>> all_test_data =
  {
    {"/abc/_service_event", "/abc"},
    {"/_service_event", ""},
    {"/abc/service_event", ""}
  };

  for (const auto & test_data : all_test_data) {
    EXPECT_TRUE(
      rosbag2_cpp::service_event_topic_name_to_service_name(test_data.first) == test_data.second);
  }
}

TEST_F(ServiceUtilsTest, check_service_event_topic_type_to_service_type)
{
  std::vector<std::pair<std::string, std::string>> all_test_data =
  {
    {"package/srv/xyz_Event", "package/srv/xyz"},
    {"package/xyz_Event", ""},
    {"package/srv/Event", ""}
  };

  for (const auto & test_data : all_test_data) {
    EXPECT_EQ(
      rosbag2_cpp::service_event_topic_type_to_service_type(test_data.first),
      test_data.second
    );
  }
}

TEST_F(ServiceUtilsTest, check_service_name_to_service_event_topic_name)
{
  std::vector<std::pair<std::string, std::string>> all_test_data =
  {
    {"", ""},
    {"/a/_service_event", "/a/_service_event"},
    {"/abc", "/abc/_service_event"}
  };

  for (const auto & test_data : all_test_data) {
    EXPECT_EQ(
      rosbag2_cpp::service_name_to_service_event_topic_name(test_data.first),
      test_data.second
    );
  }
}

TEST_F(ServiceUtilsTest, check_client_id_to_string)
{
  service_msgs::msg::ServiceEventInfo::_client_gid_type client_id = {
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16
  };
  std::string expected_string = "1-2-3-4-5-6-7-8-9-10-11-12-13-14-15-16";

  EXPECT_EQ(rosbag2_cpp::client_id_to_string(client_id), expected_string);
}
