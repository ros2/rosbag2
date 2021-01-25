// Copyright 2021, Robotec.ai sp. z o.o.
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

#include <regex>
#include <string>

#include "test_msgs/msg/arrays.hpp"
#include "test_msgs/msg/basic_types.hpp"
#include "test_msgs/message_fixtures.hpp"

#include "record_integration_fixture.hpp"

TEST_F(RecordIntegrationTestFixture, only_regex_matching_topics_are_recorded)
{
  auto test_string_messages = get_messages_strings();
  auto test_array_messages = get_messages_arrays();
  std::string regex = "/[a-z]+_nice(_.*)";
  std::string regex_exclude = "/[a-z]+_nice_[a-z]+/(.*)";

  // matching topics - the only ones that should be recorded
  std::string v1 = "/awesome_nice_topic";
  std::string v2 = "/still_nice_topic";

  // excluded topics
  std::string e1 = "/quite_nice_namespace/but_it_is_excluded";

  // topics that shouldn't match
  std::string b1 = "/numberslike1arenot_nice";
  std::string b2 = "/namespace_before/not_nice";

  // checking the test data itself
  std::regex re(regex);
  std::regex exclude(regex_exclude);
  ASSERT_TRUE(std::regex_search(v1, re));
  ASSERT_TRUE(std::regex_search(v2, re));
  ASSERT_FALSE(std::regex_search(b1, re));
  ASSERT_FALSE(std::regex_search(b2, re));

  // this example matches both regexes - should be excluded
  ASSERT_TRUE(std::regex_search(e1, re));
  ASSERT_TRUE(std::regex_search(e1, exclude));

  RecordOptions ro{false, false, {}, "rmw_format", 10ms};
  ro.regex = regex;
  ro.exclude = regex_exclude;

  start_recording(ro);

  pub_man_.add_publisher<test_msgs::msg::Strings>(
    v1, test_string_messages[0], 0);
  pub_man_.add_publisher<test_msgs::msg::Strings>(
    v2, test_string_messages[1], 0);

  pub_man_.add_publisher<test_msgs::msg::Strings>(
    b1, test_string_messages[0], 0);
  pub_man_.add_publisher<test_msgs::msg::Strings>(
    b2, test_string_messages[1], 0);

  pub_man_.add_publisher<test_msgs::msg::Arrays>(
    e1, test_array_messages[0], 0);

  run_publishers();
  stop_recording();

  MockSequentialWriter & writer =
    static_cast<MockSequentialWriter &>(writer_->get_implementation_handle());
  auto recorded_topics = writer.get_topics();

  EXPECT_THAT(recorded_topics, SizeIs(2));

  EXPECT_TRUE(recorded_topics.find(v1) != recorded_topics.end());
  EXPECT_TRUE(recorded_topics.find(v2) != recorded_topics.end());
}
