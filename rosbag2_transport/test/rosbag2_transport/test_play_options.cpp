// Copyright 2026 Old-Ding
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
#include <sstream>
#include <string>

#include "rosbag2_transport/play_options.hpp"

TEST(play_options, yaml_round_trip_preserves_encoded_options)
{
  rosbag2_transport::PlayOptions original;
  original.read_ahead_queue_size = 42;
  original.node_prefix = "prefix";
  original.rate = 2.5f;
  original.topics_to_filter = {"topic", "other_topic"};
  original.services_to_filter = {"service", "other_service"};
  original.actions_to_filter = {"action", "other_action"};
  original.regex_to_filter = "include.*";
  original.exclude_regex_to_filter = "exclude.*";
  original.exclude_topics_to_filter = {"excluded_topic"};
  original.exclude_services_to_filter = {"excluded_service"};
  original.exclude_actions_to_filter = {"excluded_action", "other_excluded_action"};
  original.topic_qos_profile_overrides.emplace(
    "topic", rclcpp::QoS(10).transient_local());
  original.loop = true;
  original.topic_remapping_options = {"--ros-args", "-r", "input:=output"};
  original.clock_publish_frequency = 10.0;
  original.clock_publish_on_topic_publish = true;
  original.clock_trigger_topics = {"trigger_topic"};
  original.delay = rclcpp::Duration(1, 2);
  original.playback_duration = rclcpp::Duration(3, 4);
  original.playback_until_timestamp = 5;
  original.start_paused = true;
  original.start_offset = 6;
  original.disable_keyboard_controls = true;
  original.wait_acked_timeout = 7;
  original.disable_loan_message = true;
  original.progress_bar_update_rate = 8;
  original.progress_bar_separation_lines = 9;

  auto node = YAML::convert<rosbag2_transport::PlayOptions>::encode(original);
  std::stringstream serializer;
  serializer << node;
  auto reconstructed = YAML::Load(serializer.str()).as<rosbag2_transport::PlayOptions>();

  #define CHECK(field) EXPECT_EQ(original.field, reconstructed.field)
  CHECK(read_ahead_queue_size);
  CHECK(node_prefix);
  CHECK(rate);
  CHECK(topics_to_filter);
  CHECK(services_to_filter);
  CHECK(actions_to_filter);
  CHECK(regex_to_filter);
  CHECK(exclude_regex_to_filter);
  CHECK(exclude_topics_to_filter);
  CHECK(exclude_services_to_filter);
  CHECK(exclude_actions_to_filter);
  CHECK(topic_qos_profile_overrides);
  CHECK(loop);
  CHECK(topic_remapping_options);
  CHECK(clock_publish_frequency);
  CHECK(clock_publish_on_topic_publish);
  CHECK(clock_trigger_topics);
  CHECK(delay);
  CHECK(playback_duration);
  CHECK(playback_until_timestamp);
  CHECK(start_paused);
  CHECK(start_offset);
  CHECK(disable_keyboard_controls);
  CHECK(wait_acked_timeout);
  CHECK(disable_loan_message);
  CHECK(progress_bar_update_rate);
  CHECK(progress_bar_separation_lines);
  #undef CHECK
}

TEST(play_options, yaml_round_trip_preserves_default_negative_ack_timeout)
{
  rosbag2_transport::PlayOptions original;

  auto node = YAML::convert<rosbag2_transport::PlayOptions>::encode(original);
  EXPECT_EQ(-1, node["wait_acked_timeout"]["sec"].as<int32_t>());
  EXPECT_EQ(999999999U, node["wait_acked_timeout"]["nsec"].as<uint32_t>());

  std::stringstream serializer;
  serializer << node;
  auto reconstructed = YAML::Load(serializer.str()).as<rosbag2_transport::PlayOptions>();

  EXPECT_EQ(original.wait_acked_timeout, reconstructed.wait_acked_timeout);
}
