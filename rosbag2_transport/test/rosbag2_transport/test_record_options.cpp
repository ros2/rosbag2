// Copyright 2021 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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
#include <string>

#include "rosbag2_transport/record_options.hpp"

using namespace ::testing;  // NOLINT

TEST(record_options, test_yaml_serialization)
{
  rosbag2_transport::RecordOptions original;
  original.all_topics = true;
  original.all_services = true;
  original.is_discovery_disabled = true;
  original.topics = {"topic", "other_topic"};
  original.services = {"service", "other_service"};
  original.exclude_topics = {"exclude_topic1", "exclude_topic2"};
  original.exclude_service_events = {"exclude_service1", "exclude_service2"};
  original.rmw_serialization_format = "cdr";
  original.topic_polling_interval = std::chrono::milliseconds{200};
  original.regex = "[xyz]/topic";
  original.exclude_regex = "[x]/topic";
  original.node_prefix = "prefix";
  original.compression_mode = "stream";
  original.compression_format = "h264";
  original.compression_queue_size = 2;
  original.compression_threads = 123;
  original.topic_qos_profile_overrides.emplace("topic", rclcpp::QoS(10).transient_local());
  original.include_hidden_topics = true;
  original.include_unpublished_topics = true;

  auto node = YAML::convert<rosbag2_transport::RecordOptions>().encode(original);

  std::stringstream serializer;
  serializer << node;
  auto reconstructed_node = YAML::Load(serializer.str());
  auto reconstructed = reconstructed_node.as<rosbag2_transport::RecordOptions>();

  #define CHECK(field) ASSERT_EQ(original.field, reconstructed.field)
  CHECK(all_topics);
  CHECK(all_services);
  CHECK(is_discovery_disabled);
  CHECK(topics);
  CHECK(services);
  CHECK(exclude_topics);
  CHECK(exclude_service_events);
  CHECK(rmw_serialization_format);
  #undef CHECK
}

TEST(record_options, test_yaml_decode_for_all_and_exclude)
{
  std::string serialized_record_options =
    "  all: true\n"
    "  all_topics: false\n"
    "  topics: []\n"
    "  rmw_serialization_format: \"\"  # defaults to using the format of the input topic\n"
    "  regex: \"[xyz]/topic\"\n"
    "  exclude: \"[x]/topic\"\n";

  YAML::Node loaded_node = YAML::Load(serialized_record_options);
  auto record_options = loaded_node.as<rosbag2_transport::RecordOptions>();
  ASSERT_EQ(record_options.all_topics, true);
  ASSERT_EQ(record_options.all_services, true);
  ASSERT_EQ(record_options.regex, "[xyz]/topic");
  ASSERT_EQ(record_options.exclude_regex, "[x]/topic");
}
