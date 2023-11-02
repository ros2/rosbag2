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

#include "rosbag2_transport/record_options.hpp"

using namespace ::testing;  // NOLINT

TEST(record_options, test_yaml_serialization)
{
  rosbag2_transport::RecordOptions original;
  original.all = true;
  original.is_discovery_disabled = true;
  original.topics = {"topic", "other_topic"};
  original.rmw_serialization_format = "cdr";
  original.topic_polling_interval = std::chrono::milliseconds{200};
  original.regex = "[xyz]/topic";
  original.exclude = "*";
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
  CHECK(all);
  CHECK(is_discovery_disabled);
  CHECK(topics);
  CHECK(rmw_serialization_format);
  #undef CMP
}
