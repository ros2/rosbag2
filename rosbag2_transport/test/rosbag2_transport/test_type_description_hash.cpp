// Copyright 2023, Foxglove
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

#include "rosbag2_transport/recorder.hpp"

using namespace ::testing;  // NOLINT

rosidl_type_hash_t all_zeros()
{
  rosidl_type_hash_t type_hash;
  type_hash.version = 0;
  for (int i = 0; i < ROSIDL_TYPE_HASH_SIZE; ++i) {
    type_hash.value[i] = 0;
  }
  return type_hash;
}

rosidl_type_hash_t valid_v1_type_hash()
{
  rosidl_type_hash_t type_hash;
  type_hash.version = 1;
  for (int i = 0; i < ROSIDL_TYPE_HASH_SIZE; ++i) {
    type_hash.value[i] = i;
  }
  return type_hash;
}

TEST(TestTopicTypeHash, test_formatting) {
  auto type_hash = valid_v1_type_hash();
  std::string result = rosbag2_transport::type_hash_to_string(type_hash);
  EXPECT_EQ("RIHS01_000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f", result);
  type_hash.version = 0;
  std::string empty = rosbag2_transport::type_hash_to_string(type_hash);
  EXPECT_EQ("", empty);
  type_hash.version = 2;
  std::string unrecognized_version = rosbag2_transport::type_hash_to_string(type_hash);
  EXPECT_EQ("", unrecognized_version);
}

rclcpp::TopicEndpointInfo make_info(rosidl_type_hash_t hash)
{
  rcl_topic_endpoint_info_t info;
  info.topic_type_hash = hash;
  info.topic_type = "topic_type";
  info.node_name = "node_name";
  info.node_namespace = "node_namespace";
  info.endpoint_type = RMW_ENDPOINT_INVALID;
  return rclcpp::TopicEndpointInfo(info);
}

TEST(TestTopicTypeHash, test_no_endpoints) {
  std::vector<rclcpp::TopicEndpointInfo> endpoints {};
  std::string result = rosbag2_transport::type_description_hash_for_topic(endpoints);
  // no type hash available
  EXPECT_EQ("", result);
}

TEST(TestTopicTypeHash, test_two_zero_type_hashes) {
  std::vector<rclcpp::TopicEndpointInfo> endpoints {
    make_info(all_zeros()),
    make_info(all_zeros()),
  };
  std::string result = rosbag2_transport::type_description_hash_for_topic(endpoints);
  // no type hash available
  EXPECT_EQ("", result);
}

TEST(TestTopicTypeHash, test_two_agreeing_infos) {
  std::vector<rclcpp::TopicEndpointInfo> endpoints {
    make_info(valid_v1_type_hash()),
    make_info(valid_v1_type_hash()),
  };
  std::string result = rosbag2_transport::type_description_hash_for_topic(endpoints);
  EXPECT_EQ("RIHS01_000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f", result);
}

TEST(TestTopicTypeHash, test_one_empty_info) {
  std::vector<rclcpp::TopicEndpointInfo> endpoints {
    make_info(all_zeros()),
    make_info(valid_v1_type_hash()),
    make_info(valid_v1_type_hash()),
  };
  std::string result = rosbag2_transport::type_description_hash_for_topic(endpoints);
  EXPECT_EQ("RIHS01_000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f", result);
}

TEST(TestTopicTypeHash, test_disagreeing_info_versions) {
  rosidl_type_hash_t v2_type_hash = all_zeros();
  v2_type_hash.version = 2;
  std::vector<rclcpp::TopicEndpointInfo> endpoints {
    make_info(rosidl_get_zero_initialized_type_hash()),
    make_info(valid_v1_type_hash()),
    make_info(v2_type_hash),
  };
  std::string result = rosbag2_transport::type_description_hash_for_topic(endpoints);
  // type hashes disagree, should return empty string and warn.
  EXPECT_EQ("", result);
}

TEST(TestTopicTypeHash, test_disagreeing_info_values) {
  rosidl_type_hash_t other_v1_type_hash = valid_v1_type_hash();
  other_v1_type_hash.value[2] = 0;
  std::vector<rclcpp::TopicEndpointInfo> endpoints {
    make_info(rosidl_get_zero_initialized_type_hash()),
    make_info(valid_v1_type_hash()),
    make_info(other_v1_type_hash),
  };
  std::string result = rosbag2_transport::type_description_hash_for_topic(endpoints);
  // type hashes disagree, should return empty string and warn.
  EXPECT_EQ("", result);
}
