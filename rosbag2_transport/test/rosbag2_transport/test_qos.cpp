// Copyright 2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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
#include <vector>

#include "rmw/types.h"

#include "qos.hpp"

TEST(TestQoS, serialization)
{
  rosbag2_transport::Rosbag2QoS expected_qos;
  YAML::Node offered_qos_profiles;
  offered_qos_profiles.push_back(expected_qos);

  std::string serialized = YAML::Dump(offered_qos_profiles);
  YAML::Node loaded_node = YAML::Load(serialized);
  auto deserialized_profiles = loaded_node.as<std::vector<rosbag2_transport::Rosbag2QoS>>();
  ASSERT_EQ(deserialized_profiles.size(), 1u);

  rosbag2_transport::Rosbag2QoS actual_qos = deserialized_profiles[0];
  EXPECT_EQ(actual_qos, expected_qos);
}

TEST(TestQoS, supports_version_4)
{
  // This test shows how this data looks at the time of introduction
  // Bags created by this version of rosbag2 look like this and must be able to be read
  std::string serialized_profiles =
    "- history: 1\n"
    "  depth: 10\n"
    "  reliability: 1\n"
    "  durability: 2\n"
    "  deadline:\n"
    "    sec: 0\n"
    "    nsec: 0\n"
    "  lifespan:\n"
    "    sec: 0\n"
    "    nsec: 0\n"
    "  liveliness: 0\n"
    "  liveliness_lease_duration:\n"
    "    sec: 0\n"
    "    nsec: 0\n"
    "  avoid_ros_namespace_conventions: false\n";

  YAML::Node loaded_node = YAML::Load(serialized_profiles);
  auto deserialized_profiles = loaded_node.as<std::vector<rosbag2_transport::Rosbag2QoS>>();
  ASSERT_EQ(deserialized_profiles.size(), 1u);
  rosbag2_transport::Rosbag2QoS actual_qos = deserialized_profiles[0];

  rmw_time_t zerotime{0, 0};
  // Explicitly set up the same QoS profile in case defaults change
  rclcpp::QoS expected_qos(10);
  expected_qos
  .keep_last(10)
  .reliable()
  .durability_volatile()
  .deadline(zerotime)
  .lifespan(zerotime)
  .liveliness(RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT)
  .liveliness_lease_duration(zerotime)
  .avoid_ros_namespace_conventions(false);
  // Any values not present in the YAML should take the default value in both profiles
  EXPECT_EQ(actual_qos, expected_qos);
}

TEST(TestQoS, detect_new_qos_fields)
{
  // By trying to construct a profile explicitly by fields, the build fails if policies are added
  // This build failure indicates that we need to update QoS serialization in rosbag2_transport
  rmw_time_t notime{0, 0};
  rmw_qos_profile_t profile{
    RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT,
    10,
    RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,
    RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT,
    notime,
    notime,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    notime,
    false,
  };
  EXPECT_EQ(profile.history, RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT);  // fix "unused variable"
}
