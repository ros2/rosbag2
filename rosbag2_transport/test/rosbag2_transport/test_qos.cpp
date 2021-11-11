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

#include "rosbag2_transport/qos.hpp"

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
  auto actual_qos = deserialized_profiles[0].get_rmw_qos_profile();

  rmw_time_t zerotime{0, 0};
  // Explicitly set up the same QoS profile in case defaults change
  auto expected_qos = rosbag2_transport::Rosbag2QoS{}
  .default_history()
  .reliable()
  .durability_volatile()
  .deadline(zerotime)
  .lifespan(zerotime)
  .liveliness(RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT)
  .liveliness_lease_duration(zerotime).get_rmw_qos_profile();

  EXPECT_EQ(actual_qos.reliability, expected_qos.reliability);
  EXPECT_EQ(actual_qos.durability, expected_qos.durability);
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

TEST(TestQoS, translates_bad_infinity_values)
{
  // Copied from hidden symbols in qos.cpp
  const rmw_time_t bad_infinities[3] {
    rmw_time_from_nsec(0x7FFFFFFFFFFFFFFFll),  // cyclone
    {0x7FFFFFFFll, 0xFFFFFFFFll},  // fastrtps
    {0x7FFFFFFFll, 0x7FFFFFFFll}  // connext
  };
  rmw_time_t infinity = RMW_DURATION_INFINITE;
  const auto expected_qos = rosbag2_transport::Rosbag2QoS{}
  .default_history()
  .reliable()
  .durability_volatile()
  .deadline(infinity)
  .lifespan(infinity)
  .liveliness(RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT)
  .liveliness_lease_duration(infinity)
  .get_rmw_qos_profile();

  for (const auto & infinity : bad_infinities) {
    std::ostringstream serialized_profile;
    serialized_profile <<
      "history: 1\n"
      "depth: 10\n"
      "reliability: 1\n"
      "durability: 2\n"
      "deadline:\n"
      "  sec: " << infinity.sec << "\n"
      "  nsec: " << infinity.nsec << "\n"
      "lifespan:\n"
      "  sec: " << infinity.sec << "\n"
      "  nsec: " << infinity.nsec << "\n"
      "liveliness: 0\n"
      "liveliness_lease_duration:\n"
      "  sec: " << infinity.sec << "\n"
      "  nsec: " << infinity.nsec << "\n"
      "avoid_ros_namespace_conventions: false\n";
    const YAML::Node loaded_node = YAML::Load(serialized_profile.str());
    const auto deserialized_profile = loaded_node.as<rosbag2_transport::Rosbag2QoS>();
    const auto actual_qos = deserialized_profile.get_rmw_qos_profile();
    EXPECT_TRUE(rmw_time_equal(actual_qos.lifespan, expected_qos.lifespan));
    EXPECT_TRUE(rmw_time_equal(actual_qos.deadline, expected_qos.deadline));
    EXPECT_TRUE(
      rmw_time_equal(
        actual_qos.liveliness_lease_duration, expected_qos.liveliness_lease_duration));
  }
}

using rosbag2_transport::Rosbag2QoS;  // NOLINT
class AdaptiveQoSTest : public ::testing::Test
{
public:
  AdaptiveQoSTest() = default;

  rclcpp::TopicEndpointInfo make_endpoint(const rclcpp::QoS & qos)
  {
    rcl_topic_endpoint_info_t endpoint_info {
      "some_node_name",
      "some_node_namespace",
      "some_topic_type",
      RMW_ENDPOINT_PUBLISHER,
      {0},
      qos.get_rmw_qos_profile(),
    };

    return rclcpp::TopicEndpointInfo(endpoint_info);
  }

  void add_endpoint(const rclcpp::QoS & qos)
  {
    endpoints_.push_back(make_endpoint(qos));
  }

  const std::string topic_name_{"/topic"};
  std::vector<rclcpp::TopicEndpointInfo> endpoints_{};
  const rosbag2_transport::Rosbag2QoS default_offer_{};
  const rosbag2_transport::Rosbag2QoS default_request_{};
};

TEST_F(AdaptiveQoSTest, adapt_request_empty_returns_default)
{
  auto adapted_request = Rosbag2QoS::adapt_request_to_offers(topic_name_, endpoints_);
  EXPECT_EQ(default_request_, adapted_request);
}

TEST_F(AdaptiveQoSTest, adapt_request_single_offer_returns_same_values)
{
  // Set up this offer to use nondefault reliability and durability,
  // expect to see those values in the output
  auto nondefault_offer = Rosbag2QoS{}.best_effort().transient_local();
  add_endpoint(nondefault_offer);

  auto adapted_request = Rosbag2QoS::adapt_request_to_offers(topic_name_, endpoints_);

  auto expected = nondefault_offer.get_rmw_qos_profile();
  auto actual = adapted_request.get_rmw_qos_profile();
  EXPECT_EQ(expected.reliability, actual.reliability);
  EXPECT_EQ(expected.durability, actual.durability);
}

TEST_F(AdaptiveQoSTest, adapt_request_multiple_similar_offers_returns_same_values)
{
  auto nondefault_offer = Rosbag2QoS{}.best_effort().transient_local();
  const size_t num_endpoints{3};
  for (size_t i = 0; i < num_endpoints; i++) {
    add_endpoint(nondefault_offer);
  }

  auto adapted_request = Rosbag2QoS::adapt_request_to_offers(topic_name_, endpoints_);

  auto expected = nondefault_offer.get_rmw_qos_profile();
  auto actual = adapted_request.get_rmw_qos_profile();
  EXPECT_EQ(expected.reliability, actual.reliability);
  EXPECT_EQ(expected.durability, actual.durability);
}

TEST_F(AdaptiveQoSTest, adapt_request_mixed_reliability_offers_return_best_effort)
{
  add_endpoint(Rosbag2QoS{}.best_effort());
  add_endpoint(Rosbag2QoS{}.reliable());
  auto adapted_request = Rosbag2QoS::adapt_request_to_offers(topic_name_, endpoints_);
  EXPECT_EQ(
    adapted_request.get_rmw_qos_profile().reliability, RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
}

TEST_F(AdaptiveQoSTest, adapt_request_mixed_durability_offers_return_volatile)
{
  add_endpoint(Rosbag2QoS{}.transient_local());
  add_endpoint(Rosbag2QoS{}.durability_volatile());
  auto adapted_request = Rosbag2QoS::adapt_request_to_offers(topic_name_, endpoints_);
  EXPECT_EQ(adapted_request.get_rmw_qos_profile().durability, RMW_QOS_POLICY_DURABILITY_VOLATILE);
}

TEST_F(AdaptiveQoSTest, adapt_offer_empty_returns_default)
{
  auto adapted_offer = Rosbag2QoS::adapt_offer_to_recorded_offers(topic_name_, {});
  EXPECT_EQ(adapted_offer, default_offer_);
}

TEST_F(AdaptiveQoSTest, adapt_offer_single_offer_returns_same_values)
{
  // Set up this offer to use nondefault reliability and durability,
  // expect to see those values in the output
  auto nondefault_offer = Rosbag2QoS{Rosbag2QoS{}.best_effort().transient_local()};
  std::vector<Rosbag2QoS> offers = {nondefault_offer};

  auto adapted_offer = Rosbag2QoS::adapt_offer_to_recorded_offers(topic_name_, offers);
  EXPECT_EQ(nondefault_offer, adapted_offer);
}

TEST_F(AdaptiveQoSTest, adapt_offer_multiple_offers_with_same_settings_return_identical)
{
  auto nondefault_offer = Rosbag2QoS{Rosbag2QoS{}.best_effort().transient_local()};
  auto adapted_offer = Rosbag2QoS::adapt_offer_to_recorded_offers(
    topic_name_, {nondefault_offer, nondefault_offer, nondefault_offer});
  EXPECT_EQ(nondefault_offer, adapted_offer);
}

TEST_F(AdaptiveQoSTest, adapt_offer_mixed_compatibility_returns_default)
{
  // When the offers have mixed values for policies that affect compatibility,
  // it should fall back to the default.
  std::vector<Rosbag2QoS> offers = {
    Rosbag2QoS{Rosbag2QoS{}.best_effort()},
    Rosbag2QoS{Rosbag2QoS{}.reliable()},
  };
  auto adapted_offer = Rosbag2QoS::adapt_offer_to_recorded_offers(topic_name_, offers);
  EXPECT_EQ(adapted_offer, default_offer_);
}

TEST_F(AdaptiveQoSTest, adapt_offer_mixed_non_compatibility_returns_first)
{
  // Some QoS policies don't affect compatibility, so even if their values are mixed we should
  // receive the first value.
  rclcpp::Duration nonstandard_duration(12, 34);
  size_t nonstandard_history{20};
  std::vector<Rosbag2QoS> offers = {
    Rosbag2QoS{Rosbag2QoS{}.lifespan(nonstandard_duration)},
    Rosbag2QoS{Rosbag2QoS{}.keep_last(nonstandard_history)},
  };
  auto adapted_offer = Rosbag2QoS::adapt_offer_to_recorded_offers(topic_name_, offers);
  EXPECT_EQ(adapted_offer, offers[0]);
}
