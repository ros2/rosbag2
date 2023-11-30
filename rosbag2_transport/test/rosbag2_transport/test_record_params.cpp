// Copyright 2023, Patrick Roncagliolo.
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
#include <memory>
#include <string>
#include <vector>

#include "mock_recorder.hpp"

using namespace std::chrono_literals;

class RecordParamsTestFixture : public testing::Test
{
public:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }
};

TEST_F(RecordParamsTestFixture, parse_parameter_from_file) {
  // _SRC_RESOURCES_DIR_PATH defined in CMakeLists.txt
  rclcpp::NodeOptions opts;
  opts.arguments(
  {
    "--ros-args",
    "--params-file", _SRC_RESOURCES_DIR_PATH "/recorder_node_params.yaml"
  });
  opts.append_parameter_override(
    "qos_profile_overrides_path",
    _SRC_RESOURCES_DIR_PATH "/qos_profile_overrides.yaml");

  auto recorder = std::make_shared<MockRecorder>("recorder_params_node", opts);
  auto record_options = recorder->get_record_options();
  auto storage_options = recorder->get_storage_options();

  EXPECT_EQ(record_options.all, true);
  EXPECT_EQ(record_options.is_discovery_disabled, true);
  std::vector<std::string> topics {"/topic", "/other_topic"};
  EXPECT_EQ(record_options.topics, topics);
  EXPECT_EQ(record_options.rmw_serialization_format, "cdr");
  EXPECT_TRUE(record_options.topic_polling_interval == 0.01s);
  EXPECT_EQ(record_options.regex, "[xyz]/topic");
  EXPECT_EQ(record_options.exclude, "*");
  EXPECT_EQ(record_options.node_prefix, "prefix");
  EXPECT_EQ(record_options.compression_mode, "stream");
  EXPECT_EQ(record_options.compression_format, "h264");
  EXPECT_EQ(record_options.compression_queue_size, 10);
  EXPECT_EQ(record_options.compression_threads, 2);
  std::unordered_map<std::string, rclcpp::QoS> topic_qos_profile_overrides{
    std::pair{
      "/overrided_topic_qos",
      rclcpp::QoS{rclcpp::KeepLast(10)}.reliable().durability_volatile()}
  };
  EXPECT_EQ(record_options.topic_qos_profile_overrides, topic_qos_profile_overrides);
  EXPECT_EQ(record_options.include_hidden_topics, true);
  EXPECT_EQ(record_options.include_unpublished_topics, true);
  EXPECT_EQ(record_options.ignore_leaf_topics, false);
  EXPECT_EQ(record_options.start_paused, false);
  EXPECT_EQ(record_options.use_sim_time, false);

  EXPECT_EQ(storage_options.uri, "path/to/some_bag");
  EXPECT_EQ(storage_options.storage_id, "sqlite3");
  EXPECT_EQ(storage_options.storage_config_uri, "");
  EXPECT_EQ(storage_options.max_bagfile_size, 12345);
  EXPECT_EQ(storage_options.max_bagfile_duration, 54321);
  EXPECT_EQ(storage_options.max_cache_size, 9898);
  EXPECT_EQ(storage_options.storage_preset_profile, "resilient");
  EXPECT_EQ(storage_options.snapshot_mode, false);
  std::unordered_map<std::string, std::string> custom_data{
    std::pair{"key1", "value1"},
    std::pair{"key2", "value2"}
  };
  EXPECT_EQ(storage_options.custom_data, custom_data);
  EXPECT_EQ(storage_options.start_time_ns, 0);
  EXPECT_EQ(storage_options.end_time_ns, 100000);
}
