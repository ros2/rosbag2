// Copyright 2023 Patrick Roncagliolo and Michael Orlov
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
#include <memory>

#include "rosbag2_transport/recorder.hpp"

using namespace std::chrono_literals;

class ComposableRecorderTestFixture : public testing::Test
{
public:
  ComposableRecorderTestFixture()
  {
    rclcpp::init(0, nullptr);
  }

  ~ComposableRecorderTestFixture() override
  {
    rclcpp::shutdown();
  }
};

class ComposableRecorder : public rosbag2_transport::Recorder
{
public:
  static const char demo_attribute_name_[];
  bool demo_attribute_value{false};

  explicit ComposableRecorder(
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions(),
    const std::string & node_name = "rosbag2_mock_composable_recorder")
  : Recorder(node_name, node_options)
  {
    // Declare demo attribute parameter for the underlying node with default value equal to false.
    // However, if node was created with option to override this parameter it will be settled up
    // to what was specified in parameter_overrides value.
    demo_attribute_value = this->declare_parameter<bool>(
      demo_attribute_name_, /*default_value=*/ false,
      rcl_interfaces::msg::ParameterDescriptor(), /*ignore_override=*/ false);
  }

  bool get_value_of_bool_parameter(const std::string & parameter_name)
  {
    bool ret_value{false};
    bool parameter_was_set = this->get_parameter(parameter_name, ret_value);
    if (!parameter_was_set) {
      throw std::runtime_error("Parameter `" + parameter_name + "` hasn't been set.");
    }
    return ret_value;
  }

  using rosbag2_transport::Recorder::get_storage_options;
  using rosbag2_transport::Recorder::get_record_options;
};
const char ComposableRecorder::demo_attribute_name_[] = "demo_attribute";

TEST_F(ComposableRecorderTestFixture, recorder_inner_params_passed_as_append_override)
{
  std::vector<rclcpp::Parameter> parameters;
  parameters.emplace_back(ComposableRecorder::demo_attribute_name_, true);
  auto options = rclcpp::NodeOptions()
    .use_global_arguments(false)
    .parameter_overrides(parameters);

  auto recorder = std::make_shared<ComposableRecorder>(options);
  // Check that rosbag2_transport::Recorder inner params will not erase our
  // parameter_overrides options
  ASSERT_TRUE(recorder->get_value_of_bool_parameter(recorder->demo_attribute_name_));
  ASSERT_TRUE(recorder->demo_attribute_value);
}

TEST_F(ComposableRecorderTestFixture, parse_parameter_from_file) {
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

  auto recorder = std::make_shared<ComposableRecorder>(opts, "recorder_params_node");
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
