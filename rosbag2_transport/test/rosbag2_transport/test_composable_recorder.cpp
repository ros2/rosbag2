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

#include <filesystem>
#include <memory>

#include "composition_manager_test_fixture.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_test_common/publication_manager.hpp"
#include "rosbag2_test_common/memory_management.hpp"
#include "rosbag2_test_common/tested_storage_ids.hpp"
#include "rosbag2_test_common/temporary_directory_fixture.hpp"
#include "rosbag2_transport/recorder.hpp"
#include "test_msgs/message_fixtures.hpp"

using namespace std::chrono_literals;  // NOLINT
using namespace ::testing;  // NOLINT
using namespace rosbag2_test_common;  // NOLINT

class ComposableRecorderIntegrationTests : public CompositionManagerTestFixture
{
public:
  std::string get_bag_file_name(int split_index) const
  {
    std::stringstream bag_file_name;
    bag_file_name << get_test_name() << "_" << GetParam() << "_" << split_index;

    return bag_file_name.str();
  }

  std::filesystem::path get_bag_file_path(int split_index)
  {
    return root_bag_path_ / get_relative_bag_file_path(split_index);
  }

  std::filesystem::path get_relative_bag_file_path(int split_index) const
  {
    const auto storage_id = GetParam();
    return std::filesystem::path(
      rosbag2_test_common::bag_filename_for_storage_id(get_bag_file_name(split_index), storage_id));
  }

  void wait_for_metadata(std::chrono::duration<float> timeout = std::chrono::seconds(10)) const
  {
    rosbag2_storage::MetadataIo metadata_io;
    const auto start_time = std::chrono::steady_clock::now();
    const auto bag_path = root_bag_path_.generic_string();

    while (std::chrono::steady_clock::now() - start_time < timeout && rclcpp::ok()) {
      if (metadata_io.metadata_file_exists(bag_path)) {
        return;
      }
      std::this_thread::sleep_for(50ms);
    }
    ASSERT_EQ(metadata_io.metadata_file_exists(bag_path), true)
      << "Could not find metadata file: \"" << bag_path.c_str() << "\"";
  }

  void wait_for_storage_file(std::chrono::duration<float> timeout = std::chrono::seconds(10))
  {
    const auto storage_path = get_bag_file_path(0);
    const auto start_time = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start_time < timeout && rclcpp::ok()) {
      if (std::filesystem::exists(storage_path)) {
        return;
      }
      std::this_thread::sleep_for(50ms);  // wait a bit to not query constantly
    }
    ASSERT_EQ(std::filesystem::exists(storage_path), true)
      << "Could not find storage file: \"" << storage_path.generic_string() << "\"";
  }

  template<typename MessageT>
  std::vector<std::shared_ptr<MessageT>> get_messages_for_topic(const std::string & topic)
  {
    auto filter = rosbag2_storage::StorageFilter{};
    filter.topics.push_back(topic);

    std::unique_ptr<rosbag2_cpp::Reader> reader = std::make_unique<rosbag2_cpp::Reader>();
    reader->open(root_bag_path_.generic_string());
    reader->set_filter(filter);

    auto messages = std::vector<std::shared_ptr<MessageT>>{};
    while (reader->has_next()) {
      auto msg = reader->read_next();
      messages.push_back(memory_management_.deserialize_message<MessageT>(msg->serialized_data));
    }
    return messages;
  }

protected:
  MemoryManagement memory_management_{};
};

class ComposableRecorderTests
  : public rosbag2_test_common::ParametrizedTemporaryDirectoryFixture
{
public:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    auto bag_name = get_test_name() + "_" + GetParam();
    root_bag_path_ = std::filesystem::path(temporary_dir_path_) / bag_name;

    // Clean up potentially leftover bag files.
    // There may be leftovers if the system reallocates a temp directory
    // used by a previous test execution and the test did not have a clean exit.
    std::filesystem::remove_all(root_bag_path_);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
    std::filesystem::remove_all(root_bag_path_);
  }

  std::string get_test_name() const
  {
    const auto * test_info = UnitTest::GetInstance()->current_test_info();
    std::string test_name = test_info->name();
    // Replace any slashes in the test name, since it is used in paths
    std::replace(test_name.begin(), test_name.end(), '/', '_');
    return test_name;
  }

  std::filesystem::path root_bag_path_;
};

class MockComposableRecorder : public rosbag2_transport::Recorder
{
public:
  static const char demo_attribute_name_[];
  bool demo_attribute_value{false};

  explicit MockComposableRecorder(
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
const char MockComposableRecorder::demo_attribute_name_[] = "demo_attribute";

TEST_P(ComposableRecorderTests, recorder_inner_params_passed_as_append_override)
{
  std::vector<rclcpp::Parameter> parameters;
  parameters.emplace_back(MockComposableRecorder::demo_attribute_name_, true);
  parameters.emplace_back("storage.uri", rclcpp::ParameterValue(root_bag_path_.generic_string()));
  auto options = rclcpp::NodeOptions()
    .use_global_arguments(false)
    .parameter_overrides(parameters);

  auto recorder = std::make_shared<MockComposableRecorder>(options);
  // Check that rosbag2_transport::Recorder inner params will not erase our
  // parameter_overrides options
  ASSERT_TRUE(recorder->get_value_of_bool_parameter(recorder->demo_attribute_name_));
  ASSERT_TRUE(recorder->demo_attribute_value);
}

TEST_P(ComposableRecorderTests, recorder_can_parse_parameters_from_file) {
  // _SRC_RESOURCES_DIR_PATH defined in CMakeLists.txt
  rclcpp::NodeOptions opts;
  opts.arguments(
  {
    "--ros-args",
    "--params-file", _SRC_RESOURCES_DIR_PATH "/recorder_node_params.yaml"
  });
  opts.append_parameter_override(
    "record.qos_profile_overrides_path",
    _SRC_RESOURCES_DIR_PATH "/qos_profile_overrides.yaml");
  opts.append_parameter_override("storage.uri", root_bag_path_.generic_string());
  opts.append_parameter_override("storage.storage_id", GetParam());

  auto recorder = std::make_shared<MockComposableRecorder>(opts, "recorder_params_node");
  auto record_options = recorder->get_record_options();
  auto storage_options = recorder->get_storage_options();

  EXPECT_EQ(record_options.all_topics, true);
  EXPECT_EQ(record_options.all_services, true);
  EXPECT_EQ(record_options.is_discovery_disabled, true);
  std::vector<std::string> topics {"/topic", "/other_topic"};
  EXPECT_EQ(record_options.topics, topics);
  std::vector<std::string> topic_types {"std_msgs/msg/Header", "geometry_msgs/msg/Pose"};
  EXPECT_EQ(record_options.topic_types, topic_types);
  std::vector<std::string> exclude_topic_types {"sensor_msgs/msg/Image"};
  EXPECT_EQ(record_options.exclude_topic_types, exclude_topic_types);
  std::vector<std::string> services {"/service/_service_event", "/other_service/_service_event"};
  EXPECT_EQ(record_options.services, services);
  EXPECT_EQ(record_options.rmw_serialization_format, "cdr");
  EXPECT_TRUE(record_options.topic_polling_interval == 0.01s);
  EXPECT_EQ(record_options.regex, "[xyz]/topic");
  EXPECT_EQ(record_options.exclude_regex, "(.*)");
  std::vector<std::string> exclude_topics {"/exclude_topic", "/other_exclude_topic"};
  EXPECT_EQ(record_options.exclude_topics, exclude_topics);
  std::vector<std::string> exclude_services {
    "/exclude_service/_service_event", "/other_exclude_service/_service_event"};
  EXPECT_EQ(record_options.exclude_service_events, exclude_services);
  EXPECT_EQ(record_options.node_prefix, "prefix");
  EXPECT_EQ(record_options.compression_mode, "stream");
  EXPECT_EQ(record_options.compression_format, "h264");
  EXPECT_EQ(record_options.compression_queue_size, 10);
  EXPECT_EQ(record_options.compression_threads, 2);
  EXPECT_EQ(record_options.compression_threads_priority, -1);
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

  EXPECT_EQ(storage_options.uri, root_bag_path_.generic_string());
  EXPECT_EQ(storage_options.storage_id, GetParam());
  EXPECT_EQ(storage_options.storage_config_uri, "");
  EXPECT_EQ(storage_options.max_bagfile_size, 2147483646);
  EXPECT_EQ(storage_options.max_bagfile_duration, 2147483646);
  EXPECT_EQ(storage_options.max_cache_size, 989888);
  EXPECT_EQ(storage_options.storage_preset_profile, "none");
  EXPECT_EQ(storage_options.snapshot_mode, false);
  std::unordered_map<std::string, std::string> custom_data{
    std::pair{"key1", "value1"},
    std::pair{"key2", "value2"}
  };
  EXPECT_EQ(storage_options.custom_data, custom_data);
  EXPECT_EQ(storage_options.start_time_ns, 0);
  EXPECT_EQ(storage_options.end_time_ns, 100000);
}

TEST_P(
  ComposableRecorderIntegrationTests,
  recorder_can_automatically_start_recording_after_composition) {
  const size_t num_messages_to_publish = 5;
  auto string_message = get_messages_strings()[0];
  string_message->string_value = "Hello World";
  const std::string test_topic_name = "/composable_recorder_test_string_topic";

  rosbag2_test_common::PublicationManager pub_manager;
  pub_manager.setup_publisher(test_topic_name, string_message, num_messages_to_publish);

  // Load composable recorder node
  auto load_node_request = std::make_shared<composition_interfaces::srv::LoadNode::Request>();
  load_node_request->package_name = "rosbag2_transport";
  load_node_request->plugin_name = "rosbag2_transport::Recorder";

  rclcpp::Parameter uri("storage.uri", rclcpp::ParameterValue(root_bag_path_.generic_string()));
  rclcpp::Parameter storage_id("storage.storage_id", GetParam());
  rclcpp::Parameter disable_all("record.all", false);
  rclcpp::Parameter topics("record.topics", std::vector{test_topic_name});

  load_node_request->parameters.push_back(uri.to_parameter_msg());
  load_node_request->parameters.push_back(storage_id.to_parameter_msg());
  load_node_request->parameters.push_back(disable_all.to_parameter_msg());
  load_node_request->parameters.push_back(topics.to_parameter_msg());

  auto load_node_future = load_node_client_->async_send_request(load_node_request);
  // Wait for the response
  auto load_node_ret = exec_->spin_until_future_complete(load_node_future, 10s);
  ASSERT_EQ(load_node_ret, rclcpp::FutureReturnCode::SUCCESS);
  auto load_node_response = load_node_future.get();
  EXPECT_EQ(load_node_response->success, true);
  EXPECT_EQ(load_node_response->error_message, "");
  EXPECT_EQ(load_node_response->full_node_name, "/rosbag2_recorder");

  ASSERT_TRUE(pub_manager.wait_for_matched(test_topic_name.c_str())) <<
    "Expected to find " << test_topic_name.c_str() << " subscription";

  wait_for_storage_file();

  pub_manager.run_publishers();

  // Unload composed recorder node
  unload_node(load_node_response->unique_id);

  wait_for_metadata();
  auto test_topic_messages = get_messages_for_topic<test_msgs::msg::Strings>(test_topic_name);
  // We shutdown node right after publishing messages. We can't guarantee that all published
  // messages will be delivered and recorded. Some messages could be still in the DDS or pub/sub
  // queues. Make sure that we recorded at least one.
  EXPECT_THAT(test_topic_messages, SizeIs(Ge(1)));
}

INSTANTIATE_TEST_SUITE_P(
  ParametrizedComposableRecorderIntegrationTests,
  ComposableRecorderIntegrationTests,
  ValuesIn(rosbag2_test_common::kTestedStorageIDs)
);

INSTANTIATE_TEST_SUITE_P(
  ParametrizedComposableRecorderTests,
  ComposableRecorderTests,
  ValuesIn(rosbag2_test_common::kTestedStorageIDs)
);
