// Copyright 2018, Bosch Software Innovations GmbH.
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
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include <rosbag2_storage/ros_helper.hpp>

#include "mock_recorder.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcpputils/scope_exit.hpp"

#include "rosbag2_test_common/publication_manager.hpp"
#include "rosbag2_test_common/wait_for.hpp"
#include "rosbag2_test_common/temporary_directory_fixture.hpp"

#include "rosbag2_transport/recorder.hpp"

#include "test_msgs/msg/arrays.hpp"
#include "test_msgs/msg/basic_types.hpp"
#include "test_msgs/message_fixtures.hpp"

#include "rosbag2_storage/qos.hpp"
#include "record_integration_fixture.hpp"
#include "rosbag2_transport/reader_writer_factory.hpp"

using namespace ::testing;  // NOLINT
using rosbag2_test_common::TemporaryDirectoryFixture;
namespace fs = std::filesystem;

TEST_F(RecordIntegrationTestFixture, published_messages_from_multiple_topics_are_recorded)
{
  auto array_message = get_messages_arrays()[0];
  std::string array_topic = "/array_topic";

  auto string_message = get_messages_strings()[1];
  std::string string_topic = "/string_topic";

  rosbag2_test_common::PublicationManager pub_manager;
  pub_manager.setup_publisher(array_topic, array_message, 2);

  rosbag2_transport::RecordOptions record_options =
  {false, false, false, false, {string_topic, array_topic},
    {}, {}, {}, {}, {}, {}, {}, {}, "rmw_format", "rmw_format", 50ms};
  auto recorder = std::make_shared<rosbag2_transport::Recorder>(
    std::move(writer_), storage_options_, record_options);
  recorder->record();

  // Note: Intentionally setup one publisher after starting recorder to test recorder's ability
  // to dynamically discover topics in runtime.
  pub_manager.setup_publisher(string_topic, string_message, 2);

  constexpr size_t expected_messages = 4;
  std::vector<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>> recorded_messages;
  std::unordered_map<
    std::string,
    std::pair<rosbag2_storage::TopicMetadata, rosbag2_storage::MessageDefinition>
  > recorded_topics;

  start_async_spin(recorder);
  {
    auto cleanup_process_handle = rcpputils::make_scope_exit([&]() {stop_spinning();});

    ASSERT_TRUE(pub_manager.wait_for_matched(array_topic.c_str()));
    ASSERT_TRUE(pub_manager.wait_for_matched(string_topic.c_str()));

    pub_manager.run_publishers();

    auto & writer = recorder->get_writer_handle();
    MockSequentialWriter & mock_writer =
      static_cast<MockSequentialWriter &>(writer.get_implementation_handle());

    auto ret = rosbag2_test_common::wait_until_condition(
      [ =, &mock_writer]() {
        return mock_writer.get_number_of_recorded_messages() >= expected_messages;
      },
      std::chrono::seconds(5));
    EXPECT_TRUE(ret) << "failed to capture expected messages in time" <<
      "recorded messages = " << recorded_messages.size();
    recorded_messages = mock_writer.get_messages();
    recorded_topics = mock_writer.get_topics();
  }

  EXPECT_THAT(recorded_messages, SizeIs(expected_messages));

  ASSERT_THAT(recorded_topics, SizeIs(2));
  EXPECT_THAT(recorded_topics.at(string_topic).first.serialization_format, Eq("rmw_format"));
  EXPECT_THAT(recorded_topics.at(array_topic).first.serialization_format, Eq("rmw_format"));
  ASSERT_THAT(recorded_messages, SizeIs(4));
  auto string_messages = filter_messages<test_msgs::msg::Strings>(
    recorded_messages, string_topic);
  auto array_messages = filter_messages<test_msgs::msg::Arrays>(
    recorded_messages, array_topic);
  ASSERT_THAT(string_messages, SizeIs(2));
  ASSERT_THAT(array_messages, SizeIs(2));
  EXPECT_THAT(string_messages[0]->string_value, Eq(string_message->string_value));
  EXPECT_THAT(array_messages[0]->bool_values, Eq(array_message->bool_values));
  EXPECT_THAT(array_messages[0]->float32_values, Eq(array_message->float32_values));

  // Check for send and received timestamps
  bool rmw_has_send_timestamp_support = true;
#ifdef _WIN32
  if (std::string(rmw_get_implementation_identifier()).find("rmw_connextdds") !=
    std::string::npos)
  {
    rmw_has_send_timestamp_support = false;
  }
#endif
  for (const auto & message : recorded_messages) {
    EXPECT_NE(message->recv_timestamp, 0) << "topic : " << message->topic_name;
    if (rmw_has_send_timestamp_support) {
      // Check that the send_timestamp is not the same as the clock message
      EXPECT_NE(message->send_timestamp, 0);
      EXPECT_THAT(message->recv_timestamp, Ge(message->send_timestamp));
    } else {
      // if rwm has not sent timestamp support, send_timestamp must be zero
      EXPECT_EQ(message->send_timestamp, 0);
    }
  }
}

TEST_F(TemporaryDirectoryFixture, can_record_again_after_stop_with_real_storage) {
  rclcpp::init(0, nullptr);
  auto cleanup_process_handle = rcpputils::make_scope_exit([&]() {rclcpp::shutdown();});
  std::string test_topic = "/can_record_again_after_stop_topic";
  rosbag2_storage::StorageOptions storage_options{};
  storage_options.uri = (fs::path(temporary_dir_path_) / "start_stop_again").generic_string();

  rosbag2_transport::RecordOptions record_options{};

  auto writer = rosbag2_transport::ReaderWriterFactory::make_writer(record_options);
  {
    auto recorder = std::make_shared<MockRecorder>(
      std::move(writer), storage_options, record_options);

    EXPECT_NO_THROW(recorder->record());
    fs::path storage_path(storage_options.uri);
    EXPECT_TRUE(fs::is_directory(storage_path));

    EXPECT_NO_THROW(recorder->stop());
    EXPECT_NO_THROW(recorder->record());
    storage_path = recorder->get_storage_options().uri;
    EXPECT_TRUE(fs::is_directory(storage_path));
    std::string expected_path_str = storage_options.uri + "(1)";
    EXPECT_EQ(storage_path.generic_string(), expected_path_str);
  }
}

TEST_F(RecordIntegrationTestFixture, can_record_again_after_stop)
{
  auto basic_type_message = get_messages_basic_types()[0];
  basic_type_message->uint64_value = 5;
  basic_type_message->int64_value = -1;
  std::string test_topic = "/can_record_again_after_stop_topic";
  const size_t num_messages_to_publish = 2;

  rosbag2_test_common::PublicationManager pub_manager;
  pub_manager.setup_publisher(
    test_topic, basic_type_message, num_messages_to_publish, rclcpp::QoS{rclcpp::KeepAll()}, 50ms);

  rosbag2_transport::RecordOptions record_options =
  {
    false, false, false, false, {test_topic}, {}, {}, {}, {}, {}, {}, {}, {}, "rmw_format",
    "rmw_format", 50ms
  };
  auto recorder = std::make_shared<rosbag2_transport::Recorder>(
    std::move(writer_), storage_options_, record_options);
  recorder->record();

  start_async_spin(recorder);
  auto cleanup_process_handle = rcpputils::make_scope_exit([&]() {stop_spinning();});

  auto & writer = recorder->get_writer_handle();
  auto & mock_writer = dynamic_cast<MockSequentialWriter &>(writer.get_implementation_handle());

  ASSERT_TRUE(pub_manager.wait_for_matched(test_topic.c_str()));

  pub_manager.run_publishers();

  EXPECT_FALSE(mock_writer.closed_was_called());
  recorder->stop();
  EXPECT_TRUE(mock_writer.closed_was_called());

  // Record one more time after stop()
  recorder->record();

  ASSERT_TRUE(pub_manager.wait_for_matched(test_topic.c_str()));
  pub_manager.run_publishers();

  // 4 because we're running recorder->record() and publishers twice
  constexpr size_t expected_messages = 4;
  auto ret = rosbag2_test_common::wait_until_condition(
    [ =, &mock_writer]() {
      return mock_writer.get_number_of_recorded_messages() >= expected_messages;
    },
    std::chrono::seconds(5));
  auto recorded_messages = mock_writer.get_messages();
  EXPECT_TRUE(ret) << "failed to capture expected messages in time";
  EXPECT_THAT(recorded_messages, SizeIs(expected_messages));

  // Output debug info if test fails
  if (recorded_messages.size() != expected_messages) {
    for (size_t i = 0; i < recorded_messages.size(); i++) {
      std::cerr << "=> recorded_messages[" << i << "].send_timestamp = " <<
        recorded_messages[i]->send_timestamp << std::endl;
      std::cerr << "recorded_messages[" << i << "].recv_timestamp = " <<
        recorded_messages[i]->recv_timestamp << std::endl;
    }
  }

  auto recorded_topics = mock_writer.get_topics();
  EXPECT_THAT(recorded_topics, SizeIs(1)) << "size=" << recorded_topics.size();
  if (recorded_topics.size() != 1) {
    for (const auto & topic : recorded_topics) {
      std::cerr << "recorded topic name : " << topic.first << std::endl;
    }
  }
  EXPECT_THAT(recorded_topics.at(test_topic).first.serialization_format, Eq("rmw_format"));

  auto basic_type_messages =
    filter_messages<test_msgs::msg::BasicTypes>(recorded_messages, test_topic);
  EXPECT_THAT(basic_type_messages, SizeIs(expected_messages));

  // Output debug info if test fails
  if (basic_type_messages.size() != expected_messages) {
    for (size_t i = 0; i < basic_type_messages.size(); i++) {
      std::cerr << "=> basic_type_messages[" << i << "].uint64_value = " <<
        basic_type_messages[i]->uint64_value << std::endl;
      std::cerr << "basic_type_messages[" << i << "].int64_value = " <<
        basic_type_messages[i]->int64_value << std::endl;
    }
  }

  for (size_t i = 0; i < basic_type_messages.size(); i++) {
    const uint64_t expected_u64_value = i % num_messages_to_publish;
    EXPECT_THAT(basic_type_messages[i]->uint64_value, Eq(expected_u64_value));
  }
}

TEST_F(RecordIntegrationTestFixture, qos_is_stored_in_metadata)
{
  auto string_message = get_messages_strings()[1];
  std::string topic = "/qos_chatter";

  rosbag2_test_common::PublicationManager pub_manager;
  pub_manager.setup_publisher(topic, string_message, 2);

  rosbag2_transport::RecordOptions record_options =
  {false, false, false, false, {topic}, {}, {}, {}, {}, {}, {}, {}, {}, {}, "rmw_format", 100ms};
  auto recorder = std::make_shared<rosbag2_transport::Recorder>(
    std::move(writer_), storage_options_, record_options);
  recorder->record();

  start_async_spin(recorder);
  auto cleanup_process_handle = rcpputils::make_scope_exit([&]() {stop_spinning();});

  ASSERT_TRUE(pub_manager.wait_for_matched(topic.c_str()));

  pub_manager.run_publishers();

  auto & writer = recorder->get_writer_handle();
  MockSequentialWriter & mock_writer =
    static_cast<MockSequentialWriter &>(writer.get_implementation_handle());

  constexpr size_t expected_messages = 2;
  auto ret = rosbag2_test_common::wait_until_condition(
    [ =, &mock_writer]() {
      return mock_writer.get_number_of_recorded_messages() >= expected_messages;
    },
    std::chrono::seconds(5));
  auto recorded_messages = mock_writer.get_messages();
  EXPECT_TRUE(ret) << "failed to capture expected messages in time";
  EXPECT_THAT(recorded_messages, SizeIs(expected_messages));

  auto recorded_topics = mock_writer.get_topics();
  auto offered_qos_profiles = recorded_topics.at(topic).first.offered_qos_profiles;
  std::string serialized_profiles = rosbag2_storage::serialize_rclcpp_qos_vector(
    offered_qos_profiles);
  // Basic smoke test that the profile was serialized into the metadata as a string.
  EXPECT_THAT(serialized_profiles, ContainsRegex("reliability: reliable\n"));
  EXPECT_THAT(serialized_profiles, ContainsRegex("durability: volatile\n"));
  EXPECT_THAT(
    serialized_profiles, ContainsRegex(
      "deadline:\n"
      "    sec: .+\n"
      "    nsec: .+\n"
  ));
  EXPECT_THAT(
    serialized_profiles, ContainsRegex(
      "lifespan:\n"
      "    sec: .+\n"
      "    nsec: .+\n"
  ));
  EXPECT_THAT(serialized_profiles, ContainsRegex("liveliness: automatic\n"));
  EXPECT_THAT(
    serialized_profiles, ContainsRegex(
      "liveliness_lease_duration:\n"
      "    sec: .+\n"
      "    nsec: .+\n"
  ));
  EXPECT_EQ(recorded_topics.at(topic).second.topic_type, "test_msgs/msg/Strings");
  EXPECT_EQ(recorded_topics.at(topic).second.encoding, "");
}

TEST_F(RecordIntegrationTestFixture, records_sensor_data)
{
  auto string_message = get_messages_strings()[1];
  std::string topic = "/sensor_chatter";

  rosbag2_test_common::PublicationManager pub_manager;
  pub_manager.setup_publisher(topic, string_message, 2, rclcpp::SensorDataQoS());

  rosbag2_transport::RecordOptions record_options =
  {false, false, false, false, {topic}, {}, {}, {}, {}, {}, {}, {}, {}, {}, "rmw_format", 100ms};
  auto recorder = std::make_shared<rosbag2_transport::Recorder>(
    std::move(writer_), storage_options_, record_options);
  recorder->record();

  start_async_spin(recorder);
  auto cleanup_process_handle = rcpputils::make_scope_exit([&]() {stop_spinning();});

  ASSERT_TRUE(pub_manager.wait_for_matched(topic.c_str()));

  pub_manager.run_publishers();

  auto & writer = recorder->get_writer_handle();
  MockSequentialWriter & mock_writer =
    static_cast<MockSequentialWriter &>(writer.get_implementation_handle());

  constexpr size_t expected_messages = 2;
  auto ret = rosbag2_test_common::wait_until_condition(
    [ =, &mock_writer]() {
      return mock_writer.get_number_of_recorded_messages() >= expected_messages;
    },
    std::chrono::seconds(5));
  auto recorded_messages = mock_writer.get_messages();
  auto recorded_topics = mock_writer.get_topics();
  EXPECT_TRUE(ret) << "failed to capture expected messages in time";
  EXPECT_THAT(recorded_messages, SizeIs(expected_messages));
  EXPECT_EQ(recorded_topics.size(), 1u);
  EXPECT_FALSE(recorded_messages.empty());
}

TEST_F(RecordIntegrationTestFixture, receives_latched_messages)
{
  auto string_message = get_messages_strings()[1];
  std::string topic = "/latched_chatter";

  size_t num_latched_messages = 3;
  auto profile_transient_local = rclcpp::QoS(num_latched_messages).transient_local();
  rosbag2_test_common::PublicationManager pub_manager;
  pub_manager.setup_publisher(
    topic, string_message, num_latched_messages, profile_transient_local);
  // Publish messages before starting recording
  pub_manager.run_publishers();

  rosbag2_transport::RecordOptions record_options =
  {false, false, false, false, {topic}, {}, {}, {}, {}, {}, {}, {}, {}, {}, "rmw_format", 100ms};
  auto recorder = std::make_shared<rosbag2_transport::Recorder>(
    std::move(writer_), storage_options_, record_options);
  recorder->record();

  start_async_spin(recorder);
  auto cleanup_process_handle = rcpputils::make_scope_exit([&]() {stop_spinning();});

  ASSERT_TRUE(pub_manager.wait_for_matched(topic.c_str()));

  auto & writer = recorder->get_writer_handle();
  MockSequentialWriter & mock_writer =
    static_cast<MockSequentialWriter &>(writer.get_implementation_handle());

  size_t expected_messages = num_latched_messages;
  auto ret = rosbag2_test_common::wait_until_condition(
    [&mock_writer, &expected_messages]() {
      return mock_writer.get_number_of_recorded_messages() >= expected_messages;
    },
    std::chrono::seconds(5));
  auto recorded_messages = mock_writer.get_messages();
  auto recorded_topics = mock_writer.get_topics();
  EXPECT_TRUE(ret) << "failed to capture expected messages in time";
  EXPECT_THAT(recorded_messages, SizeIs(expected_messages));
  EXPECT_EQ(recorded_topics.size(), 1u);
  EXPECT_FALSE(recorded_messages.empty());
}

TEST_F(RecordIntegrationTestFixture, mixed_qos_subscribes) {
  // Ensure that rosbag2 subscribes to publishers that offer different durability policies
  const size_t arbitrary_history = 5;

  std::string topic = "/string_topic";
  test_msgs::msg::Strings msg;
  msg.string_value = "Hello";

  auto profile_volatile = rclcpp::QoS(arbitrary_history).reliable().durability_volatile();
  auto profile_transient_local = rclcpp::QoS(arbitrary_history).reliable().transient_local();

  auto publisher_node = std::make_shared<rclcpp::Node>("rosbag2_test_record_5");
  auto publisher_volatile = publisher_node->create_publisher<test_msgs::msg::Strings>(
    topic, profile_volatile);
  auto publisher_transient_local = publisher_node->create_publisher<test_msgs::msg::Strings>(
    topic, profile_transient_local);

  rosbag2_transport::RecordOptions record_options =
  {false, false, false, false, {topic}, {}, {}, {}, {}, {}, {}, {}, {}, {}, "rmw_format", 100ms};
  auto recorder = std::make_shared<rosbag2_transport::Recorder>(
    std::move(writer_), storage_options_, record_options);
  recorder->record();

  // Takes ~100ms in local testing, 5s chosen as a very long timeout
  bool succeeded = rosbag2_test_common::spin_and_wait_for(
    std::chrono::seconds(5), recorder,
    [publisher_volatile, publisher_transient_local]() {
      // This test is a success if rosbag2 has connected to both publishers
      // *INDENT-OFF*
      return
        publisher_volatile->get_subscription_count() &&
        publisher_transient_local->get_subscription_count();
      // *INDENT-ON*
    });
  ASSERT_TRUE(succeeded);
}

TEST_F(RecordIntegrationTestFixture, duration_and_noncompatibility_policies_mixed) {
  // Ensure that the duration-based and non-compatibility QoS policies don't affect subscription
  // These values are arbitrary, the significance is that they are non-default
  const std::string topic = "/mixed_nondelivery_policies";
  const size_t same_history = 5;
  const size_t different_history = 12;
  const rmw_time_t deadline{0, 1000};
  const rmw_time_t lifespan{3, 12};
  const auto liveliness = RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC;
  const rmw_time_t liveliness_lease_duration{0, 5000000};

  auto publisher_node = std::make_shared<rclcpp::Node>("rosbag2_test_record_6");
  auto create_pub = [publisher_node, topic](auto qos) {
      return publisher_node->create_publisher<test_msgs::msg::Strings>(topic, qos);
    };

  auto profile_history = rclcpp::QoS(different_history);
  auto publisher_history = create_pub(profile_history);

  auto profile_lifespan = rclcpp::QoS(same_history).lifespan(lifespan);
  auto publisher_lifespan = create_pub(profile_lifespan);

  auto profile_deadline = rclcpp::QoS(same_history).deadline(deadline);
  auto publisher_deadline = create_pub(profile_deadline);

  auto profile_liveliness = rclcpp::QoS(same_history)
    .liveliness(liveliness).liveliness_lease_duration(liveliness_lease_duration);
  auto publisher_liveliness = create_pub(profile_liveliness);

  rosbag2_transport::RecordOptions record_options =
  {false, false, false, false, {topic}, {}, {}, {}, {}, {}, {}, {}, {}, {}, "rmw_format", 100ms};
  auto recorder = std::make_shared<rosbag2_transport::Recorder>(
    std::move(writer_), storage_options_, record_options);
  recorder->record();

  // Takes ~200ms in local testing, 5s chosen as a very long timeout
  bool succeeded = rosbag2_test_common::spin_and_wait_for(
    std::chrono::seconds(5), recorder,
    [publisher_history, publisher_lifespan, publisher_deadline, publisher_liveliness]() {
      // *INDENT-OFF*
      return
        publisher_history->get_subscription_count() &&
        publisher_lifespan->get_subscription_count() &&
        publisher_deadline->get_subscription_count() &&
        publisher_liveliness->get_subscription_count();
      // *INDENT-ON*
    });
  ASSERT_TRUE(succeeded);
}

TEST_F(RecordIntegrationTestFixture, write_split_callback_is_called)
{
  auto string_message = get_messages_strings()[1];
  std::string string_topic = "/string_topic";

  bool callback_called = false;
  std::string closed_file, opened_file;
  rosbag2_cpp::bag_events::WriterEventCallbacks callbacks;
  callbacks.write_split_callback =
    [&callback_called, &closed_file, &opened_file](rosbag2_cpp::bag_events::BagSplitInfo & info) {
      closed_file = info.closed_file;
      opened_file = info.opened_file;
      callback_called = true;
    };
  writer_->add_event_callbacks(callbacks);

  {
    auto & mock_writer = dynamic_cast<MockSequentialWriter &>(writer_->get_implementation_handle());
    mock_writer.set_max_messages_per_file(5);
  }

  rosbag2_transport::RecordOptions record_options =
  {
    false, false, false, false, {string_topic}, {}, {}, {}, {}, {}, {}, {}, {}, {}, "rmw_format",
    10ms
  };
  auto recorder = std::make_shared<rosbag2_transport::Recorder>(
    std::move(writer_), storage_options_, record_options);

  start_async_spin(recorder);
  auto cleanup_process_handle = rcpputils::make_scope_exit([&]() {stop_spinning();});

  auto & writer = recorder->get_writer_handle();
  auto & mock_writer = dynamic_cast<MockSequentialWriter &>(writer.get_implementation_handle());

  size_t expected_messages = mock_writer.max_messages_per_file() + 1;

  rosbag2_test_common::PublicationManager pub_manager;
  pub_manager.setup_publisher(string_topic, string_message, expected_messages);

  recorder->record();

  ASSERT_TRUE(pub_manager.wait_for_matched(string_topic.c_str()));
  pub_manager.run_publishers();

  auto ret = rosbag2_test_common::wait_until_condition(
    [&mock_writer, &expected_messages]() {
      return mock_writer.get_number_of_recorded_messages() >= expected_messages;
    },
    std::chrono::seconds(5));
  auto recorded_messages = mock_writer.get_messages();
  EXPECT_TRUE(ret) << "failed to capture expected messages in time";
  EXPECT_THAT(recorded_messages, SizeIs(expected_messages));

  // Confirm that the callback was called and the file names have been sent with the event
  ASSERT_TRUE(callback_called);
  EXPECT_EQ(closed_file, "BagFile0");
  EXPECT_EQ(opened_file, "BagFile1");
}

TEST_F(RecordIntegrationTestFixture, toggle_paused_do_pause_resume)
{
  rosbag2_transport::RecordOptions record_options{};
  auto recorder = std::make_shared<rosbag2_transport::Recorder>(
    std::move(writer_), storage_options_, record_options);
  recorder->pause();
  ASSERT_TRUE(recorder->is_paused());

  testing::internal::CaptureStderr();
  recorder->toggle_paused();
  std::string test_output = testing::internal::GetCapturedStderr();
  EXPECT_FALSE(recorder->is_paused());
  EXPECT_TRUE(test_output.find("Resuming recording.") != std::string::npos);

  testing::internal::CaptureStderr();
  recorder->toggle_paused();
  test_output = testing::internal::GetCapturedStderr();
  EXPECT_TRUE(recorder->is_paused());
  EXPECT_TRUE(test_output.find("Pausing recording.") != std::string::npos);
}

TEST_F(RecordIntegrationTestFixture, add_channel_creates_topic_in_writer)
{
  rosbag2_transport::RecordOptions record_options{};
  record_options.is_discovery_disabled = true;
  auto recorder = std::make_shared<rosbag2_transport::Recorder>(
    std::move(writer_), storage_options_, record_options);

  std::string topic_name = "/test_channel";
  std::string topic_type = "rosbag2_test_msgdefs/ComplexMsg";
  std::string serialization_format = "cdr";

  recorder->record();

  recorder->add_channel(topic_name, topic_type, serialization_format);

  EXPECT_THAT(recorder->subscriptions().size(), 0u);

  auto & writer = recorder->get_writer_handle();
  auto & mock_writer = dynamic_cast<MockSequentialWriter &>(writer.get_implementation_handle());
  auto recorded_topics = mock_writer.get_topics();

  EXPECT_EQ(recorded_topics.count(topic_name), 1u);
  EXPECT_EQ(recorded_topics.at(topic_name).first.name, topic_name);
  EXPECT_EQ(recorded_topics.at(topic_name).first.type, topic_type);
  EXPECT_EQ(recorded_topics.at(topic_name).first.serialization_format, serialization_format);

  recorder->stop();
}

TEST_F(RecordIntegrationTestFixture, add_channel_with_message_definition)
{
  rosbag2_transport::RecordOptions record_options{};
  record_options.is_discovery_disabled = true;
  auto recorder = std::make_shared<rosbag2_transport::Recorder>(
    std::move(writer_), storage_options_, record_options);

  std::string topic_name = "/test_channel_with_def";
  std::string topic_type = "test_msgs/msg/BasicTypes";
  std::string message_definition_encoding = "ros2idl";
  std::string encoded_message_definition = "string test_field";
  std::string serialization_format = "cdr";

  recorder->record();

  recorder->add_channel(topic_name, topic_type, message_definition_encoding,
                        encoded_message_definition, serialization_format);

  auto & writer = recorder->get_writer_handle();
  auto & mock_writer = dynamic_cast<MockSequentialWriter &>(writer.get_implementation_handle());
  auto recorded_topics = mock_writer.get_topics();

  EXPECT_EQ(recorded_topics.count(topic_name), 1u);
  EXPECT_EQ(recorded_topics.at(topic_name).first.name, topic_name);
  EXPECT_EQ(recorded_topics.at(topic_name).first.type, topic_type);
  EXPECT_EQ(recorded_topics.at(topic_name).second.encoding, message_definition_encoding);
  EXPECT_EQ(
    recorded_topics.at(topic_name).second.encoded_message_definition,
    encoded_message_definition
  );
  recorder->stop();
}

TEST_F(RecordIntegrationTestFixture, write_message_writes_to_bag)
{
  rosbag2_transport::RecordOptions record_options{};
  const std::string serialization_format = "memory_view";
  record_options.input_serialization_format = serialization_format;
  record_options.output_serialization_format = serialization_format;
  record_options.is_discovery_disabled = true;

  auto recorder = std::make_shared<rosbag2_transport::Recorder>(
    std::move(writer_), storage_options_, record_options);

  const std::string topic_name = "/direct_write_topic";
  const std::string topic_type = "test_msgs/msg/Strings";

  // Add channel first
  recorder->add_channel(topic_name, topic_type, serialization_format);

  recorder->record();

  const uint32_t num_messages = 5U;
  const rcutils_time_point_value_t initial_timestamp = 1234567890;

  for (uint32_t i = 0; i < num_messages; i++) {
    // Create serialized message
    uint64_t msg_content_uint64_value = i * 1000;
    auto serialized_data =
      rosbag2_storage::make_serialized_message(&msg_content_uint64_value,
                                               sizeof(msg_content_uint64_value));

    rcutils_time_point_value_t timestamp = initial_timestamp + (i * 200);
    uint32_t sequence_number = i;
    recorder->write_message(serialized_data, topic_name, timestamp, timestamp + 1, sequence_number);
  }

  auto & writer = recorder->get_writer_handle();
  auto & mock_writer = dynamic_cast<MockSequentialWriter &>(writer.get_implementation_handle());

  ASSERT_EQ(mock_writer.get_number_of_recorded_messages(), num_messages);

  auto recorded_messages = mock_writer.get_messages();
  ASSERT_EQ(recorded_messages.size(), num_messages);

  for (uint32_t i = 0; i < num_messages; i++) {
    EXPECT_EQ(recorded_messages[i]->topic_name, topic_name);
    EXPECT_EQ(recorded_messages[i]->send_timestamp, initial_timestamp + (i * 200));
    EXPECT_EQ(recorded_messages[i]->recv_timestamp, initial_timestamp + (i * 200) + 1);
    EXPECT_EQ(recorded_messages[i]->sequence_number, i);

    uint64_t recorded_message_value =
      *(reinterpret_cast<uint64_t *>(recorded_messages[i]->serialized_data->buffer));
    EXPECT_EQ(recorded_message_value, i * 1000);
  }

  recorder->stop();
}

TEST_F(RecordIntegrationTestFixture, write_message_uses_recv_timestamp_from_node_clock)
{
  rosbag2_transport::RecordOptions record_options{};
  const std::string serialization_format = "memory_view";
  record_options.input_serialization_format = serialization_format;
  record_options.output_serialization_format = serialization_format;
  record_options.is_discovery_disabled = true;

  auto recorder = std::make_shared<rosbag2_transport::Recorder>(
    std::move(writer_), storage_options_, record_options);

  const std::string topic_name = "/direct_write_topic";
  const std::string topic_type = "test_msgs/msg/Strings";

  // Add channel first
  recorder->add_channel(topic_name, topic_type, serialization_format);

  recorder->record();

  const uint32_t num_messages = 5u;
  const rcutils_time_point_value_t initial_timestamp = 1234567890;

  for (uint32_t i = 0; i < num_messages; i++) {
    // Create serialized message
    uint64_t msg_content_uint64_value = i * 1000;
    auto serialized_data =
      rosbag2_storage::make_serialized_message(&msg_content_uint64_value,
                                               sizeof(msg_content_uint64_value));

    rcutils_time_point_value_t timestamp = initial_timestamp + (i * 200);
    uint32_t sequence_number = i;
    recorder->write_message(serialized_data, topic_name, timestamp, sequence_number);
  }

  auto & writer = recorder->get_writer_handle();
  auto & mock_writer = dynamic_cast<MockSequentialWriter &>(writer.get_implementation_handle());

  ASSERT_EQ(mock_writer.get_number_of_recorded_messages(), num_messages);

  auto recorded_messages = mock_writer.get_messages();
  ASSERT_EQ(recorded_messages.size(), num_messages);

  for (uint32_t i = 0; i < num_messages; i++) {
    EXPECT_EQ(recorded_messages[i]->topic_name, topic_name);
    EXPECT_EQ(recorded_messages[i]->send_timestamp, initial_timestamp + (i * 200));
    // recv_timestamp should be set by the node's clock, so it should be different from
    // send_timestamp or zero.
    EXPECT_NE(recorded_messages[i]->recv_timestamp, recorded_messages[i]->send_timestamp);
    EXPECT_NE(recorded_messages[i]->recv_timestamp, 0);
    EXPECT_EQ(recorded_messages[i]->sequence_number, i);

    uint64_t recorded_message_value =
      *(reinterpret_cast<uint64_t *>(recorded_messages[i]->serialized_data->buffer));
    EXPECT_EQ(recorded_message_value, i * 1000);
  }

  recorder->stop();
}

TEST_F(RecordIntegrationTestFixture, write_message_does_not_write_when_paused)
{
  rosbag2_transport::RecordOptions record_options{};
  const std::string serialization_format = "memory_view";
  record_options.input_serialization_format = serialization_format;
  record_options.output_serialization_format = serialization_format;
  record_options.is_discovery_disabled = true;

  auto recorder = std::make_shared<rosbag2_transport::Recorder>(
    std::move(writer_), storage_options_, record_options);

  // Create serialized message
  uint64_t msg_content_uint64_value = 12345;
  auto serialized_data = rosbag2_storage::make_serialized_message(&msg_content_uint64_value,
                                                                  sizeof(msg_content_uint64_value));

  recorder->record();

  const std::string topic_name = "/paused_write_topic";
  const std::string topic_type = "test_msgs/msg/Strings";
  recorder->add_channel(topic_name, topic_type, serialization_format, "unknown", "");

  // Pause recording
  recorder->pause();
  ASSERT_TRUE(recorder->is_paused());

  // Try to write message while paused
  rcutils_time_point_value_t timestamp = 1234567890;
  uint32_t sequence_number = 42;
  recorder->write_message(serialized_data, topic_name, timestamp, sequence_number);

  auto & writer = recorder->get_writer_handle();
  auto & mock_writer = dynamic_cast<MockSequentialWriter &>(writer.get_implementation_handle());
  auto recorded_messages = mock_writer.get_messages();

  // No messages should be recorded while paused
  EXPECT_EQ(recorded_messages.size(), 0u);

  recorder->stop();
}

TEST_F(RecordIntegrationTestFixture, on_messages_lost_in_transport_updates_statistics)
{
  rosbag2_transport::RecordOptions record_options{};
  record_options.is_discovery_disabled = true;
  auto recorder = std::make_shared<rosbag2_transport::Recorder>(
    std::move(writer_), storage_options_, record_options);

  std::string topic_name = "/lost_topic";
  std::string topic_type = "test_msgs/msg/BasicTypes";
  std::string serialization_format = "cdr";

  recorder->record();
  recorder->add_channel(topic_name, topic_type, serialization_format);

  rclcpp::QOSMessageLostInfo lost_info;
  lost_info.total_count = 5;
  lost_info.total_count_change = 3;

  recorder->on_messages_lost_in_transport(topic_name, lost_info);
  lost_info.total_count++;
  lost_info.total_count_change = 1;
  recorder->on_messages_lost_in_transport(topic_name, lost_info);

  EXPECT_EQ(recorder->get_total_num_messages_lost_in_transport(), 4u);

  recorder->stop();
}
