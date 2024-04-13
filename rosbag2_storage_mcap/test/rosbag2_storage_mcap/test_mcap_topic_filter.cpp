// Copyright 2024 Open Source Robotics Foundation, Inc.
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

#include "rosbag2_storage/storage_factory.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "rosbag2_test_common/memory_management.hpp"
#include "std_msgs/msg/string.hpp"

#include <gmock/gmock.h>

#include <filesystem>
#include <memory>
#include <string>

using namespace ::testing;  // NOLINT

namespace fs = std::filesystem;

class McapTopicFilterTestFixture : public testing::Test
{
public:
  std::shared_ptr<rosbag2_storage::storage_interfaces::ReadOnlyInterface>
  open_test_bag_for_read_only()
  {
    rosbag2_storage::StorageOptions storage_options{};
    storage_options.storage_id = storage_id_;
    storage_options.uri = (fs::path(temporary_dir_path_str_) / "rosbag.mcap").generic_string();
    return storage_factory_.open_read_only(storage_options);
  }

protected:
  // Per-test-suite set-up.
  // Called before the first test in this test suite.
  static void SetUpTestSuite()
  {
    fs::path tmp_dir_path{fs::temp_directory_path() / "tmp_mcap_test_dir_"};
    temporary_dir_path_str_ = tmp_dir_path.generic_string();
    fs::remove_all(tmp_dir_path);
    fs::create_directories(tmp_dir_path);

    std::vector<std::string> topics = {"topic1", "service_topic1/_service_event", "topic2",
                                       "service_topic2/_service_event", "topic3"};

    rosbag2_storage::TopicMetadata topic_metadata_1 = {
      1u, topics[0], "std_msgs/msg/String", "cdr", {rclcpp::QoS(1)}, "type_hash1"};
    rosbag2_storage::TopicMetadata topic_metadata_2 = {
      2u, topics[1], "std_msgs/msg/String", "cdr", {rclcpp::QoS(2)}, "type_hash2"};
    rosbag2_storage::TopicMetadata topic_metadata_3 = {
      3u, topics[2], "std_msgs/msg/String", "cdr", {rclcpp::QoS(3)}, "type_hash3"};
    rosbag2_storage::TopicMetadata topic_metadata_4 = {
      4u, topics[3], "std_msgs/msg/String", "cdr", {rclcpp::QoS(4)}, "type_hash4"};
    rosbag2_storage::TopicMetadata topic_metadata_5 = {
      5u, topics[4], "std_msgs/msg/String", "cdr", {rclcpp::QoS(5)}, "type_hash5"};

    const rosbag2_storage::MessageDefinition definition = {"std_msgs/msg/String", "ros2msg",
                                                           "string data", ""};

    std::vector<std::tuple<std::string, int64_t, rosbag2_storage::TopicMetadata,
                           rosbag2_storage::MessageDefinition>>
      string_messages = {
        std::make_tuple("topic1 message", 1, topic_metadata_1, definition),
        std::make_tuple("service event topic 1 message", 2, topic_metadata_2, definition),
        std::make_tuple("topic2 message", 3, topic_metadata_3, definition),
        std::make_tuple("service event topic 2 message", 4, topic_metadata_4, definition),
        std::make_tuple("topic3 message", 5, topic_metadata_5, definition)};

    rosbag2_storage::StorageFactory factory;
    rosbag2_storage::StorageOptions options;
    options.storage_id = storage_id_;
    options.uri = (fs::path(temporary_dir_path_str_) / "rosbag").generic_string();

    // Write test data
    {
      auto rw_writer = factory.open_read_write(options);
      write_messages_to_mcap(string_messages, rw_writer);
    }
  }

  static void write_messages_to_mcap(
    const std::vector<std::tuple<std::string, int64_t, rosbag2_storage::TopicMetadata,
                                 rosbag2_storage::MessageDefinition>> & messages,
    const std::shared_ptr<rosbag2_storage::storage_interfaces::ReadWriteInterface> & rw_storage)
  {
    auto memory_management = std::make_unique<rosbag2_test_common::MemoryManagement>();
    for (const auto & [string_message, time_stamp, topic_metadata, message_definition] : messages) {
      rw_storage->create_topic(topic_metadata, message_definition);
      auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
      auto std_string_msg = std::make_shared<std_msgs::msg::String>();
      std_string_msg->data = string_message;
      bag_message->serialized_data = memory_management->serialize_message(std_string_msg);
      bag_message->recv_timestamp = time_stamp;
      bag_message->send_timestamp = time_stamp;
      bag_message->topic_name = topic_metadata.name;
      rw_storage->write(bag_message);
    }
  }

  // Per-test-suite tear-down.
  // Called after the last test in this test suite.
  static void TearDownTestSuite()
  {
    if (fs::remove_all(fs::path(temporary_dir_path_str_)) == 0) {
      std::cerr << "Failed to remove temporary directory\n";
    }
  }

  // The storage factory shall persist while returned storage object persist
  rosbag2_storage::StorageFactory storage_factory_;
  static const char storage_id_[];
  static std::string temporary_dir_path_str_;
};

const char McapTopicFilterTestFixture::storage_id_[] = "mcap";

// Note: It is safe to use static string here since we don't expect to use temporary_dir_path_ in
// other static variables or constants initialization or in other translation units.
// Note: The temporary_dir_path_ shall not be used in the McapTopicFilterTestFixture ctor since its
// initialization deferred to the SetUpTestSuite()
std::string McapTopicFilterTestFixture::temporary_dir_path_str_;  // NOLINT

TEST_F(McapTopicFilterTestFixture, CanSelectTopicsAndServicesWithEmptyFilters)
{
  auto readable_storage = open_test_bag_for_read_only();

  rosbag2_storage::StorageFilter storage_filter{};
  readable_storage->set_filter(storage_filter);

  EXPECT_TRUE(readable_storage->has_next());
  auto first_message = readable_storage->read_next();
  EXPECT_THAT(first_message->topic_name, Eq("topic1"));
  EXPECT_TRUE(readable_storage->has_next());
  auto second_message = readable_storage->read_next();
  EXPECT_THAT(second_message->topic_name, Eq("service_topic1/_service_event"));
  EXPECT_TRUE(readable_storage->has_next());
  auto third_message = readable_storage->read_next();
  EXPECT_THAT(third_message->topic_name, Eq("topic2"));
  EXPECT_TRUE(readable_storage->has_next());
  auto fourth_message = readable_storage->read_next();
  EXPECT_THAT(fourth_message->topic_name, Eq("service_topic2/_service_event"));
  EXPECT_TRUE(readable_storage->has_next());
  auto fifth_message = readable_storage->read_next();
  EXPECT_THAT(fifth_message->topic_name, Eq("topic3"));
  EXPECT_FALSE(readable_storage->has_next());
}

TEST_F(McapTopicFilterTestFixture, CanSelectWithTopicsListOnly)
{
  auto readable_storage = open_test_bag_for_read_only();

  rosbag2_storage::StorageFilter storage_filter{};
  storage_filter.topics = {"topic1"};
  readable_storage->set_filter(storage_filter);

  EXPECT_TRUE(readable_storage->has_next());
  auto first_message = readable_storage->read_next();
  EXPECT_THAT(first_message->topic_name, Eq("topic1"));
  EXPECT_TRUE(readable_storage->has_next());
  auto second_message = readable_storage->read_next();
  EXPECT_THAT(second_message->topic_name, Eq("service_topic1/_service_event"));
  EXPECT_TRUE(readable_storage->has_next());
  auto third_message = readable_storage->read_next();
  EXPECT_THAT(third_message->topic_name, Eq("service_topic2/_service_event"));
  EXPECT_FALSE(readable_storage->has_next());
}

TEST_F(McapTopicFilterTestFixture, CanSelectWithServiceEventsListOnly)
{
  auto readable_storage = open_test_bag_for_read_only();

  rosbag2_storage::StorageFilter storage_filter{};
  storage_filter.services_events = {"service_topic2/_service_event"};
  readable_storage->set_filter(storage_filter);

  EXPECT_TRUE(readable_storage->has_next());
  auto first_message = readable_storage->read_next();
  EXPECT_THAT(first_message->topic_name, Eq("topic1"));
  EXPECT_TRUE(readable_storage->has_next());
  EXPECT_TRUE(readable_storage->has_next());
  auto second_message = readable_storage->read_next();
  EXPECT_THAT(second_message->topic_name, Eq("topic2"));
  EXPECT_TRUE(readable_storage->has_next());
  auto third_message = readable_storage->read_next();
  EXPECT_THAT(third_message->topic_name, Eq("service_topic2/_service_event"));
  EXPECT_TRUE(readable_storage->has_next());
  auto fourth_message = readable_storage->read_next();
  EXPECT_THAT(fourth_message->topic_name, Eq("topic3"));
  EXPECT_FALSE(readable_storage->has_next());
}

TEST_F(McapTopicFilterTestFixture, TestResetFilter)
{
  auto readable_storage = open_test_bag_for_read_only();

  rosbag2_storage::StorageFilter storage_filter{};
  storage_filter.topics = {"topic1"};
  readable_storage->set_filter(storage_filter);

  EXPECT_TRUE(readable_storage->has_next());
  auto first_message = readable_storage->read_next();
  EXPECT_THAT(first_message->topic_name, Eq("topic1"));
  EXPECT_TRUE(readable_storage->has_next());
  auto second_message = readable_storage->read_next();
  EXPECT_THAT(second_message->topic_name, Eq("service_topic1/_service_event"));
  EXPECT_TRUE(readable_storage->has_next());
  auto third_message = readable_storage->read_next();
  EXPECT_THAT(third_message->topic_name, Eq("service_topic2/_service_event"));
  EXPECT_FALSE(readable_storage->has_next());

  readable_storage->reset_filter();

  EXPECT_TRUE(readable_storage->has_next());
  auto fourth_message = readable_storage->read_next();
  EXPECT_THAT(fourth_message->topic_name, Eq("topic3"));
  EXPECT_FALSE(readable_storage->has_next());
}

TEST_F(McapTopicFilterTestFixture, CanSelectFromTopicsListAndRegexWithServices)
{
  auto readable_storage = open_test_bag_for_read_only();
  // Set topic list and regex for service
  rosbag2_storage::StorageFilter storage_filter{};
  storage_filter.topics = {"topic2", "topic3"};
  storage_filter.regex = "service.*";  // Add service
  readable_storage->set_filter(storage_filter);

  EXPECT_TRUE(readable_storage->has_next());
  auto first_message = readable_storage->read_next();
  EXPECT_THAT(first_message->topic_name, Eq("service_topic1/_service_event"));
  EXPECT_TRUE(readable_storage->has_next());
  auto second_message = readable_storage->read_next();
  EXPECT_THAT(second_message->topic_name, Eq("topic2"));
  EXPECT_TRUE(readable_storage->has_next());
  auto third_message = readable_storage->read_next();
  EXPECT_THAT(third_message->topic_name, Eq("service_topic2/_service_event"));
  EXPECT_TRUE(readable_storage->has_next());
  auto fourth_message = readable_storage->read_next();
  EXPECT_THAT(fourth_message->topic_name, Eq("topic3"));
  EXPECT_FALSE(readable_storage->has_next());
}

TEST_F(McapTopicFilterTestFixture, CanSelectFromServicesListAndRegexWithTopics)
{
  auto readable_storage = open_test_bag_for_read_only();
  // Set service list and regex for topic
  rosbag2_storage::StorageFilter storage_filter{};
  storage_filter.services_events = {"service_topic2/_service_event"};
  storage_filter.regex = "topic(1|3)";  // Add topic1 and topic3
  readable_storage->set_filter(storage_filter);

  EXPECT_TRUE(readable_storage->has_next());
  auto first_message = readable_storage->read_next();
  EXPECT_THAT(first_message->topic_name, Eq("topic1"));
  EXPECT_TRUE(readable_storage->has_next());
  auto second_message = readable_storage->read_next();
  EXPECT_THAT(second_message->topic_name, Eq("service_topic2/_service_event"));
  EXPECT_TRUE(readable_storage->has_next());
  auto third_message = readable_storage->read_next();
  EXPECT_THAT(third_message->topic_name, Eq("topic3"));
  EXPECT_FALSE(readable_storage->has_next());
}

TEST_F(McapTopicFilterTestFixture, CanSelectFromTopicsListAndServiceList)
{
  auto readable_storage = open_test_bag_for_read_only();
  // Set topic list and service list
  rosbag2_storage::StorageFilter storage_filter{};
  storage_filter.topics = {"topic2", "topic3"};
  storage_filter.services_events = {"service_topic1/_service_event"};
  readable_storage->set_filter(storage_filter);

  EXPECT_TRUE(readable_storage->has_next());
  auto first_message = readable_storage->read_next();
  EXPECT_THAT(first_message->topic_name, Eq("service_topic1/_service_event"));
  EXPECT_TRUE(readable_storage->has_next());
  auto second_message = readable_storage->read_next();
  EXPECT_THAT(second_message->topic_name, Eq("topic2"));
  EXPECT_TRUE(readable_storage->has_next());
  auto third_message = readable_storage->read_next();
  EXPECT_THAT(third_message->topic_name, Eq("topic3"));
  EXPECT_FALSE(readable_storage->has_next());
}

TEST_F(McapTopicFilterTestFixture, FilterTopicsAndServicesWithRegexOnly)
{
  auto readable_storage = open_test_bag_for_read_only();
  // No topic list and service list. Only regex
  rosbag2_storage::StorageFilter storage_filter{};
  storage_filter.regex = ".*topic2.*";
  readable_storage->set_filter(storage_filter);

  EXPECT_TRUE(readable_storage->has_next());
  auto first_message = readable_storage->read_next();
  EXPECT_THAT(first_message->topic_name, Eq("topic2"));
  EXPECT_TRUE(readable_storage->has_next());
  auto second_message = readable_storage->read_next();
  EXPECT_THAT(second_message->topic_name, Eq("service_topic2/_service_event"));
  EXPECT_FALSE(readable_storage->has_next());
}

TEST_F(McapTopicFilterTestFixture, FilterTopicsAndServicesWithRegexAndNonexistentTopicsList)
{
  auto readable_storage = open_test_bag_for_read_only();
  // Topics list with non-existent topic and regex with topic and service event
  rosbag2_storage::StorageFilter storage_filter{};
  storage_filter.topics = {"non-existent_topic"};
  storage_filter.regex = ".*topic2.*";
  readable_storage->set_filter(storage_filter);

  EXPECT_TRUE(readable_storage->has_next());
  auto first_message = readable_storage->read_next();
  EXPECT_THAT(first_message->topic_name, Eq("topic2"));
  EXPECT_TRUE(readable_storage->has_next());
  auto second_message = readable_storage->read_next();
  EXPECT_THAT(second_message->topic_name, Eq("service_topic2/_service_event"));
  EXPECT_FALSE(readable_storage->has_next());
}

TEST_F(McapTopicFilterTestFixture, FilterTopicsAndServicesWithRegexAndNonexistentServicesList)
{
  auto readable_storage = open_test_bag_for_read_only();
  // Service events list with non-existent service and regex with topic and service event
  rosbag2_storage::StorageFilter storage_filter{};
  storage_filter.services_events = {"non-existent_service_topic/_service_event"};
  storage_filter.regex = ".*topic2.*";
  readable_storage->set_filter(storage_filter);

  EXPECT_TRUE(readable_storage->has_next());
  auto first_message = readable_storage->read_next();
  EXPECT_THAT(first_message->topic_name, Eq("topic2"));
  EXPECT_TRUE(readable_storage->has_next());
  auto second_message = readable_storage->read_next();
  EXPECT_THAT(second_message->topic_name, Eq("service_topic2/_service_event"));
  EXPECT_FALSE(readable_storage->has_next());
}

TEST_F(McapTopicFilterTestFixture, FilterTopicsAndServicesWithBlackListsAndExcludeRegexOnly)
{
  auto readable_storage = open_test_bag_for_read_only();
  // No topic list, service list and regex.
  // Set excluded topic list, excluded service list and excluded regex
  rosbag2_storage::StorageFilter storage_filter{};
  storage_filter.exclude_topics = {"topic1"};
  storage_filter.exclude_service_events = {"service_topic2/_service_event"};
  storage_filter.regex_to_exclude = "^topic3$";
  readable_storage->set_filter(storage_filter);

  EXPECT_TRUE(readable_storage->has_next());
  auto first_message = readable_storage->read_next();
  EXPECT_THAT(first_message->topic_name, Eq("service_topic1/_service_event"));
  EXPECT_TRUE(readable_storage->has_next());
  auto second_message = readable_storage->read_next();
  EXPECT_THAT(second_message->topic_name, Eq("topic2"));
  EXPECT_FALSE(readable_storage->has_next());
}

TEST_F(McapTopicFilterTestFixture, FilterTopicsAndServicesExcludeOverlapsWithIncludeLists)
{
  auto readable_storage = open_test_bag_for_read_only();
  // Exclude from include lists
  rosbag2_storage::StorageFilter storage_filter{};
  storage_filter.topics = {"topic1", "topic2", "topic3"};
  storage_filter.services_events = {"service_topic1/_service_event",
                                    "service_topic2/_service_event"};
  storage_filter.exclude_topics = {"topic1"};
  storage_filter.exclude_service_events = {"service_topic2/_service_event"};
  readable_storage->set_filter(storage_filter);

  EXPECT_TRUE(readable_storage->has_next());
  auto first_message = readable_storage->read_next();
  EXPECT_THAT(first_message->topic_name, Eq("service_topic1/_service_event"));
  EXPECT_TRUE(readable_storage->has_next());
  auto second_message = readable_storage->read_next();
  EXPECT_THAT(second_message->topic_name, Eq("topic2"));
  EXPECT_TRUE(readable_storage->has_next());
  auto third_message = readable_storage->read_next();
  EXPECT_THAT(third_message->topic_name, Eq("topic3"));
  EXPECT_FALSE(readable_storage->has_next());
}

TEST_F(McapTopicFilterTestFixture, FilterWithRegexAndExcludeRegex)
{
  auto readable_storage = open_test_bag_for_read_only();
  // Set regex and excluded regex.
  rosbag2_storage::StorageFilter storage_filter{};
  storage_filter.regex = ".*topic1.*";
  storage_filter.regex_to_exclude = ".*service.*";
  readable_storage->set_filter(storage_filter);

  EXPECT_TRUE(readable_storage->has_next());
  auto first_message = readable_storage->read_next();
  EXPECT_THAT(first_message->topic_name, Eq("topic1"));
  EXPECT_FALSE(readable_storage->has_next());
}

TEST_F(McapTopicFilterTestFixture, FilterWithExcludeRegexOnly)
{
  auto readable_storage = open_test_bag_for_read_only();
  // Set regex_to_exclude only.
  rosbag2_storage::StorageFilter storage_filter{};
  storage_filter.regex_to_exclude = ".*topic2.*";
  readable_storage->set_filter(storage_filter);

  EXPECT_TRUE(readable_storage->has_next());
  auto first_message = readable_storage->read_next();
  EXPECT_THAT(first_message->topic_name, Eq("topic1"));
  EXPECT_TRUE(readable_storage->has_next());
  auto second_message = readable_storage->read_next();
  EXPECT_THAT(second_message->topic_name, Eq("service_topic1/_service_event"));
  EXPECT_TRUE(readable_storage->has_next());
  auto fifth_message = readable_storage->read_next();
  EXPECT_THAT(fifth_message->topic_name, Eq("topic3"));
  EXPECT_FALSE(readable_storage->has_next());
}
