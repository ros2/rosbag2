// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#include "rmw/rmw.h"

#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/writer.hpp"

#include "rosbag2_storage/bag_metadata.hpp"
#include "rosbag2_storage/metadata_io.hpp"
#include "rosbag2_storage/ros_helper.hpp"
#include "rosbag2_storage/topic_metadata.hpp"

#include "rosbag2_test_common/tested_storage_ids.hpp"
#include "rosbag2_test_common/temporary_directory_fixture.hpp"

#include "std_msgs/msg/string.hpp"
#include "test_msgs/msg/basic_types.hpp"

#include "fake_data.hpp"
#include "mock_converter.hpp"
#include "mock_converter_factory.hpp"
#include "mock_metadata_io.hpp"
#include "mock_storage.hpp"
#include "mock_storage_factory.hpp"

using namespace testing;  // NOLINT
using rosbag2_test_common::ParametrizedTemporaryDirectoryFixture;
using rosbag2_test_common::TemporaryDirectoryFixture;
namespace fs = std::filesystem;

class SequentialReaderTest : public Test
{
public:
  SequentialReaderTest()
  : storage_(std::make_shared<NiceMock<MockStorage>>()),
    converter_factory_(std::make_shared<StrictMock<MockConverterFactory>>()),
    storage_serialization_format_("rmw1_format"),
    storage_uri_(fs::temp_directory_path().generic_string()),
    default_storage_options_({storage_uri_, "mock_storage"})
  {
    rosbag2_storage::TopicMetadata topic_with_type;
    topic_with_type.name = "topic";
    topic_with_type.type = "test_msgs/BasicTypes";
    topic_with_type.serialization_format = storage_serialization_format_;
    topic_with_type.type_description_hash = "";
    auto topics_and_types = std::vector<rosbag2_storage::TopicMetadata>{topic_with_type};

    auto message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
    message->topic_name = topic_with_type.name;

    relative_file_path_ =
      (fs::path(storage_uri_) / "some/folder").generic_string();
    auto storage_factory = std::make_unique<StrictMock<MockStorageFactory>>();
    auto metadata_io = std::make_unique<NiceMock<MockMetadataIo>>();
    bag_file_1_path_ = relative_file_path_ / "bag_file1";
    bag_file_2_path_ = relative_file_path_ / "bag_file2";
    metadata_.relative_file_paths = {
      bag_file_1_path_.generic_string(),
      bag_file_2_path_.generic_string()
    };
    metadata_.version = 4;
    metadata_.topics_with_message_count.push_back({{topic_with_type}, 6});
    metadata_.storage_identifier = "mock_storage";

    EXPECT_CALL(*metadata_io, read_metadata(_)).WillRepeatedly(Return(metadata_));
    EXPECT_CALL(*metadata_io, metadata_file_exists(_)).WillRepeatedly(Return(true));

    EXPECT_CALL(*storage_, get_all_topics_and_types())
    .Times(AtMost(1)).WillRepeatedly(Return(topics_and_types));
    // 5 messages in the first bag file, then infinite in the second
    EXPECT_CALL(*storage_, has_next()).Times(AnyNumber());
    ON_CALL(*storage_, has_next).WillByDefault(
      [this]() {
        num_next_++;
        if (num_next_ % 5 == 0 && num_next_ > 0) {
          return false;
        } else {
          return true;
        }
      });
    EXPECT_CALL(*storage_, has_next_file()).WillRepeatedly(Return(true));
    EXPECT_CALL(*storage_, read_next()).WillRepeatedly(Return(message));
    ON_CALL(*storage_, set_read_order).WillByDefault(Return(true));

    EXPECT_CALL(*storage_factory, open_read_only(_)).Times(AnyNumber());
    ON_CALL(*storage_factory, open_read_only).WillByDefault(
      [this](const rosbag2_storage::StorageOptions & storage_options) {
        EXPECT_TRUE(
          std::find(
            metadata_.relative_file_paths.begin(),
            metadata_.relative_file_paths.end(),
            storage_options.uri) !=
          metadata_.relative_file_paths.end());
        // Storage_id has to be set to something for open to succeed
        EXPECT_EQ(storage_options.storage_id, "mock_storage");
        return storage_;
      });

    auto sequential_reader = std::make_unique<rosbag2_cpp::readers::SequentialReader>(
      std::move(storage_factory), converter_factory_, std::move(metadata_io));
    reader_ = std::make_unique<rosbag2_cpp::Reader>(std::move(sequential_reader));
  }

  std::shared_ptr<NiceMock<MockStorage>> storage_;
  std::shared_ptr<StrictMock<MockConverterFactory>> converter_factory_;
  std::unique_ptr<rosbag2_cpp::Reader> reader_;
  std::string storage_serialization_format_;
  std::string storage_uri_;
  rosbag2_storage::BagMetadata metadata_;
  fs::path relative_file_path_;
  fs::path bag_file_1_path_;
  fs::path bag_file_2_path_;
  rosbag2_storage::StorageOptions default_storage_options_;
  size_t num_next_ = 0;
};

TEST_F(SequentialReaderTest, read_next_uses_converters_to_convert_serialization_format) {
  std::string output_format = "rmw2_format";

  auto format1_converter = std::make_unique<StrictMock<MockConverter>>();
  auto format2_converter = std::make_unique<StrictMock<MockConverter>>();
  EXPECT_CALL(*format1_converter, deserialize(_, _, _)).Times(1);
  EXPECT_CALL(*format2_converter, serialize(_, _, _)).Times(1);

  EXPECT_CALL(*converter_factory_, load_deserializer(storage_serialization_format_))
  .WillOnce(Return(ByMove(std::move(format1_converter))));
  EXPECT_CALL(*converter_factory_, load_serializer(output_format))
  .WillOnce(Return(ByMove(std::move(format2_converter))));

  reader_->open(default_storage_options_, {"", output_format});
  reader_->read_next();
}

TEST_F(SequentialReaderTest, open_throws_error_if_converter_plugin_does_not_exist) {
  std::string output_format = "rmw2_format";

  auto format1_converter = std::make_unique<StrictMock<MockConverter>>();
  EXPECT_CALL(*converter_factory_, load_deserializer(storage_serialization_format_))
  .WillOnce(Return(ByMove(std::move(format1_converter))));
  EXPECT_CALL(*converter_factory_, load_serializer(output_format))
  .WillOnce(Return(ByMove(nullptr)));

  EXPECT_ANY_THROW(reader_->open(default_storage_options_, {"", output_format}));
}

TEST_F(
  SequentialReaderTest,
  read_next_does_not_use_converters_if_input_and_output_format_are_equal) {
  std::string storage_serialization_format = "rmw1_format";

  EXPECT_CALL(*converter_factory_, load_deserializer(storage_serialization_format)).Times(0);
  EXPECT_CALL(*converter_factory_, load_serializer(storage_serialization_format)).Times(0);

  reader_->open(default_storage_options_, {"", storage_serialization_format});
  reader_->read_next();
}

TEST_F(SequentialReaderTest, set_filter_calls_storage) {
  // Prior to opening the file, setting filter should throw exception
  rosbag2_storage::StorageFilter storage_filter;
  storage_filter.topics.push_back("topic");
  EXPECT_ANY_THROW(reader_->get_implementation_handle().set_filter(storage_filter));
  EXPECT_ANY_THROW(reader_->get_implementation_handle().reset_filter());

  // Three times + initial open
  EXPECT_CALL(*storage_, set_filter(_)).Times(4);
  reader_->open(default_storage_options_, {"", storage_serialization_format_});
  reader_->get_implementation_handle().set_filter(storage_filter);
  reader_->read_next();
  storage_filter.topics.clear();
  storage_filter.topics.push_back("topic2");
  reader_->get_implementation_handle().set_filter(storage_filter);
  reader_->read_next();
  reader_->get_implementation_handle().reset_filter();
  reader_->read_next();
}

TEST_F(SequentialReaderTest, open_determines_unspecified_storage_id_from_metadata) {
  auto storage_options = default_storage_options_;
  storage_options.storage_id = "";
  // This call fails if the SequentialReader doesn't pull storage impl from metadata
  reader_->open(storage_options, {"", storage_serialization_format_});
}

TEST_F(SequentialReaderTest, next_file_calls_callback) {
  bool callback_called = false;
  std::string closed_file, opened_file;
  rosbag2_cpp::bag_events::ReaderEventCallbacks callbacks;
  callbacks.read_split_callback =
    [&callback_called, &closed_file, &opened_file](rosbag2_cpp::bag_events::BagSplitInfo & info) {
      closed_file = info.closed_file;
      opened_file = info.opened_file;
      callback_called = true;
    };
  reader_->add_event_callbacks(callbacks);
  ASSERT_TRUE(reader_->has_callback_for_event(rosbag2_cpp::bag_events::BagEvent::READ_SPLIT));

  reader_->open(default_storage_options_, {"", storage_serialization_format_});
  // Calling read_next() 6 times should trigger the read-split event callback
  reader_->read_next();
  reader_->read_next();
  reader_->read_next();
  reader_->read_next();
  reader_->read_next();
  reader_->read_next();

  ASSERT_TRUE(callback_called);
  EXPECT_EQ(closed_file, bag_file_1_path_.generic_string());
  EXPECT_EQ(opened_file, bag_file_2_path_.generic_string());
}

class SequentialReaderMixedFormatsTest : public Test
{
public:
  void init_reader(const std::vector<std::string> & serialization_formats)
  {
    storage_ = std::make_shared<NiceMock<MockStorage>>();
    converter_factory_ = std::make_shared<StrictMock<MockConverterFactory>>();
    auto storage_factory = std::make_unique<NiceMock<MockStorageFactory>>();
    auto metadata_io = std::make_unique<NiceMock<MockMetadataIo>>();

    storage_uri_ = fs::temp_directory_path().generic_string();

    rosbag2_storage::BagMetadata metadata;
    metadata.version = 4;
    metadata.relative_file_paths = {(fs::path(storage_uri_) / "bag_file1").generic_string()};
    metadata.storage_identifier = "mock_storage";
    std::vector<rosbag2_storage::TopicMetadata> topics_and_types;
    size_t topic_id = 0;
    for (const auto & serialization_format : serialization_formats) {
      rosbag2_storage::TopicMetadata topic_metadata;
      topic_metadata.id = static_cast<uint16_t>(topic_id);
      topic_metadata.name = "topic" + std::to_string(topic_id);
      topic_metadata.type = "test_msgs/BasicTypes";
      topic_metadata.serialization_format = serialization_format;
      metadata.topics_with_message_count.push_back({topic_metadata, 1});
      topics_and_types.push_back(topic_metadata);
      topic_id++;
    }

    auto message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
    message->topic_name = "topic0";

    ON_CALL(*metadata_io, read_metadata(_)).WillByDefault(Return(metadata));
    ON_CALL(*metadata_io, metadata_file_exists(_)).WillByDefault(Return(true));
    ON_CALL(*storage_, get_all_topics_and_types()).WillByDefault(Return(topics_and_types));
    ON_CALL(*storage_, has_next()).WillByDefault(Return(true));
    ON_CALL(*storage_, read_next()).WillByDefault(Return(message));
    ON_CALL(*storage_, set_read_order).WillByDefault(Return(true));
    ON_CALL(*storage_factory, open_read_only(_)).WillByDefault(Return(storage_));

    auto sequential_reader = std::make_unique<rosbag2_cpp::readers::SequentialReader>(
      std::move(storage_factory), converter_factory_, std::move(metadata_io));
    reader_ = std::make_unique<rosbag2_cpp::Reader>(std::move(sequential_reader));
  }

  std::shared_ptr<NiceMock<MockStorage>> storage_;
  std::shared_ptr<StrictMock<MockConverterFactory>> converter_factory_;
  std::unique_ptr<rosbag2_cpp::Reader> reader_;
  std::string storage_uri_;
};

TEST_F(SequentialReaderMixedFormatsTest, open_and_read_succeed_if_output_format_matches_a_topic) {
  init_reader({"rmw1_format", "rmw2_format"});

  // No converter shall be created; messages are returned in their stored serialization format.
  EXPECT_CALL(*converter_factory_, load_deserializer(_)).Times(0);
  EXPECT_CALL(*converter_factory_, load_serializer(_)).Times(0);

  EXPECT_NO_THROW(reader_->open({storage_uri_, "mock_storage"}, {"", "rmw1_format"}));
  EXPECT_TRUE(reader_->has_next());
  auto message = reader_->read_next();
  ASSERT_NE(nullptr, message);
  EXPECT_EQ("topic0", message->topic_name);
}

TEST_F(SequentialReaderMixedFormatsTest, open_and_read_succeed_if_no_output_format_is_requested) {
  init_reader({"rmw1_format", "rmw2_format"});

  EXPECT_CALL(*converter_factory_, load_deserializer(_)).Times(0);
  EXPECT_CALL(*converter_factory_, load_serializer(_)).Times(0);

  EXPECT_NO_THROW(reader_->open({storage_uri_, "mock_storage"}, {"", ""}));
  EXPECT_TRUE(reader_->has_next());
  auto message = reader_->read_next();
  ASSERT_NE(nullptr, message);
  EXPECT_EQ("topic0", message->topic_name);
}

TEST_F(SequentialReaderMixedFormatsTest, open_throws_if_output_format_matches_no_topic) {
  init_reader({"rmw1_format", "rmw2_format"});

  EXPECT_THROW(
    reader_->open({storage_uri_, "mock_storage"}, {"", "rmw3_format"}),
    std::runtime_error);
}

TEST_F(SequentialReaderMixedFormatsTest, read_next_fills_in_serialization_format_from_metadata) {
  init_reader({"rmw1_format", "rmw2_format"});

  EXPECT_CALL(*converter_factory_, load_deserializer(_)).Times(0);
  EXPECT_CALL(*converter_factory_, load_serializer(_)).Times(0);

  // The mock storage leaves the serialization format of the message empty, so the reader has
  // to fill it in from the topic metadata.
  EXPECT_NO_THROW(reader_->open({storage_uri_, "mock_storage"}, {"", ""}));
  auto message = reader_->read_next();
  ASSERT_NE(nullptr, message);
  EXPECT_EQ("topic0", message->topic_name);
  EXPECT_EQ("rmw1_format", message->serialization_format);
}

TEST_F(SequentialReaderMixedFormatsTest, read_next_keeps_serialization_format_set_by_storage) {
  init_reader({"rmw1_format", "rmw2_format"});

  auto message_from_storage = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  message_from_storage->topic_name = "topic0";
  message_from_storage->serialization_format = "rmw1_format";
  ON_CALL(*storage_, read_next()).WillByDefault(Return(message_from_storage));

  EXPECT_NO_THROW(reader_->open({storage_uri_, "mock_storage"}, {"", "rmw1_format"}));
  auto message = reader_->read_next();
  ASSERT_NE(nullptr, message);
  EXPECT_EQ("rmw1_format", message->serialization_format);
}

TEST_F(SequentialReaderMixedFormatsTest, read_next_throws_if_message_is_not_in_output_format) {
  init_reader({"rmw1_format", "rmw2_format"});

  // topic1 is stored in rmw2_format, which is neither the requested output serialization
  // format nor convertible to it, since the bag has mixed serialization formats.
  auto message_from_storage = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  message_from_storage->topic_name = "topic1";
  ON_CALL(*storage_, read_next()).WillByDefault(Return(message_from_storage));

  EXPECT_CALL(*converter_factory_, load_deserializer(_)).Times(0);
  EXPECT_CALL(*converter_factory_, load_serializer(_)).Times(0);

  EXPECT_NO_THROW(reader_->open({storage_uri_, "mock_storage"}, {"", "rmw1_format"}));
  EXPECT_TRUE(reader_->has_next());
  EXPECT_THROW(reader_->read_next(), std::runtime_error);
}

TEST_F(SequentialReaderMixedFormatsTest, read_next_returns_any_format_if_none_is_requested) {
  init_reader({"rmw1_format", "rmw2_format"});

  auto message_from_storage = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  message_from_storage->topic_name = "topic1";
  ON_CALL(*storage_, read_next()).WillByDefault(Return(message_from_storage));

  EXPECT_NO_THROW(reader_->open({storage_uri_, "mock_storage"}, {"", ""}));
  auto message = reader_->read_next();
  ASSERT_NE(nullptr, message);
  EXPECT_EQ("topic1", message->topic_name);
  EXPECT_EQ("rmw2_format", message->serialization_format);
}

namespace
{
std::shared_ptr<rosbag2_storage::SerializedBagMessage> make_fake_protobuf_message(
  const std::string & topic_name, int64_t stamp_ns, uint8_t index)
{
  // A minimal, valid protobuf encoding of a foxglove.CompressedVideo message:
  //   field 2 (frame_id, string) = "cam0"
  //   field 3 (data, bytes) = 16 filler bytes
  //   field 4 (format, string) = "h264"
  std::vector<uint8_t> payload = {0x12, 0x04, 'c', 'a', 'm', '0', 0x1a, 0x10};
  payload.insert(payload.end(), 16, index);
  const std::vector<uint8_t> format_field = {0x22, 0x04, 'h', '2', '6', '4'};
  payload.insert(payload.end(), format_field.begin(), format_field.end());

  auto message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  message->topic_name = topic_name;
  message->recv_timestamp = stamp_ns;
  message->send_timestamp = stamp_ns;
  message->serialized_data =
    rosbag2_storage::make_serialized_message(payload.data(), payload.size());
  return message;
}
}  // namespace

TEST_F(TemporaryDirectoryFixture, reads_mixed_serialization_formats_from_real_bag) {
  const auto bag_path = (fs::path(temporary_dir_path_) / "mixed_formats_bag").generic_string();
  const std::string cdr_topic = "/chatter";
  const std::string protobuf_topic = "/camera/video_compressed";
  constexpr size_t kNumMessagesPerTopic = 5;

  // Write a small bag with one CDR encoded and one protobuf encoded topic, mimicking
  // recordings made by non-ROS tools.
  {
    rosbag2_cpp::Writer writer;
    rosbag2_storage::StorageOptions options;
    options.uri = bag_path;
    options.storage_id = "mcap";
    writer.open(options);

    rosbag2_storage::TopicMetadata protobuf_topic_metadata;
    protobuf_topic_metadata.name = protobuf_topic;
    protobuf_topic_metadata.type = "foxglove.CompressedVideo";
    protobuf_topic_metadata.serialization_format = "protobuf";
    writer.create_topic(protobuf_topic_metadata);

    test_msgs::msg::BasicTypes msg;
    for (size_t i = 0; i < kNumMessagesPerTopic; i++) {
      const int64_t stamp_ns = 10000000 * static_cast<int64_t>(i);
      msg.int32_value = static_cast<int32_t>(i);
      writer.write(msg, cdr_topic, rclcpp::Time(stamp_ns));
      writer.write(make_fake_protobuf_message(protobuf_topic, stamp_ns + 1,
        static_cast<uint8_t>(i)));
    }
  }

  // A bag with mixed serialization formats can be opened and read raw when no output
  // serialization format is requested.
  {
    rosbag2_cpp::Reader reader;
    EXPECT_NO_THROW(reader.open(bag_path));

    std::unordered_map<std::string, std::string> serialization_formats;
    for (const auto & topic : reader.get_all_topics_and_types()) {
      serialization_formats[topic.name] = topic.serialization_format;
    }
    EXPECT_EQ(serialization_formats[protobuf_topic], "protobuf");
    EXPECT_NE(serialization_formats[cdr_topic], "protobuf");

    std::unordered_map<std::string, size_t> counts;
    while (reader.has_next()) {
      const auto message = reader.read_next();
      counts[message->topic_name]++;
      // Every message is tagged with the serialization format of its topic.
      EXPECT_EQ(message->serialization_format, serialization_formats[message->topic_name]);
    }
    EXPECT_EQ(counts[cdr_topic], kNumMessagesPerTopic);
    EXPECT_EQ(counts[protobuf_topic], kNumMessagesPerTopic);
  }

  // Opening also succeeds when the requested output serialization format matches some of
  // the topics; messages are not converted. Messages of the matching topics are returned as
  // stored, but reading a message of a topic in another serialization format throws instead
  // of handing out data in an unexpected serialization format.
  {
    rosbag2_cpp::Reader reader;
    rosbag2_storage::StorageOptions options;
    options.uri = bag_path;
    options.storage_id = "mcap";
    EXPECT_NO_THROW(reader.open(options, {"", rmw_get_serialization_format()}));
    ASSERT_TRUE(reader.has_next());
    // The first message in the bag is on the CDR topic.
    const auto message = reader.read_next();
    EXPECT_EQ(message->topic_name, cdr_topic);
    EXPECT_EQ(message->serialization_format, rmw_get_serialization_format());
    // The second message is on the protobuf topic.
    ASSERT_TRUE(reader.has_next());
    EXPECT_THROW(reader.read_next(), std::runtime_error);
  }

  // Excluding the topics which are not stored in the requested output serialization format
  // makes the whole bag readable.
  {
    rosbag2_cpp::Reader reader;
    rosbag2_storage::StorageOptions options;
    options.uri = bag_path;
    options.storage_id = "mcap";
    EXPECT_NO_THROW(reader.open(options, {"", rmw_get_serialization_format()}));
    rosbag2_storage::StorageFilter filter;
    filter.topics = {cdr_topic};
    reader.set_filter(filter);
    size_t count = 0;
    while (reader.has_next()) {
      const auto message = reader.read_next();
      EXPECT_EQ(message->topic_name, cdr_topic);
      EXPECT_EQ(message->serialization_format, rmw_get_serialization_format());
      count++;
    }
    EXPECT_EQ(count, kNumMessagesPerTopic);
  }

  // Opening fails when the requested output serialization format matches none of the
  // topics, since conversion of mixed serialization formats is not supported.
  {
    rosbag2_cpp::Reader reader;
    rosbag2_storage::StorageOptions options;
    options.uri = bag_path;
    options.storage_id = "mcap";
    EXPECT_THROW(reader.open(options, {"", "some_other_format"}), std::runtime_error);
  }
}

TEST_P(ParametrizedTemporaryDirectoryFixture, reader_accepts_bare_file) {
  const auto bag_path = fs::path(temporary_dir_path_) / "bag";
  const auto storage_id = GetParam();

  {
    // Create an empty bag with default storage
    rosbag2_cpp::Writer writer;
    rosbag2_storage::StorageOptions options;
    options.uri = bag_path.generic_string();
    options.storage_id = storage_id;
    writer.open(options);
    test_msgs::msg::BasicTypes msg;
    writer.write(msg, "testtopic", rclcpp::Time{});
  }
  rosbag2_storage::MetadataIo metadata_io;
  auto metadata = metadata_io.read_metadata(bag_path.generic_string());
  auto first_storage = bag_path / metadata.relative_file_paths[0];

  rosbag2_cpp::Reader reader;
  EXPECT_NO_THROW(reader.open(first_storage.generic_string()));
  EXPECT_TRUE(reader.has_next());
  EXPECT_THAT(reader.get_metadata().topics_with_message_count, SizeIs(1));
}

TEST_P(ParametrizedTemporaryDirectoryFixture, get_metadata_include_topics_with_zero_messages) {
  const auto bag_path = fs::path(temporary_dir_path_) / "bag_with_no_msgs";
  const std::string topic_name = "topic_with_0_messages";
  const auto storage_id = GetParam();
  {
    rosbag2_storage::TopicMetadata topic_metadata;
    topic_metadata.name = topic_name;
    topic_metadata.type = "std_msgs/msg/String";

    rosbag2_cpp::Writer writer;
    rosbag2_storage::StorageOptions options;
    options.uri = bag_path.string();
    options.storage_id = storage_id;
    writer.open(options);
    writer.create_topic(topic_metadata);
  }

  rosbag2_storage::MetadataIo metadata_io;
  ASSERT_TRUE(metadata_io.metadata_file_exists(bag_path.string()));
  auto metadata_from_yaml = metadata_io.read_metadata(bag_path.string());
  auto first_storage = bag_path / metadata_from_yaml.relative_file_paths[0];

  rosbag2_storage::StorageFactory factory;
  rosbag2_storage::StorageOptions options;
  options.uri = first_storage.string();
  options.storage_id = storage_id;
  auto reader = factory.open_read_only(options);
  auto metadata = reader->get_metadata();
  ASSERT_THAT(metadata.topics_with_message_count, SizeIs(1));
  EXPECT_EQ(metadata.topics_with_message_count[0].message_count, 0U);
}

INSTANTIATE_TEST_SUITE_P(
  BareFileTests,
  ParametrizedTemporaryDirectoryFixture,
  ValuesIn(rosbag2_test_common::kTestedStorageIDs)
);


class ReadOrderTest : public ParametrizedTemporaryDirectoryFixture
{
public:
  ReadOrderTest()
  {
    storage_options.uri =
      (fs::path(temporary_dir_path_) / "ordertest").generic_string();
    storage_options.storage_id = GetParam();
    write_sample_split_bag(storage_options, fake_messages, split_every);
  }

  void sort_expected(rosbag2_storage::ReadOrder order)
  {
    sorted_messages.clear();
    for (const auto & message : fake_messages) {
      sorted_messages.push_back(message);
    }

    switch (order.sort_by) {
      case rosbag2_storage::ReadOrder::ReceivedTimestamp: {
          if (order.reverse) {
            std::sort(
              sorted_messages.begin(), sorted_messages.end(), [](auto a, auto b) {
                return a.first > b.first || (a.first == b.first && a.second > b.second);
              });
          } else {
            std::sort(
              sorted_messages.begin(), sorted_messages.end(), [](auto a, auto b) {
                return a.first < b.first || (a.first == b.first && a.second < b.second);
              });
          }
        } break;
      case rosbag2_storage::ReadOrder::File: {
          if (order.reverse) {
            std::reverse(sorted_messages.begin(), sorted_messages.end());
          } else {
            // Already in forward file order
          }
        } break;
      case rosbag2_storage::ReadOrder::PublishedTimestamp:
        throw std::runtime_error("PublishedTimestamp not implemented.");
        break;
    }
  }

  void check_against_sorted(bool do_reset)
  {
    // If do_reset - try to reset the storage internal iterator every time, to test its ability
    // to track order when the query changes.
    // If not, do a single chain of uninterrupted read_next, which likely uses the same iterator
    for (const auto & expect_message : sorted_messages) {
      auto expect_timestamp = expect_message.first;
      uint32_t expect_value = expect_message.second;

      // Check both timestamp and value to uniquely identify messages in expected order
      ASSERT_TRUE(reader.has_next());
      auto next = reader.read_next();
      ASSERT_NE(next, nullptr);
      EXPECT_EQ(next->recv_timestamp, expect_timestamp);

      ASSERT_EQ(next->serialized_data->buffer_length, 4u);
      uint32_t value = *reinterpret_cast<uint32_t *>(next->serialized_data->buffer);
      EXPECT_EQ(value, expect_value);

      if (do_reset) {
        reader.reset_filter();
      }
    }
    ASSERT_FALSE(reader.has_next());
  }

  const std::vector<std::pair<rcutils_time_point_value_t, uint32_t>> fake_messages {
    {100, 1},
    {100, 2},
    {300, 3},
    {200, 4},
    {300, 5},
    {500, 6},
    {400, 7},
    {600, 8}
  };
  const size_t split_every = 5;
  std::vector<std::pair<rcutils_time_point_value_t, uint32_t>> sorted_messages;

  rosbag2_cpp::readers::SequentialReader reader{};
  rosbag2_storage::StorageOptions storage_options{};
};

TEST_P(ReadOrderTest, received_timestamp_order) {
  rosbag2_storage::ReadOrder order(rosbag2_storage::ReadOrder::ReceivedTimestamp, false);
  sort_expected(order);

  for (bool do_reset : {false, true}) {
    reader.open(storage_options, rosbag2_cpp::ConverterOptions{});
    EXPECT_TRUE(reader.set_read_order(order));
    check_against_sorted(do_reset);
    reader.close();
  }
}

TEST_P(ReadOrderTest, reverse_received_timestamp_order) {
  rosbag2_storage::ReadOrder order(rosbag2_storage::ReadOrder::ReceivedTimestamp, true);
  sort_expected(order);
  reader.open(storage_options, rosbag2_cpp::ConverterOptions{});
  EXPECT_TRUE(reader.set_read_order(order));
  auto metadata = reader.get_metadata();
  // Seek to end before reading reverse messages
  auto end_timestamp = (metadata.starting_time + metadata.duration).time_since_epoch().count();
  reader.close();

  for (bool do_reset : {false, true}) {
    reader.open(storage_options, rosbag2_cpp::ConverterOptions{});
    reader.seek(end_timestamp);
    check_against_sorted(do_reset);
    reader.close();
  }
}

TEST_P(ReadOrderTest, reverse_file_order) {
  reader.open(storage_options, rosbag2_cpp::ConverterOptions{});
  EXPECT_FALSE(
    reader.set_read_order(rosbag2_storage::ReadOrder(rosbag2_storage::ReadOrder::File, true)));
}

TEST_P(ReadOrderTest, published_timestamp_order) {
  reader.open(storage_options, rosbag2_cpp::ConverterOptions{});
  EXPECT_FALSE(
    reader.set_read_order(
      rosbag2_storage::ReadOrder(rosbag2_storage::ReadOrder::PublishedTimestamp, false)));
}

INSTANTIATE_TEST_SUITE_P(
  ThisReadOrderTest,
  ReadOrderTest,
  ValuesIn(rosbag2_test_common::kTestedStorageIDs)
);
