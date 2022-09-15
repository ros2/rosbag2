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

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rcpputils/filesystem_helper.hpp"

#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/writer.hpp"

#include "rosbag2_storage/bag_metadata.hpp"
#include "rosbag2_storage/metadata_io.hpp"
#include "rosbag2_storage/topic_metadata.hpp"

#include "rosbag2_test_common/temporary_directory_fixture.hpp"

#include "test_msgs/msg/basic_types.hpp"

#include "mock_converter.hpp"
#include "mock_converter_factory.hpp"
#include "mock_metadata_io.hpp"
#include "mock_storage.hpp"
#include "mock_storage_factory.hpp"

using namespace testing;  // NOLINT
using rosbag2_test_common::TemporaryDirectoryFixture;

class SequentialReaderTest : public Test
{
public:
  SequentialReaderTest()
  : storage_(std::make_shared<NiceMock<MockStorage>>()),
    converter_factory_(std::make_shared<StrictMock<MockConverterFactory>>()),
    storage_serialization_format_("rmw1_format"),
    storage_uri_(rcpputils::fs::temp_directory_path().string()),
    default_storage_options_({storage_uri_, "mock_storage"})
  {
    rosbag2_storage::TopicMetadata topic_with_type;
    topic_with_type.name = "topic";
    topic_with_type.type = "test_msgs/BasicTypes";
    topic_with_type.serialization_format = storage_serialization_format_;
    auto topics_and_types = std::vector<rosbag2_storage::TopicMetadata>{topic_with_type};

    auto message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
    message->topic_name = topic_with_type.name;

    relative_file_path_ =
      (rcpputils::fs::path(storage_uri_) / "some/folder").string();
    auto storage_factory = std::make_unique<StrictMock<MockStorageFactory>>();
    auto metadata_io = std::make_unique<NiceMock<MockMetadataIo>>();
    bag_file_1_path_ = relative_file_path_ / "bag_file1";
    bag_file_2_path_ = relative_file_path_ / "bag_file2";
    metadata_.relative_file_paths = {bag_file_1_path_.string(), bag_file_2_path_.string()};
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
  rcpputils::fs::path relative_file_path_;
  rcpputils::fs::path bag_file_1_path_;
  rcpputils::fs::path bag_file_2_path_;
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

  reader_->open(default_storage_options_, {"", storage_serialization_format_});
  // Calling read_next() 6 times should trigger the read-split event callback
  reader_->read_next();
  reader_->read_next();
  reader_->read_next();
  reader_->read_next();
  reader_->read_next();
  reader_->read_next();

  ASSERT_TRUE(callback_called);
  EXPECT_EQ(closed_file, bag_file_1_path_.string());
  EXPECT_EQ(opened_file, bag_file_2_path_.string());
}

TEST_F(TemporaryDirectoryFixture, reader_accepts_bare_file) {
  const auto bag_path = rcpputils::fs::path(temporary_dir_path_) / "bag";
  const auto expected_bagfile_path = bag_path / "bag_0.db3";

  {
    // Create an empty bag with default storage
    rosbag2_cpp::Writer writer;
    writer.open(bag_path.string());
    test_msgs::msg::BasicTypes msg;
    writer.write(msg, "testtopic", rclcpp::Time{});
  }

  rosbag2_cpp::Reader reader;
  EXPECT_NO_THROW(reader.open(expected_bagfile_path.string()));
  EXPECT_TRUE(reader.has_next());
  EXPECT_THAT(reader.get_metadata().topics_with_message_count, SizeIs(1));
}
