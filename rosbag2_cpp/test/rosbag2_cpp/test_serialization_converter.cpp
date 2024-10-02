// Copyright 2024 Apex.AI, Inc.
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

#include "mock_converter.hpp"
#include "mock_converter_factory.hpp"
#include "mock_metadata_io.hpp"
#include "mock_storage.hpp"
#include "mock_storage_factory.hpp"
#include "rmw/rmw.h"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rosbag2_cpp/rmw_implemented_serialization_format_converter.hpp"
#include "rosbag2_cpp/writers/sequential_writer.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_storage/ros_helper.hpp"
#include "std_msgs/msg/string.hpp"

using namespace testing;  // NOLINT

namespace fs = rcpputils::fs;

class SerializationConverterTest : public Test
{
public:
  SerializationConverterTest()
  {
    storage_factory_ = std::make_unique<StrictMock<MockStorageFactory>>();
    storage_ = std::make_shared<NiceMock<MockStorage>>();
    converter_factory_ = std::make_shared<StrictMock<MockConverterFactory>>();
    metadata_io_ = std::make_unique<NiceMock<MockMetadataIo>>();
    tmp_dir_path_ = fs::temp_directory_path() / "SerializationConverterTest";
    storage_options_ = rosbag2_storage::StorageOptions{};
    // We are using in memory mock storage to avoid writing to file. However, writer->open(..)
    // creating a new subfolder for the new recording. Therefore, need to provide a valid uri and
    // clean up those newly created folder in the destructor.
    storage_options_.uri = (tmp_dir_path_ / bag_base_dir_).string();
    fs::remove_all(tmp_dir_path_);

    ON_CALL(*storage_factory_, open_read_write(_)).WillByDefault(
      DoAll(
        Invoke(
          [this](const rosbag2_storage::StorageOptions & storage_options) {
            mock_storage_data_.clear();
            (void)storage_options;
          }),
        Return(storage_)));
    EXPECT_CALL(*storage_factory_, open_read_write(_)).Times(AtLeast(1));

    ON_CALL(
      *storage_,
      write(An<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>>())).WillByDefault(
      [this](std::shared_ptr<const rosbag2_storage::SerializedBagMessage> serialized_message) {
        mock_storage_data_.push_back(serialized_message);
      });

    using VectorSharedBagMessages =
      std::vector<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>>;

    EXPECT_CALL(*storage_, write(An<const VectorSharedBagMessages &>())).Times(0);

    EXPECT_CALL(*storage_, get_bagfile_size()).Times(0);
  }

  ~SerializationConverterTest() override
  {
    fs::remove_all(tmp_dir_path_);
  }

  std::shared_ptr<rosbag2_storage::SerializedBagMessage> make_test_msg()
  {
    std_msgs::msg::String std_string_msg;
    std_string_msg.data = test_msg_content_;
    auto rclcpp_serialized_msg = std::make_shared<rclcpp::SerializedMessage>();
    rclcpp::Serialization<std_msgs::msg::String> serialization;
    serialization.serialize_message(&std_string_msg, rclcpp_serialized_msg.get());
    // Convert rclcpp serialized message to the rosbag2_storage::SerializedBagMessage
    auto message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
    message->topic_name = test_topic_name_;
    message->serialized_data = rosbag2_storage::make_serialized_message(
      rclcpp_serialized_msg->get_rcl_serialized_message().buffer, rclcpp_serialized_msg->size());

    return message;
  }

  std::unique_ptr<StrictMock<MockStorageFactory>> storage_factory_;
  std::shared_ptr<NiceMock<MockStorage>> storage_;
  std::shared_ptr<StrictMock<MockConverterFactory>> converter_factory_;
  std::unique_ptr<MockMetadataIo> metadata_io_;

  fs::path tmp_dir_path_;
  const std::string bag_base_dir_ = "test_bag";
  const std::string test_msg_content_ = "Test message string";
  const std::string test_topic_name_ = "test_topic";
  rosbag2_storage::StorageOptions storage_options_;
  std::vector<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>> mock_storage_data_;
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
};

TEST_F(SerializationConverterTest, default_rmw_converter_can_deserialize) {
  auto sequential_writer = std::make_unique<rosbag2_cpp::writers::SequentialWriter>(
    std::move(storage_factory_), converter_factory_, std::move(metadata_io_));
  writer_ = std::make_unique<rosbag2_cpp::Writer>(std::move(sequential_writer));

  const std::string mock_serialization_format = "mock_serialization_format";
  const std::string rmw_serialization_format = rmw_get_serialization_format();

  auto mock_converter = std::make_unique<StrictMock<MockConverter>>();
  std::vector<std::shared_ptr<const rosbag2_cpp::rosbag2_introspection_message_t>>
  intercepted_converter_messages;
  EXPECT_CALL(
    *mock_converter,
    serialize(
      An<std::shared_ptr<const rosbag2_cpp::rosbag2_introspection_message_t>>(),
      An<const rosidl_message_type_support_t *>(),
      An<std::shared_ptr<rosbag2_storage::SerializedBagMessage>>())).
  WillRepeatedly(
    [&](std::shared_ptr<const rosbag2_cpp::rosbag2_introspection_message_t> ros_message,
    const rosidl_message_type_support_t * type_support,
    std::shared_ptr<rosbag2_storage::SerializedBagMessage> serialized_message) {
      intercepted_converter_messages.push_back(ros_message);
      (void)serialized_message;
      (void)type_support;
    });

  auto rmw_converter =
    std::make_unique<rosbag2_cpp::RMWImplementedConverter>(rmw_serialization_format);

  EXPECT_CALL(*converter_factory_, load_serializer(mock_serialization_format))
  .WillOnce(Return(ByMove(std::move(mock_converter))));
  EXPECT_CALL(*converter_factory_, load_deserializer(rmw_serialization_format))
  .WillOnce(Return(ByMove(std::move(rmw_converter))));

  auto message = make_test_msg();
  writer_->open(storage_options_, {rmw_serialization_format, mock_serialization_format});

  writer_->create_topic({test_topic_name_, "std_msgs/msg/String", "", {}});
  writer_->write(message);

  ASSERT_EQ(intercepted_converter_messages.size(), 1);
  // Using the type punning since we know the type of the message in advance
  const auto & ros_message =
    *static_cast<const std_msgs::msg::String *>(intercepted_converter_messages[0]->message);
  EXPECT_THAT(ros_message.data, StrEq(test_msg_content_));
}

TEST_F(SerializationConverterTest, default_rmw_converter_can_serialize) {
  auto sequential_writer = std::make_unique<rosbag2_cpp::writers::SequentialWriter>(
    std::move(storage_factory_), converter_factory_, std::move(metadata_io_));
  writer_ = std::make_unique<rosbag2_cpp::Writer>(std::move(sequential_writer));

  const std::string mock_serialization_format = "mock_serialization_format";
  const std::string rmw_serialization_format = rmw_get_serialization_format();

  auto mock_converter = std::make_unique<StrictMock<MockConverter>>();
  EXPECT_CALL(
    *mock_converter,
    deserialize(
      An<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>>(),
      An<const rosidl_message_type_support_t *>(),
      An<std::shared_ptr<rosbag2_cpp::rosbag2_introspection_message_t>>())).
  WillRepeatedly(
    [&](std::shared_ptr<const rosbag2_storage::SerializedBagMessage> serialized_message,
    const rosidl_message_type_support_t * type_support,
    std::shared_ptr<rosbag2_cpp::rosbag2_introspection_message_t> ros_message)
    {
      // Use rclcpp deserialization for concrete std_msgs::msg::String message type
      rclcpp::Serialization<std_msgs::msg::String> serialization;
      rclcpp::SerializedMessage rclcpp_serialized_msg(*serialized_message->serialized_data);
      serialization.deserialize_message(&rclcpp_serialized_msg, ros_message->message);
      (void)type_support;
    });

  auto rmw_converter =
    std::make_unique<rosbag2_cpp::RMWImplementedConverter>(rmw_serialization_format);

  EXPECT_CALL(*converter_factory_, load_serializer(rmw_serialization_format))
  .WillOnce(Return(ByMove(std::move(rmw_converter))));
  EXPECT_CALL(*converter_factory_, load_deserializer(mock_serialization_format))
  .WillOnce(Return(ByMove(std::move(mock_converter))));

  auto message = make_test_msg();
  writer_->open(storage_options_, {mock_serialization_format, rmw_serialization_format});

  writer_->create_topic({test_topic_name_, "std_msgs/msg/String", "", {}});
  writer_->write(message);

  ASSERT_EQ(mock_storage_data_.size(), 1);

  rclcpp::Serialization<std_msgs::msg::String> serialization;
  rclcpp::SerializedMessage rclcpp_serialized_msg(*mock_storage_data_[0]->serialized_data);
  std_msgs::msg::String ros_message;
  serialization.deserialize_message(&rclcpp_serialized_msg, &ros_message);
  EXPECT_THAT(ros_message.data, StrEq(test_msg_content_));
}
