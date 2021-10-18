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

#include "rosbag2_cpp/writer.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "rclcpp/logging.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rclcpp/time.hpp"

#include "rosbag2_cpp/info.hpp"
#include "rosbag2_cpp/writer_interfaces/base_writer_interface.hpp"

#include "rosbag2_storage/serialized_bag_message.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "rosbag2_storage/topic_metadata.hpp"

#include "rmw/rmw.h"

namespace rosbag2_cpp
{

static constexpr char const * kDefaultStorageID = "sqlite3";

Writer::Writer(std::unique_ptr<rosbag2_cpp::writer_interfaces::BaseWriterInterface> writer_impl)
: writer_impl_(std::move(writer_impl))
{}

Writer::~Writer()
{
  writer_impl_.reset();
}

void Writer::open(const std::string & uri)
{
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = uri;
  storage_options.storage_id = kDefaultStorageID;

  rosbag2_cpp::ConverterOptions converter_options{};
  return open(storage_options, converter_options);
}

void Writer::open(
  const rosbag2_storage::StorageOptions & storage_options,
  const ConverterOptions & converter_options)
{
  writer_impl_->open(storage_options, converter_options);
}

void Writer::create_topic(const rosbag2_storage::TopicMetadata & topic_with_type)
{
  std::lock_guard<std::mutex> writer_lock(writer_mutex_);
  writer_impl_->create_topic(topic_with_type);
}

void Writer::remove_topic(const rosbag2_storage::TopicMetadata & topic_with_type)
{
  std::lock_guard<std::mutex> writer_lock(writer_mutex_);
  writer_impl_->remove_topic(topic_with_type);
}

bool Writer::take_snapshot()
{
  return writer_impl_->take_snapshot();
}

void Writer::write(std::shared_ptr<rosbag2_storage::SerializedBagMessage> message)
{
  std::lock_guard<std::mutex> writer_lock(writer_mutex_);
  writer_impl_->write(message);
}

void Writer::write(
  std::shared_ptr<rosbag2_storage::SerializedBagMessage> message,
  const std::string & topic_name,
  const std::string & type_name,
  const std::string & serialization_format)
{
  if (message->topic_name != topic_name) {
    auto err = std::string("trying to write a message with mismatching topic information: ");
    err += "(" + message->topic_name + ") vs (" + topic_name + ")";
    throw std::runtime_error(err);
  }

  rosbag2_storage::TopicMetadata tm;
  tm.name = topic_name;
  tm.type = type_name;
  tm.serialization_format = serialization_format;
  create_topic(tm);
  write(message);
}

void Writer::write(
  const rclcpp::SerializedMessage & message,
  const std::string & topic_name,
  const std::string & type_name,
  const rclcpp::Time & time)
{
  auto serialized_bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  serialized_bag_message->topic_name = topic_name;
  serialized_bag_message->time_stamp = time.nanoseconds();

  serialized_bag_message->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
    new rcutils_uint8_array_t,
    [](rcutils_uint8_array_t * msg) {
      auto fini_return = rcutils_uint8_array_fini(msg);
      delete msg;
      if (fini_return != RCUTILS_RET_OK) {
        RCLCPP_ERROR_STREAM(
          rclcpp::get_logger("rosbag2_cpp"),
          "Failed to destroy serialized message: " << rcutils_get_error_string().str);
      }
    });

  // While using compression mode and cache size isn't 0, another thread deals with this serialized
  // message asynchronously.
  // In order to keep serialized message valid, have to duplicate message.

  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  rcutils_ret_t ret = rcutils_uint8_array_init(
    serialized_bag_message->serialized_data.get(),
    message.get_rcl_serialized_message().buffer_capacity,
    &allocator);
  if (ret != RCUTILS_RET_OK) {
    auto err = std::string("Failed to call rcutils_uint8_array_init(): return ");
    err += ret;
    throw std::runtime_error(err);
  }

  std::memcpy(
    serialized_bag_message->serialized_data->buffer,
    message.get_rcl_serialized_message().buffer,
    message.get_rcl_serialized_message().buffer_length);

  serialized_bag_message->serialized_data->buffer_length =
    message.get_rcl_serialized_message().buffer_length;

  return write(
    serialized_bag_message, topic_name, type_name, rmw_get_serialization_format());
}

void Writer::write(
  std::shared_ptr<rclcpp::SerializedMessage> message,
  const std::string & topic_name,
  const std::string & type_name,
  const rclcpp::Time & time)
{
  auto serialized_bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  serialized_bag_message->topic_name = topic_name;
  serialized_bag_message->time_stamp = time.nanoseconds();

  serialized_bag_message->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
    new rcutils_uint8_array_t,
    [](rcutils_uint8_array_t * msg) {
      auto fini_return = rcutils_uint8_array_fini(msg);
      delete msg;
      if (fini_return != RCUTILS_RET_OK) {
        RCLCPP_ERROR_STREAM(
          rclcpp::get_logger("rosbag2_cpp"),
          "Failed to destroy serialized message: " << rcutils_get_error_string().str);
      }
    });

  *serialized_bag_message->serialized_data = message->release_rcl_serialized_message();

  return write(serialized_bag_message, topic_name, type_name, rmw_get_serialization_format());
}

}  // namespace rosbag2_cpp
