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
#include "rosbag2_storage/default_storage_id.hpp"

#include "rmw/rmw.h"

namespace rosbag2_cpp
{


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

  rosbag2_cpp::ConverterOptions converter_options{};
  return open(storage_options, converter_options);
}

void Writer::open(
  const rosbag2_storage::StorageOptions & storage_options,
  const ConverterOptions & converter_options)
{
  std::lock_guard<std::mutex> writer_lock(writer_mutex_);
  writer_impl_->open(storage_options, converter_options);
}

void Writer::close()
{
  std::lock_guard<std::mutex> writer_lock(writer_mutex_);
  writer_impl_->close();
}

void Writer::create_topic(const rosbag2_storage::TopicMetadata & topic_with_type)
{
  std::lock_guard<std::mutex> writer_lock(writer_mutex_);
  writer_impl_->create_topic(topic_with_type);
}

void Writer::create_topic(
  const rosbag2_storage::TopicMetadata & topic_with_type,
  const rosbag2_storage::MessageDefinition & message_definition)
{
  std::lock_guard<std::mutex> writer_lock(writer_mutex_);
  writer_impl_->create_topic(topic_with_type, message_definition);
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

void Writer::split_bagfile()
{
  std::lock_guard<std::mutex> writer_lock(writer_mutex_);
  return writer_impl_->split_bagfile();
}

void Writer::write(std::shared_ptr<const rosbag2_storage::SerializedBagMessage> message)
{
  std::lock_guard<std::mutex> writer_lock(writer_mutex_);
  writer_impl_->write(message);
}

void Writer::write(
  std::shared_ptr<const rosbag2_storage::SerializedBagMessage> message,
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
  std::shared_ptr<const rclcpp::SerializedMessage> message,
  const std::string & topic_name,
  const std::string & type_name,
  const rclcpp::Time & time)
{
  write(message, topic_name, type_name, time.nanoseconds(), time.nanoseconds());
}

void Writer::write(
  std::shared_ptr<const rclcpp::SerializedMessage> message,
  const std::string & topic_name,
  const std::string & type_name,
  const rcutils_time_point_value_t & recv_timestamp,
  const rcutils_time_point_value_t & send_timestamp)
{
  auto serialized_bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  serialized_bag_message->topic_name = topic_name;
  serialized_bag_message->recv_timestamp = recv_timestamp;
  serialized_bag_message->send_timestamp = send_timestamp;
  // point to actual data and keep reference to original message to avoid premature releasing
  serialized_bag_message->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
    new rcutils_uint8_array_t(message->get_rcl_serialized_message()),
    [message](rcutils_uint8_array_t * data) {
      (void)message;
      if (data != nullptr) {
        data->buffer = nullptr;
        delete data;
      }
    });

  return write(serialized_bag_message, topic_name, type_name, rmw_get_serialization_format());
}

void Writer::add_event_callbacks(bag_events::WriterEventCallbacks & callbacks)
{
  writer_impl_->add_event_callbacks(callbacks);
}

}  // namespace rosbag2_cpp
