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

#include "player.hpp"

#include <chrono>
#include <memory>
#include <queue>
#include <string>
#include <vector>
#include <utility>

#include "rcl/graph.h"

#include "rclcpp/rclcpp.hpp"

#include "rcutils/time.h"

#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/typesupport_helpers.hpp"

#include "rosbag2_transport/logging.hpp"

#include "rosbag2_node.hpp"
#include "replayable_message.hpp"

namespace rosbag2_transport
{

const std::chrono::milliseconds
Player::queue_read_wait_period_ = std::chrono::milliseconds(100);

Player::Player(
  std::shared_ptr<rosbag2_cpp::Reader> reader, std::shared_ptr<Rosbag2Node> rosbag2_transport)
: reader_(std::move(reader)), rosbag2_transport_(rosbag2_transport)
{}

bool Player::is_storage_completely_loaded() const
{
  if (storage_loading_future_.valid() &&
    storage_loading_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
  {
    storage_loading_future_.get();
  }
  return !storage_loading_future_.valid();
}

void Player::play(const PlayOptions & options)
{
  prepare_publishers();

  if(options.use_current_time)
  {
    prepare_topic_ts_map();
  }

  storage_loading_future_ = std::async(
    std::launch::async,
    [this, options]() {load_storage_content(options);});

  wait_for_filled_queue(options);

  play_messages_from_queue();
}

void Player::wait_for_filled_queue(const PlayOptions & options) const
{
  while (
    message_queue_.size_approx() < options.read_ahead_queue_size &&
    !is_storage_completely_loaded() && rclcpp::ok())
  {
    std::this_thread::sleep_for(queue_read_wait_period_);
  }
}

void Player::load_storage_content(const PlayOptions & options)
{
  TimePoint time_first_message;

  ReplayableMessage message;
  if (reader_->has_next()) {
    message.message = reader_->read_next();
    message.time_since_start = std::chrono::nanoseconds(0);
    time_first_message = TimePoint(std::chrono::nanoseconds(message.message->time_stamp));
    message_queue_.enqueue(message);
  }

  auto queue_lower_boundary =
    static_cast<size_t>(options.read_ahead_queue_size * read_ahead_lower_bound_percentage_);
  auto queue_upper_boundary = options.read_ahead_queue_size;

  while (reader_->has_next() && rclcpp::ok()) {
    if (message_queue_.size_approx() < queue_lower_boundary) {
      enqueue_up_to_boundary(time_first_message, queue_upper_boundary);
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
}

void Player::enqueue_up_to_boundary(const TimePoint & time_first_message, uint64_t boundary)
{
  ReplayableMessage message;
  for (size_t i = message_queue_.size_approx(); i < boundary; i++) {
    if (!reader_->has_next()) {
      break;
    }
    message.message = reader_->read_next();
    message.time_since_start =
      TimePoint(std::chrono::nanoseconds(message.message->time_stamp)) - time_first_message;

    message_queue_.enqueue(message);
  }
}

void Player::play_messages_from_queue()
{
  start_time_ = std::chrono::system_clock::now();
  do {
    play_messages_until_queue_empty();
    if (!is_storage_completely_loaded() && rclcpp::ok()) {
      ROSBAG2_TRANSPORT_LOG_WARN(
        "Message queue starved. Messages will be delayed. Consider "
        "increasing the --read-ahead-queue-size option.");
    }
  } while (!is_storage_completely_loaded() && rclcpp::ok());
}

//Dynamic Alignment as Fastrtps
unsigned long alignment(unsigned long data_size, unsigned long last_data_size, unsigned long current_position)
{
  return data_size > last_data_size ? (data_size - current_position % data_size) & (data_size-1):0;
}

//deal with string
void Player::deal_with_string(const uint8_t *dds_buffer, bool is_wstring)
{
  uint32_t length;
  size_t string_header = sizeof(uint32_t);//string header 4 Bytes

  unsigned long one_offset = alignment(string_header, last_data_size, current_position);
  current_position = current_position + one_offset;
  memcpy(&length, (dds_buffer + current_position + 4), string_header);

  if(!is_wstring)
    {
      last_data_size = sizeof(char);
      current_position += (string_header + length);
    }
  else {
      last_data_size = sizeof(uint32_t);
      current_position += (string_header + length * sizeof(uint32_t));
  }
}

//find out the real position of header in fastrtps serialized data
void Player::calculate_position_with_align(const uint8_t * dds_buffer_ptr, const rosidl_typesupport_introspection_cpp::MessageMember *message_member, unsigned long stop_index)
{
  unsigned long one_offset = 0;
  unsigned long data_size = 0;

  for (unsigned int i=0; i < stop_index; i++) {
    bool is_string = false;
    bool is_wstring = false;
    bool is_ros_msg_type = false;
    const rosidl_typesupport_introspection_cpp::MessageMembers * sub_members;
    //reference:https://github.com/ros2/rmw_fastrtps/blob/9438cca2a6a21b2436684607fea8a78624363f80/rmw_fastrtps_cpp/include/rmw_fastrtps_cpp/TypeSupport_impl.hpp#L525
    switch (message_member[i].type_id_) {
      case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOL:
        data_size = sizeof(bool);
        break;
      case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BYTE:
      case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8:
        data_size = sizeof (uint8_t);
        break;
      case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_CHAR:
      case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8:
        data_size = sizeof (char);
        break;
      case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT:
        data_size = sizeof (float);
        break;
      case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT64:
        data_size = sizeof (double);
        break;
      case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16:
        data_size = sizeof (int16_t);
        break;
      case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16:
        data_size = sizeof (uint16_t);
        break;
      case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32:
        data_size = sizeof (int32_t);
        break;
      case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32:
        data_size = sizeof (uint32_t);
        break;
      case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64:
        data_size = sizeof (int64_t);
        break;
      case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64:
        data_size = sizeof (uint64_t);
        break;
      case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_LONG_DOUBLE:
        data_size = 8;
        break;
      case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING:
        deal_with_string(dds_buffer_ptr, is_wstring);
        is_string = true;
        break;
      case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_WCHAR:
        data_size = sizeof(uint16_t);
        break;
      case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_WSTRING:
        deal_with_string(dds_buffer_ptr, is_wstring);
        is_wstring = true;
        break;
      case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE:
        sub_members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(message_member[i].members_->data);
        calculate_position_with_align(dds_buffer_ptr, sub_members->members_, sub_members->member_count_);
        is_ros_msg_type = true;
        break;
      }
    //standard element
    if(!is_string && !is_wstring && !is_ros_msg_type && !message_member[i].is_array_)
      {
        one_offset = alignment(data_size, last_data_size, current_position);
        current_position += (one_offset + data_size);
        last_data_size = data_size;
      }
    //standard array
    else if(!is_string && !is_wstring && !is_ros_msg_type && message_member[i].is_array_)
      {
        for (uint j = 0;j < message_member[i].array_size_; j++) {
          one_offset = alignment(data_size, last_data_size, current_position);
          current_position += (one_offset + data_size);
          last_data_size = data_size;
        }
      }
    //array of string
    else if (is_string && !is_wstring && !is_ros_msg_type && message_member[i].is_array_)
      {
        for (uint j = 0;j < message_member[i].array_size_ - 1; j++) {
          deal_with_string(dds_buffer_ptr, false);
        }
      }
    //array of wstring
    else if (is_wstring && !is_string && !is_ros_msg_type && message_member[i].is_array_)
      {
        for (uint j = 0;j < message_member[i].array_size_ - 1; j++) {
          deal_with_string(dds_buffer_ptr, true);
        }
      }
    else if (is_ros_msg_type && !is_string && !is_wstring && message_member[i].is_array_)
      {
        for (uint j = 0;j < message_member[i].array_size_ - 1; j++) {
          calculate_position_with_align(dds_buffer_ptr, sub_members->members_, sub_members->member_count_);
        }
      }
  }

}

void Player::play_messages_until_queue_empty()
{
  ReplayableMessage message;
  while (message_queue_.try_dequeue(message) && rclcpp::ok()) {
    std::this_thread::sleep_until(start_time_ + message.time_since_start);

    auto it = topics_ts_map_.find(message.message->topic_name);
    if (it != topics_ts_map_.end() ) {
      builtin_interfaces::msg::Time ros_time_to_set;

      std::chrono::time_point<std::chrono::high_resolution_clock> time_to_be_set = start_time_ + message.time_since_start;

      auto offset_index = it->second.stop_index;
      const rosidl_typesupport_introspection_cpp::MessageMember * msg_ptr = it->second.msg_member_ptr;

      std::chrono::duration_cast<std::chrono::nanoseconds>(time_to_be_set.time_since_epoch());

      ros_time_to_set.sec = static_cast<int32_t>(floor(time_to_be_set.time_since_epoch().count()/1e9));
      ros_time_to_set.nanosec = static_cast<uint32_t>(round(time_to_be_set.time_since_epoch().count() - ros_time_to_set.sec*1e9));

      //memcpy
      uint8_t * buffer_temp = message.message->serialized_data->buffer;
      dds_buffer_ptr = message.message->serialized_data->buffer;
      calculate_position_with_align(dds_buffer_ptr, msg_ptr, offset_index);
      size_t header_time_sec_size = sizeof (int32_t);
      unsigned long last_offset = alignment(header_time_sec_size, last_data_size, current_position);
      current_position += last_offset;
      buffer_temp = buffer_temp + current_position + 4; //plus dds header

      memcpy(buffer_temp, &ros_time_to_set, sizeof(builtin_interfaces::msg::Time));

      current_position = 0;
      last_data_size = ULONG_MAX;
    }

    if (rclcpp::ok()) {
      publishers_[message.message->topic_name]->publish(message.message->serialized_data);
    }
  }
}


void Player::prepare_topic_ts_map()
{
  auto topics = reader_->get_all_topics_and_types();

  //build the map
  for (const auto & topic : topics){
      auto type_support = rosbag2::get_typesupport(topic.type, "rosidl_typesupport_introspection_cpp");
      auto msg_members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(type_support->data);

      const rosidl_typesupport_introspection_cpp::MessageMember *msg_member_ptr = msg_members->members_;
      auto msg_member_count = msg_members->member_count_;
      for (unsigned int i = 0;i < msg_member_count;i++) {
          if(strcmp(msg_member_ptr[i].name_, "header") == 0)
          {
            header_support_struct header_support;
            header_support.stop_index = i;
            header_support.msg_member_ptr = msg_member_ptr;
            topics_ts_map_.insert(std::make_pair(topic.name, header_support));
            break;
          }
      }
  }

}

void Player::prepare_publishers()
{
  auto topics = reader_->get_all_topics_and_types();
  for (const auto & topic : topics) {
    publishers_.insert(
      std::make_pair(
        topic.name, rosbag2_transport_->create_generic_publisher(topic.name, topic.type)));
  }
}

}  // namespace rosbag2_transport
