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

#include "rosbag2/types/ros2_message.hpp"

#include <memory>
#include <string>
#include <vector>

#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"

namespace rosbag2
{

std::shared_ptr<rosbag2_ros2_message_t>
allocate_ros2_message(
  const rosidl_message_type_support_t * introspection_ts, const rcutils_allocator_t * allocator)
{
  auto intro_ts_members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(
    introspection_ts->data);
  auto raw_ros2_message = new rosbag2_ros2_message_t();
  raw_ros2_message->allocator = *allocator;
  raw_ros2_message->message = raw_ros2_message->allocator.zero_allocate(
    1, intro_ts_members->size_of_, raw_ros2_message->allocator.state);
  // TODO(Martin-Idel-SI): Use custom allocator to make sure all memory is obtained that way
  allocate_internal_types(raw_ros2_message->message, intro_ts_members);

  auto deleter = [intro_ts_members](rosbag2_ros2_message_t * msg) {
      deallocate_ros2_message(msg, intro_ts_members);
    };
  return std::shared_ptr<rosbag2_ros2_message_t>(raw_ros2_message, deleter);
}

void deallocate_ros2_message(
  rosbag2_ros2_message_t * msg,
  const rosidl_typesupport_introspection_cpp::MessageMembers * members)
{
  deallocate_internal_types(msg->message, members);
  msg->allocator.deallocate(msg->message, msg->allocator.state);
  delete msg;
}

void deallocate_internal_types(
  void * msg, const rosidl_typesupport_introspection_cpp::MessageMembers * members)
{
  for (size_t i = 0; i < members->member_count_; ++i) {
    auto member = members->members_[i];
    void * message_member = static_cast<uint8_t *>(msg) + member.offset_;

    if ((member.is_array_ && member.array_size_ == 0) || member.is_upper_bound_) {
      cleanup_vector(message_member, member);
    } else if (member.is_array_ && member.array_size_ > 0) {
      cleanup_array(message_member, member);
    } else {
      cleanup_element(message_member, member);
    }
  }
}

void cleanup_element(
  void * data, const rosidl_typesupport_introspection_cpp::MessageMember & member)
{
  if (member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING) {
    std::string empty;
    static_cast<std::string *>(data)->swap(empty);
  } else if (member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE) {
    deallocate_internal_types(
      data,
      static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(
        member.members_->data));
  }
}

void cleanup_array(
  void * data, const rosidl_typesupport_introspection_cpp::MessageMember & member)
{
  if (member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING) {
    for (size_t i = 0; i < member.array_size_; ++i) {
      std::string empty;
      auto * element = static_cast<std::string *>(member.get_function(data, i));
      element->swap(empty);
    }
  } else if (member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE) {
    auto nested_ts =
      static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(
      member.members_->data);
    for (size_t j = 0; j < member.array_size_; ++j) {
      auto nested_member = static_cast<uint8_t *>(data) + j * nested_ts->size_of_;
      deallocate_internal_types(nested_member, nested_ts);
    }
  }
}

void cleanup_vector(
  void * data, const rosidl_typesupport_introspection_cpp::MessageMember & member)
{
  switch (member.type_id_) {
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOL: {
        auto data_vector = static_cast<std::vector<bool> *>(data);
        std::vector<bool> empty;
        data_vector->swap(empty);
        break;
      }
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_BYTE: {
        auto data_vector = static_cast<std::vector<uint8_t> *>(data);
        std::vector<uint8_t> empty;
        data_vector->swap(empty);
        break;
      }
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_CHAR: {
        auto data_vector = static_cast<std::vector<char> *>(data);
        std::vector<char> empty;
        data_vector->swap(empty);
        break;
      }
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT32: {
        auto data_vector = static_cast<std::vector<float> *>(data);
        std::vector<float> empty;
        data_vector->swap(empty);
        break;
      }
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT64: {
        auto data_vector = static_cast<std::vector<double> *>(data);
        std::vector<double> empty;
        data_vector->swap(empty);
        break;
      }
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8: {
        auto data_vector = static_cast<std::vector<int8_t> *>(data);
        std::vector<int8_t> empty;
        data_vector->swap(empty);
        break;
      }
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8: {
        auto data_vector = static_cast<std::vector<uint8_t> *>(data);
        std::vector<uint8_t> empty;
        data_vector->swap(empty);
        break;
      }
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16: {
        auto data_vector = static_cast<std::vector<int16_t> *>(data);
        std::vector<int16_t> empty;
        data_vector->swap(empty);
        break;
      }
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16: {
        auto data_vector = static_cast<std::vector<uint16_t> *>(data);
        std::vector<uint16_t> empty;
        data_vector->swap(empty);
        break;
      }
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32: {
        auto data_vector = static_cast<std::vector<int32_t> *>(data);
        std::vector<int32_t> empty;
        data_vector->swap(empty);
        break;
      }
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32: {
        auto data_vector = static_cast<std::vector<uint32_t> *>(data);
        std::vector<uint32_t> empty;
        data_vector->swap(empty);
        break;
      }
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64: {
        auto data_vector = static_cast<std::vector<int64_t> *>(data);
        std::vector<int64_t> empty;
        data_vector->swap(empty);
        break;
      }
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64: {
        auto data_vector = static_cast<std::vector<uint64_t> *>(data);
        std::vector<uint64_t> empty;
        data_vector->swap(empty);
        break;
      }
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING: {
        auto data_vector = static_cast<std::vector<std::string> *>(data);
        std::vector<std::string> empty;
        data_vector->swap(empty);
        break;
      }
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE: {
        auto data_vector = static_cast<std::vector<uint8_t> *>(data);

        auto nested_ts =
          static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(
          member.members_->data);
        auto size = member.size_function(data);

        for (size_t j = 0; j < size; ++j) {
          auto nested_member = data_vector->data() + j * nested_ts->size_of_;
          deallocate_internal_types(nested_member, nested_ts);
        }

        data_vector->resize(0);
        data_vector->shrink_to_fit();
        delete data_vector->data();
        break;
      }
  }
}

void allocate_internal_types(
  void * msg, const rosidl_typesupport_introspection_cpp::MessageMembers * members)
{
  for (size_t i = 0; i < members->member_count_; ++i) {
    auto member = members->members_[i];
    void * message_member = static_cast<uint8_t *>(msg) + member.offset_;

    if ((member.is_array_ && member.array_size_ == 0) || member.is_upper_bound_) {
      allocate_vector(message_member, member);
    } else if (member.is_array_ && member.array_size_ > 0) {
      allocate_array(message_member, member);
    } else {
      allocate_element(message_member, member);
    }
  }
}

void allocate_vector(
  void * data, const rosidl_typesupport_introspection_cpp::MessageMember & member)
{
  // This is necessary because initialization otherwise fails for MSVC++ compiled builds
  if (member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOL) {
    new (data) std::vector<bool>();
  } else {
    // Properly initialize the zero allocated memory with a vector of size 0.
    new (data) std::vector<uint8_t>();
  }
}

void allocate_array(
  void * data, const rosidl_typesupport_introspection_cpp::MessageMember & member)
{
  if (member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING) {
    for (size_t i = 0; i < member.array_size_; ++i) {
      auto * element = static_cast<std::string *>(member.get_function(data, i));
      // This is necessary because initialization of empty strings fails for g++ compiled builds
      new (element) std::string("");
    }
  } else if (member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE) {
    auto nested_ts = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(
      member.members_->data);
    for (size_t j = 0; j < member.array_size_; ++j) {
      auto nested_member = static_cast<uint8_t *>(data) + j * nested_ts->size_of_;
      allocate_internal_types(nested_member, nested_ts);
    }
  }
}

void allocate_element(
  void * data, const rosidl_typesupport_introspection_cpp::MessageMember & member)
{
  if (member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING) {
    // This is necessary because initialization of empty strings fails for g++ compiled builds
    new (data) std::string("");
  } else if (member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE) {
    allocate_internal_types(
      data,
      static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(
        member.members_->data));
  }
}

}  // namespace rosbag2
