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

#ifndef ROSBAG2_STORAGE__TOPIC_METADATA_HPP_
#define ROSBAG2_STORAGE__TOPIC_METADATA_HPP_

#include <string>
#include <vector>
#include "rclcpp/qos.hpp"

namespace rosbag2_storage
{

struct TopicMetadata
{
  uint16_t id = 0;  // Topic id returned by storage
  std::string name;
  std::string type;
  std::string serialization_format;
  std::vector<rclcpp::QoS> offered_qos_profiles;
  // REP-2011 type description hash if available for topic, "" otherwise.
  std::string type_description_hash;

  bool operator==(const rosbag2_storage::TopicMetadata & rhs) const
  {
    return id == rhs.id &&
           name == rhs.name &&
           type == rhs.type &&
           serialization_format == rhs.serialization_format &&
           type_description_hash == rhs.type_description_hash;
  }
};

}  // namespace rosbag2_storage

#endif  // ROSBAG2_STORAGE__TOPIC_METADATA_HPP_
