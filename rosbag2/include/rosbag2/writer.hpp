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

#ifndef ROSBAG2__WRITER_HPP_
#define ROSBAG2__WRITER_HPP_

#include <memory>
#include <string>

#include "rosbag2_storage/storage_factory.hpp"
#include "rosbag2_storage/storage_interfaces/read_write_interface.hpp"
#include "rosbag2/types.hpp"
#include "rosbag2/visibility_control.hpp"

namespace rosbag2
{

class Writer
{
public:
  ROSBAG2_PUBLIC Writer(std::string uri, std::string storage_identifier);

  ROSBAG2_PUBLIC ~Writer();

  ROSBAG2_PUBLIC
  void create_topic(const TopicWithType & topic_with_type);

  ROSBAG2_PUBLIC
  void write(std::shared_ptr<SerializedBagMessage> message);

private:
  rosbag2_storage::StorageFactory factory_;
  std::shared_ptr<rosbag2_storage::storage_interfaces::ReadWriteInterface> writer_;
};

}  // namespace rosbag2

#endif  // ROSBAG2__WRITER_HPP_
