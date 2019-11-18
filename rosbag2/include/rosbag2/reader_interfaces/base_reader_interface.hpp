// Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

#ifndef ROSBAG2__READER_INTERFACES__BASE_READER_INTERFACE_HPP_
#define ROSBAG2__READER_INTERFACES__BASE_READER_INTERFACE_HPP_

#include <memory>
#include <vector>

#include "rosbag2_storage/serialized_bag_message.hpp"
#include "rosbag2_storage/topic_metadata.hpp"

#include "rosbag2/converter_options.hpp"
#include "rosbag2/storage_options.hpp"
#include "rosbag2/visibility_control.hpp"

namespace rosbag2
{
namespace reader_interfaces
{

class ROSBAG2_PUBLIC BaseReaderInterface
{
public:
  virtual ~BaseReaderInterface() {}

  virtual void open(
    const StorageOptions & storage_options, const ConverterOptions & converter_options) = 0;

  virtual void reset() = 0;

  virtual bool has_next() = 0;

  virtual std::shared_ptr<rosbag2_storage::SerializedBagMessage> read_next() = 0;

  virtual std::vector<rosbag2_storage::TopicMetadata> get_all_topics_and_types() = 0;
};

}  // namespace reader_interfaces
}  // namespace rosbag2

#endif  // ROSBAG2__READER_INTERFACES__BASE_READER_INTERFACE_HPP_
