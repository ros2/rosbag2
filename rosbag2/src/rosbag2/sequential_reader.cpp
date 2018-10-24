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

#include "rosbag2/sequential_reader.hpp"

#include <memory>
#include <string>
#include <vector>

namespace rosbag2
{

SequentialReader::~SequentialReader()
{
  storage_.reset();  // Necessary to ensure that the writer is destroyed before the factory
}

void SequentialReader::open(const StorageOptions & options, const std::string & rmw_format)
{
  rmw_format_ = rmw_format;
  storage_ = factory_.open_read_only(options.uri, options.storage_id);
  if (!storage_) {
    throw std::runtime_error("No storage could be initialized. Abort");
  }
}

bool SequentialReader::has_next()
{
  if (storage_) {
    return storage_->has_next();
  }
  throw std::runtime_error("Bag is not open. Call open() before reading.");
}

std::shared_ptr<SerializedBagMessage> SequentialReader::read_next()
{
  if (storage_) {
    return storage_->read_next();
  }
  throw std::runtime_error("Bag is not open. Call open() before reading.");
}

std::vector<TopicWithType> SequentialReader::get_all_topics_and_types()
{
  if (storage_) {
    return storage_->get_all_topics_and_types();
  }
  throw std::runtime_error("Bag is not open. Call open() before reading.");
}

}  // namespace rosbag2
