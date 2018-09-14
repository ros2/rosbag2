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

rosbag2::SequentialReader::SequentialReader(
  const std::string & uri,
  const std::string & storage_identifier)
{
  reader_ = factory_.open_read_only(uri, storage_identifier);
  if (!reader_) {
    throw std::runtime_error("No storage could be initialized. Abort");
  }
}

rosbag2::SequentialReader::~SequentialReader()
{
  reader_.reset();  // Necessary to ensure that the writer is destroyed before the factory
}

bool rosbag2::SequentialReader::has_next()
{
  return reader_->has_next();
}

std::shared_ptr<rosbag2::SerializedBagMessage> rosbag2::SequentialReader::read_next()
{
  return reader_->read_next();
}

std::vector<rosbag2::TopicWithType> rosbag2::SequentialReader::get_all_topics_and_types()
{
  return reader_->get_all_topics_and_types();
}
