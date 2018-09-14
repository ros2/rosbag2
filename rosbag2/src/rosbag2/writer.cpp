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

#include "rosbag2/writer.hpp"

#include <memory>
#include <string>

namespace rosbag2
{

Writer::Writer(std::string uri, std::string storage_identifier)
{
  writer_ = factory_.open_read_write(uri, storage_identifier);
  if (!writer_) {
    throw std::runtime_error("No storage could be initialized. Abort");
  }
}

Writer::~Writer()
{
  writer_.reset();  // Necessary to ensure that the writer is destroyed before the factory
}

void Writer::create_topic(const TopicWithType & topic_with_type)
{
  writer_->create_topic(topic_with_type);
}

void Writer::write(std::shared_ptr<SerializedBagMessage> message)
{
  writer_->write(message);
}

}  // namespace rosbag2
