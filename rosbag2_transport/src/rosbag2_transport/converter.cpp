// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#include "converter.hpp"

#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_transport/logging.hpp"

namespace rosbag2_transport
{
Converter::Converter(
  std::shared_ptr<rosbag2_cpp::Reader> reader,
  std::shared_ptr<rosbag2_cpp::Writer> writer)
: reader_(std::move(reader)), writer_(std::move(writer)) {}

void Converter::convert(std::string serialization_format)
{
  for (auto topic : reader_->get_all_topics_and_types()) {
    topic.serialization_format = serialization_format;
    writer_->create_topic(topic);
  }
  while (reader_->has_next()) {
    auto msg = reader_->read_next();
    writer_->write(msg);
  }
}

}  // namespace rosbag2_transport
