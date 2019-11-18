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

#include <algorithm>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "rosbag2/info.hpp"
#include "rosbag2/storage_options.hpp"
#include "rosbag2/writer_interfaces/base_writer_interface.hpp"

#include "rosbag2_storage/filesystem_helper.hpp"

namespace rosbag2
{

Writer::Writer(std::unique_ptr<rosbag2::writer_interfaces::BaseWriterInterface> writer_impl)
: writer_impl_(std::move(writer_impl))
{}

Writer::~Writer()
{
  writer_impl_.reset();
}

void Writer::open(
  const StorageOptions & storage_options, const ConverterOptions & converter_options)
{
  writer_impl_->open(storage_options, converter_options);
}

void Writer::create_topic(const TopicMetadata & topic_with_type)
{
  writer_impl_->create_topic(topic_with_type);
}

void Writer::remove_topic(const TopicMetadata & topic_with_type)
{
  writer_impl_->remove_topic(topic_with_type);
}

void Writer::write(std::shared_ptr<SerializedBagMessage> message)
{
  writer_impl_->write(message);
}

}  // namespace rosbag2
