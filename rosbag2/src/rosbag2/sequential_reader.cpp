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
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rosbag2/info.hpp"

namespace rosbag2
{

SequentialReader::SequentialReader(
  std::unique_ptr<rosbag2_storage::StorageFactoryInterface> storage_factory,
  std::shared_ptr<SerializationFormatConverterFactoryInterface> converter_factory)
: reader_pimpl_(std::make_unique<ReaderImpl>(
      std::move(storage_factory),
      std::move(converter_factory)))
{}

SequentialReader::~SequentialReader()
{
  storage_.reset();  // Necessary to ensure that the storage is destroyed before the factory
}

void
SequentialReader::open(
  const StorageOptions & storage_options, const ConverterOptions & converter_options)
{
  reader_pimpl_->open(storage_options, converter_options);
}

bool SequentialReader::has_next()
{
  return reader_pimpl_->has_next();
}

std::shared_ptr<SerializedBagMessage> SequentialReader::read_next()
{
  return reader_pimpl_->read_next();
}

std::vector<TopicMetadata> SequentialReader::get_all_topics_and_types()
{
  return reader_pimpl_->get_all_topics_and_types();
}

}  // namespace rosbag2
