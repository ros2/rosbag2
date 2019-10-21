// Copyright 2018, Bosch Software Innovations GmbH.
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

#include <memory>
#include <utility>
#include <vector>

#include "rosbag2/info.hpp"
#include "rosbag2/sequential_reader.hpp"

#include "./impl/sequential_reader_impl.hpp"

namespace rosbag2
{

SequentialReader::SequentialReader(
  std::unique_ptr<rosbag2_storage::StorageFactoryInterface> storage_factory,
  std::shared_ptr<SerializationFormatConverterFactoryInterface> converter_factory)
: reader_impl_(std::make_unique<ReaderImpl>(
      std::move(storage_factory),
      std::move(converter_factory)))
{}

SequentialReader::~SequentialReader()
{
  reader_impl_->reset();
}

void SequentialReader::open(
  const StorageOptions & storage_options, const ConverterOptions & converter_options)
{
  reader_impl_->open(storage_options, converter_options);
}

bool SequentialReader::has_next()
{
  return reader_impl_->has_next();
}

std::shared_ptr<SerializedBagMessage> SequentialReader::read_next()
{
  return reader_impl_->read_next();
}

std::vector<TopicMetadata> SequentialReader::get_all_topics_and_types()
{
  return reader_impl_->get_all_topics_and_types();
}

}  // namespace rosbag2
