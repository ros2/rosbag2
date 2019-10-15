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

#include "rosbag2/info.hpp"
#include "rosbag2/sequential_reader.hpp"

#include <cassert>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace rosbag2
{

SequentialReader::SequentialReader(
  std::unique_ptr<rosbag2_storage::StorageFactoryInterface> storage_factory,
  std::shared_ptr<SerializationFormatConverterFactoryInterface> converter_factory)
: reader_(std::make_shared<rosbag2::Reader>(std::move(storage_factory), std::move(converter_factory)))
{}

void SequentialReader::open(
  const StorageOptions & storage_options, const ConverterOptions & converter_options)
{
  storage_options_ = storage_options;
  reader_->open(storage_options, converter_options);
  const auto bag_metadata = reader_->get_metadata();
  file_paths_ = bag_metadata.relative_file_paths;
  ROSBAG2_LOG_DEBUG("Sequential reader opened.");
}

bool SequentialReader::has_next_file() const
{
  assert(!file_paths_.empty());
  return current_file_offset_ + 1 < file_paths_.size();
}

std::string SequentialReader::get_next_file()
{
  assert(current_file_offset_ < file_paths_.size());
  current_file_offset_++;
  ROSBAG2_LOG_DEBUG_STREAM("New file: " << file_paths_.at(current_file_offset_));
  return file_paths_.at(current_file_offset_);
}

bool SequentialReader::has_next()
{
  if (!reader_->has_next()) {
    if (has_next_file()) {
      std::string current_file = get_next_file();
      reader_->open_read_only(current_file, storage_options_.storage_id);
    }
  }
  return reader_->has_next();
}

std::shared_ptr<SerializedBagMessage> SequentialReader::read_next()
{
  return reader_->read_next();
}

std::vector<TopicMetadata> SequentialReader::get_all_topics_and_types()
{
  return reader_->get_all_topics_and_types();
}

}  // namespace rosbag2
