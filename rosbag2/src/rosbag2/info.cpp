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

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rosbag2_storage/metadata_io.hpp"

namespace rosbag2
{

Info::Info(std::shared_ptr<rosbag2_storage::Rosbag2StorageFactory> storage_factory)
: storage_factory_(std::move(storage_factory)), formatter_(std::make_unique<Formatter>())
{}

rosbag2::BagMetadata Info::read_metadata(const std::string & uri)
{
  return storage_factory_->metadata_io()->read_metadata(uri);
}

std::map<std::string, std::string> Info::format_duration(
  std::chrono::high_resolution_clock::duration duration)
{
  return formatter_->format_duration(duration);
}

std::string Info::format_time_point(std::chrono::high_resolution_clock::duration duration)
{
  return formatter_->format_time_point(duration);
}

std::string Info::format_file_size(size_t file_size)
{
  return formatter_->format_file_size(file_size);
}

void Info::format_file_paths(std::vector<std::string> paths, std::stringstream & info_stream)
{
  formatter_->format_file_paths(paths, info_stream);
}

void Info::format_topics_with_type(
  std::vector<rosbag2::TopicMetadata> topics, std::stringstream & info_stream)
{
  formatter_->format_topics_with_type(topics, info_stream);
}

}  // namespace rosbag2
