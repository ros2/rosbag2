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

#include "writer_impl.hpp"

#include <memory>
#include <string>

#include "rosbag2_storage/metadata_io.hpp"
#include "rosbag2/storage_options.hpp"

namespace rosbag2
{

WriterImpl::~WriterImpl()
{
  rosbag2_storage::write_metadata(options_.uri + "metadata.yaml", storage_->get_metadata());
  storage_.reset();  // Necessary to ensure that the writer is destroyed before the factory
}

void WriterImpl::open(const StorageOptions & options)
{
  storage_ = factory_.open_read_write(options.uri, options.storage_id);
  if (!storage_) {
    throw std::runtime_error("No storage could be initialized. Abort");
  }
  options_ = options;
}

void WriterImpl::create_topic(const TopicWithType & topic_with_type)
{
  if (!storage_) {
    throw std::runtime_error("Bag is not open. Call open() before writing.");
  }

  storage_->create_topic(topic_with_type);
}

void WriterImpl::write(std::shared_ptr<SerializedBagMessage> message)
{
  if (!storage_) {
    throw std::runtime_error("Bag is not open. Call open() before writing.");
  }

  storage_->write(message);
}

}  // namespace rosbag2
