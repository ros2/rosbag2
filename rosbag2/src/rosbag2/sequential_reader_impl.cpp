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

#include "sequential_reader_impl.hpp"

#include <memory>
#include <string>
#include <vector>

#include "rosbag2_storage/metadata_io.hpp"

namespace rosbag2
{

SequentialReaderImpl::SequentialReaderImpl(const StorageOptions & options)
{
  storage_ = factory_.open_read_only(options.uri, options.storage_id);

  if (!storage_) {
    throw std::runtime_error("No storage could be initialized. Abort");
  }
}

SequentialReaderImpl::~SequentialReaderImpl()
{
  storage_.reset();  // Necessary to ensure that the writer is destroyed before the factory
}

bool SequentialReaderImpl::has_next()
{
  if (storage_) {
    return storage_->has_next();
  }
  throw std::runtime_error("Bag is not open. Call open() before reading.");
}

std::shared_ptr<SerializedBagMessage> SequentialReaderImpl::read_next()
{
  if (storage_) {
    return storage_->read_next();
  }
  throw std::runtime_error("Bag is not open. Call open() before reading.");
}

std::vector<TopicWithType> SequentialReaderImpl::get_all_topics_and_types()
{
  if (storage_) {
    return storage_->get_all_topics_and_types();
  }
  throw std::runtime_error("Bag is not open. Call open() before reading.");
}

}  // namespace rosbag2
