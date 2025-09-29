// Copyright 2025 Apex.AI, Inc.
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

#include <vector>
#include "rosbag2_transport/readers_manager.hpp"
#include "readers_manager_impl.hpp"

namespace rosbag2_transport
{

ReadersManager::ReadersManager(std::vector<reader_storage_options_pair_t> && reader_with_options)
: pimpl_(std::make_unique<ReadersManagerImpl>(std::move(reader_with_options)))
{}

ReadersManager::~ReadersManager()
{
  // Explicitly reset the pimpl_ to ensure the destructor of ReadersManagerImpl is called
  // only once.
  pimpl_.reset();
}

[[nodiscard]] std::vector<rosbag2_storage::StorageOptions>
ReadersManager::get_all_storage_options() const
{
  return pimpl_->get_all_storage_options();
}

[[nodiscard]] bool ReadersManager::has_next() const
{
  return pimpl_->has_next();
}

[[nodiscard]] std::shared_ptr<rosbag2_storage::SerializedBagMessage>
ReadersManager::get_next_message_in_chronological_order()
{
  return pimpl_->get_next_message_in_chronological_order();
}

void ReadersManager::seek(const rcutils_time_point_value_t & timestamp)
{
  pimpl_->seek(timestamp);
}

[[nodiscard]] rcutils_time_point_value_t ReadersManager::get_earliest_timestamp() const
{
  return pimpl_->get_earliest_timestamp();
}

[[nodiscard]] rcutils_time_point_value_t ReadersManager::get_latest_timestamp() const
{
  return pimpl_->get_latest_timestamp();
}

void ReadersManager::set_filter(const rosbag2_storage::StorageFilter & storage_filter)
{
  pimpl_->set_filter(storage_filter);
}

[[nodiscard]] std::vector<rosbag2_storage::TopicMetadata>
ReadersManager::get_all_topics_and_types() const
{
  return pimpl_->get_all_topics_and_types();
}

void ReadersManager::add_event_callbacks(rosbag2_cpp::bag_events::ReaderEventCallbacks & callbacks)
{
  pimpl_->add_event_callbacks(callbacks);
}

}  // namespace rosbag2_transport
