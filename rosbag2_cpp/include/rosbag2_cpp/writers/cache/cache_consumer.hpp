// Copyright 2020, Robotec.ai sp. z o.o.
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

#ifndef ROSBAG2_CPP__WRITERS__CACHE__CACHE_CONSUMER_HPP_
#define ROSBAG2_CPP__WRITERS__CACHE__CACHE_CONSUMER_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <thread>

#include "rosbag2_cpp/writers/cache/message_cache.hpp"
#include "rosbag2_storage/storage_interfaces/read_write_interface.hpp"

namespace rosbag2_cpp
{
namespace writers
{
namespace cache
{
// This class is responsible for constantly writing the cache to storage
class ROSBAG2_CPP_PUBLIC CacheConsumer
{
public:
  CacheConsumer(
    std::shared_ptr<MessageCache> message_cache,
    std::shared_ptr<rosbag2_storage::storage_interfaces::ReadWriteInterface> storage);
  ~CacheConsumer();

  // shut down consumer thread
  void close();

  // Set new storage to write to, restart thread if necessary
  void change_storage(
    std::shared_ptr<rosbag2_storage::storage_interfaces::ReadWriteInterface> storage);

private:
  std::shared_ptr<MessageCache> message_cache_;
  std::shared_ptr<rosbag2_storage::storage_interfaces::ReadWriteInterface> storage_;

  // Write buffer data to a storage
  void exec_consuming();

  // Consumer thread shutdown sync
  std::atomic_bool is_stop_issued_ {false};
  std::mutex consumer_mutex_;

  std::thread consumer_thread_;
};

}  // namespace cache
}  // namespace writers
}  // namespace rosbag2_cpp

#endif  // ROSBAG2_CPP__WRITERS__CACHE__CACHE_CONSUMER_HPP_
