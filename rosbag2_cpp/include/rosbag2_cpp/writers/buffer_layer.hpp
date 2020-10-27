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

#ifndef ROSBAG2_CPP__WRITERS__BUFFER_LAYER_HPP
#define ROSBAG2_CPP__WRITERS__BUFFER_LAYER_HPP

#include <memory>
#include <thread>
#include <mutex>
#include <atomic>

#include "rosbag2_cpp/visibility_control.hpp"
#include "rosbag2_storage/storage_interfaces/read_write_interface.hpp"
#include "rosbag2_cpp/storage_options.hpp"
#include "rosbag2_cpp/writers/circular_buffer.hpp"

namespace rosbag2_cpp
{
namespace writers
{

class ROSBAG2_CPP_PUBLIC BufferLayer
{
public:
  BufferLayer(
      std::shared_ptr<rosbag2_storage::storage_interfaces::ReadWriteInterface> storage,
      const rosbag2_cpp::StorageOptions & storage_options);
  ~BufferLayer();

  // Flush data into storage, and reset cache
  void reset_cache();
  void push(std::shared_ptr<const rosbag2_storage::SerializedBagMessage> msg);


private:
  using BagMessageCircBuffer =
    CircularBuffer<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>>;

  void swap_buffers();
  void consume_buffers();
  void close();

  std::shared_ptr<rosbag2_storage::storage_interfaces::ReadWriteInterface> storage_;

  // Double buffers
  std::shared_ptr<BagMessageCircBuffer> primary_message_queue_;
  uint64_t primary_buffer_size_;
  std::shared_ptr<BagMessageCircBuffer> secondary_message_queue_;
  uint64_t secondary_buffer_size_;
  std::shared_ptr<BagMessageCircBuffer> current_queue_;
  std::shared_ptr<BagMessageCircBuffer> writing_queue_;
  std::mutex buffer_mutex_;
  std::atomic_bool is_stop_issued_ {false};
  std::mutex stop_mutex_;
  std::thread consumer_thread_;

  // Max cache size in bytes and length limits for buffers
  uint64_t max_cache_size_;
  uint32_t max_cache_length_;

};

}  // namespace writers
}  // namespace rosbag2_cpp

#endif // ROSBAG2_CPP__WRITERS__BUFFER_LAYER_HPP
