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

#ifndef ROSBAG2_CPP__WRITERS__BUFFER_LAYER_HPP_
#define ROSBAG2_CPP__WRITERS__BUFFER_LAYER_HPP_
#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "rosbag2_storage/storage_interfaces/read_write_interface.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "rosbag2_cpp/visibility_control.hpp"

namespace rosbag2_cpp
{
namespace writers
{

class ROSBAG2_CPP_PUBLIC BufferLayer
{
public:
  BufferLayer(
    std::shared_ptr<rosbag2_storage::storage_interfaces::ReadWriteInterface> storage,
    const uint64_t & max_buffer_size);
  ~BufferLayer();

  // Flush data into storage, and reset cache
  void reset_cache();
  // Push data into primary buffer
  bool push(std::shared_ptr<const rosbag2_storage::SerializedBagMessage> msg);
  // Flush buffers and reset them, shut down consumer thread
  void close();
  // Starts consumer thread if one is not working
  void start_consumer();
  // Sets new storage
  void set_storage(
    std::shared_ptr<rosbag2_storage::storage_interfaces::ReadWriteInterface> storage);

private:
  class BagMessageBuffer
  {
public:
    explicit BagMessageBuffer(const uint64_t max_cache_size)
    : max_bytes_size_(max_cache_size) {}

    // If buffer size got some space left, we push message regardless of its size, but if
    // this results in exceeding buffer size, we mark buffer to drop all new incoming messages.
    // This flag is cleared when buffers are swapped.
    bool push(const std::shared_ptr<const rosbag2_storage::SerializedBagMessage> & msg)
    {
      bool pushed = false;
      if (!drop_messages_) {
        buffer_bytes_size_ += msg->serialized_data->buffer_length;
        buffer_.push_back(msg);
        pushed = true;
      }

      if (buffer_bytes_size_ >= max_bytes_size_) {
        drop_messages_ = true;
      }
      return pushed;
    }

    // Clears buffer and allow it to receive messages again
    void clear()
    {
      buffer_.clear();
      buffer_bytes_size_ = 0u;
      drop_messages_ = false;
    }

    // Get number of elements in the buffer
    size_t size() {return buffer_.size();}

    // Get buffer data
    std::vector<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>> & data()
    {
      return buffer_;
    }

private:
    std::vector<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>> buffer_;
    uint64_t buffer_bytes_size_ {0u};  // in bytes
    const uint64_t max_bytes_size_;  // in bytes
    bool drop_messages_ {false};
  };

  // Write secondary buffer data to a storage
  void exec_consuming();

  // Storage handler
  std::shared_ptr<rosbag2_storage::storage_interfaces::ReadWriteInterface> storage_;

  // Double buffers
  std::shared_ptr<BagMessageBuffer> primary_buffer_;
  std::shared_ptr<BagMessageBuffer> secondary_buffer_;

  // Total number of dropped messages
  uint32_t elements_dropped_ = {0u};

  // Double buffers sync
  std::condition_variable buffers_condition_var_;
  std::mutex buffer_mutex_;

  // Consumer thread shutdown sync
  std::atomic_bool is_stop_issued_ {false};
  std::mutex stop_mutex_;

  // Thread for writing secondary buffer to a storage
  bool is_consumer_needed_;
  std::thread consumer_thread_;
};

}  // namespace writers
}  // namespace rosbag2_cpp

#endif  // ROSBAG2_CPP__WRITERS__BUFFER_LAYER_HPP_
