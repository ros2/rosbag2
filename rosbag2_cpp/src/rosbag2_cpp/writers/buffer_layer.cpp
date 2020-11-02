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
#include <memory>
#include <utility>
#include <vector>

#include "rosbag2_cpp/writers/buffer_layer.hpp"

#include "rosbag2_cpp/logging.hpp"

namespace rosbag2_cpp
{
namespace writers
{

BufferLayer::BufferLayer(
  std::shared_ptr<rosbag2_storage::storage_interfaces::ReadWriteInterface> storage,
  const rosbag2_cpp::StorageOptions & storage_options)
: storage_(storage), max_cache_size_(storage_options.max_cache_size)
{
  // Set buffers
  primary_buffer_ = std::shared_ptr<BagMessageBuffer>(
    new BagMessageBuffer());
  secondary_buffer_ =
    std::shared_ptr<BagMessageBuffer>(new BagMessageBuffer());

  // Run consumer thread. We need this only when max cache size is not zero. Otherwise direct
  // storage write call is used.
  if (max_cache_size_ > 0u) {
    consumer_thread_ = std::thread(&BufferLayer::exec_consuming, this);
  }
}

bool BufferLayer::push(std::shared_ptr<const rosbag2_storage::SerializedBagMessage> msg)
{
  // If cache size is set to zero, we directly call write
  if (max_cache_size_ == 0u) {
    storage_->write(msg);
    return true;
  }

  bool pushed = false;
  // If buffer size got some space left, we push message regardless of its size, but if
  // this results in exceeding buffer size, we mark buffer to drop all new incoming messages.
  // This flag is cleared when buffers are swapped.
  {
    std::lock_guard<std::mutex> buffer_lock(buffer_mutex_);
    if (!drop_messages_) {
      primary_buffer_size_ += msg->serialized_data->buffer_length;
      primary_buffer_->push_back(msg);
      pushed = true;
    } else {
      elements_dropped_++;
    }

    if (primary_buffer_size_ >= max_cache_size_) {
      drop_messages_ = true;
    }
  }
  buffers_condition_var_.notify_one();
  return pushed;
}

void BufferLayer::swap_buffers()
{
  {
    std::swap(primary_buffer_, secondary_buffer_);
    std::swap(primary_buffer_size_, secondary_buffer_size_);
    drop_messages_ = false;
  }
}

void BufferLayer::exec_consuming()
{
  bool exit_flag = false;
  while (true) {
    {
      if (!is_stop_issued_) {
        std::unique_lock<std::mutex> lock(buffer_mutex_);
        buffers_condition_var_.wait(lock);
      }
      swap_buffers();
    }
    buffers_condition_var_.notify_one();

    secondary_buffer_size_ = 0u;
    storage_->write(*secondary_buffer_.get());
    secondary_buffer_->clear();

    {
      std::lock_guard<std::mutex> writer_lock(stop_mutex_);
      if (exit_flag) {return;}
      if (is_stop_issued_) {exit_flag = true;}
    }
  }
}

void BufferLayer::close()
{
  {
    std::lock_guard<std::mutex> writer_lock(stop_mutex_);
    ROSBAG2_CPP_LOG_INFO_STREAM(
      "Writing remaining " <<
        primary_buffer_->size() + secondary_buffer_->size() <<
        " messages from buffers to the bag. It may take a while...");
    is_stop_issued_ = true;
  }

  buffers_condition_var_.notify_one();

  if (consumer_thread_.joinable()) {
    consumer_thread_.join();
  }

  if (elements_dropped_) {
    ROSBAG2_CPP_LOG_WARN_STREAM(
      "Done writing! Total missed messages: " <<
        elements_dropped_ <<
        " where " <<
        primary_buffer_->size() + secondary_buffer_->size() <<
        " left in buffers.");
  }
}

void BufferLayer::reset_cache()
{
  std::lock_guard<std::mutex> buffer_lock(buffer_mutex_);
  if (!primary_buffer_->empty()) {
    primary_buffer_->clear();
    primary_buffer_size_ = 0u;
  }
  if (!secondary_buffer_->empty()) {
    secondary_buffer_->clear();
    secondary_buffer_size_ = 0u;
  }
}

BufferLayer::~BufferLayer()
{
  close();
}

}  // namespace writers
}  // namespace rosbag2_cpp
