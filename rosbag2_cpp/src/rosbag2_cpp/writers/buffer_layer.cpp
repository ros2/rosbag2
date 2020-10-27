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

#include "rosbag2_cpp/writers/buffer_layer.hpp"

#include "rosbag2_cpp/logging.hpp"

namespace rosbag2_cpp
{
namespace writers
{

BufferLayer::BufferLayer(
    std::shared_ptr<rosbag2_storage::storage_interfaces::ReadWriteInterface> storage,
    const rosbag2_cpp::StorageOptions &storage_options) :
  storage_(storage)
{
  max_cache_size_ = storage_options.max_cache_size;
  max_cache_length_ = 10000;  // TODO(piotr.jaroszek) move to storage opts

  // Set buffers
  primary_message_queue_ = std::shared_ptr<BagMessageCircBuffer>(
    new BagMessageCircBuffer(
      max_cache_length_));
  secondary_message_queue_ =
    std::shared_ptr<BagMessageCircBuffer>(new BagMessageCircBuffer(max_cache_length_));
  current_queue_ = primary_message_queue_;
  writing_queue_ = secondary_message_queue_;

  // Run consumer thread
  consumer_thread_ = std::thread(&BufferLayer::consume_buffers, this);
}

void BufferLayer::push(std::shared_ptr<const rosbag2_storage::SerializedBagMessage> msg)
{
  std::lock_guard<std::mutex> buffer_lock(buffer_mutex_);

  uint64_t current_buffer_size {0u};

  // Count size for both buffers
  if (current_queue_ == primary_message_queue_) {
    primary_buffer_size_ += msg->serialized_data->buffer_length;
    current_buffer_size = primary_buffer_size_;
  } else {
    secondary_buffer_size_ += msg->serialized_data->buffer_length;
    current_buffer_size = secondary_buffer_size_;
  }

  // Just skip enqueue message if buffer size is reached, else just register failed writing
  if (current_buffer_size < max_cache_size_ || max_cache_size_ == 0u) {
    // Automatically takes buffer length limit into accout
    if (!current_queue_->enqueue(msg)) {
      ROSBAG2_CPP_LOG_DEBUG_STREAM(
        "Message on '" <<
        msg->topic_name <<
        "' dropped. Failed with buffer length limit." <<
        std::flush);
    }
  } else {
    ROSBAG2_CPP_LOG_DEBUG_STREAM(
      "Message on '" <<
      msg->topic_name <<
      "' dropped. Failed with buffer size limit." <<
      std::flush);
    current_queue_->increment_failed_counter();
  }
}

void BufferLayer::swap_buffers()
{
  {
    std::lock_guard<std::mutex> buffer_lock(buffer_mutex_);
    std::swap(writing_queue_, current_queue_);
  }
}

void BufferLayer::consume_buffers()
{
  bool exit_flag = false;
  while (true) {
    swap_buffers();

    if (writing_queue_->is_empty()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::vector<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>> transaction_vector;
    while (!writing_queue_->is_empty()) {
      auto message = writing_queue_->front();
      writing_queue_->dequeue();
      transaction_vector.push_back(message);
      if(writing_queue_ == primary_message_queue_) {
        primary_buffer_size_ = 0u;
      } else {
        secondary_buffer_size_ = 0u;
      }
    }

    primary_buffer_size_ = 0u;
    storage_->write(transaction_vector);

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
        current_queue_->elements_num() + writing_queue_->elements_num() <<
        " messages from buffers to the bag. It may take a while...");
    is_stop_issued_ = true;
  }

  if (consumer_thread_.joinable()) {
    consumer_thread_.join();
  }

  unsigned int missed_messages = current_queue_->failed_counter() +
    writing_queue_->failed_counter() +
    current_queue_->elements_num() +
    writing_queue_->elements_num();
  ROSBAG2_CPP_LOG_INFO_STREAM(
    "Done writing! Total missed messages: " <<
      missed_messages << " where " << current_queue_->elements_num() + writing_queue_->elements_num() << " left in buffers.");
}

void BufferLayer::reset_cache()
{
  if (!primary_message_queue_->is_empty()) {
    primary_message_queue_->clear();
    primary_buffer_size_ = 0u;
  }
  if (!secondary_message_queue_->is_empty()) {
    secondary_message_queue_->clear();
    secondary_buffer_size_ = 0u;
  }
}

BufferLayer::~BufferLayer()
{
  close();
  reset_cache();
}

}  // writers
}  // rosbag2_cpp
