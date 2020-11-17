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

#include "rosbag2_cpp/cache/cache_consumer.hpp"
#include "rosbag2_cpp/logging.hpp"

namespace rosbag2_cpp
{
namespace cache
{

CacheConsumer::CacheConsumer(
  std::shared_ptr<MessageCache> message_cache,
  consume_callback_function_t consume_callback)
: message_cache_(message_cache),
  consume_callback_(consume_callback)
{
  consumer_thread_ = std::thread(&CacheConsumer::exec_consuming, this);
}

CacheConsumer::~CacheConsumer()
{
  close();
}

void CacheConsumer::close()
{
  ROSBAG2_CPP_LOG_INFO_STREAM(
    "Writing remaining messages from cache to the bag. It may take a while");
  is_stop_issued_ = true;

  message_cache_->allow_swap();

  if (consumer_thread_.joinable()) {
    consumer_thread_.join();
  }
}

void CacheConsumer::change_consume_callback(
  CacheConsumer::consume_callback_function_t consume_callback)
{
  std::lock_guard<std::mutex> consumer_lock(consumer_mutex_);
  consume_callback_ = consume_callback;
  if (!consumer_thread_.joinable()) {
    is_stop_issued_ = false;
    consumer_thread_ = std::thread(&CacheConsumer::exec_consuming, this);
  }
}

void CacheConsumer::exec_consuming()
{
  bool exit_flag = false;
  while (!exit_flag) {
    if (!is_stop_issued_) {
      // wait for any data in producer buffer to swap it with consumer buffer
      message_cache_->wait_for_swap();
    }

    // consume all the data from consumer buffer
    auto consumer_buffer = message_cache_->consumer_buffer();
    consume_callback_(consumer_buffer->data());
    consumer_buffer->clear();

    {
      std::lock_guard<std::mutex> consumer_lock(consumer_mutex_);
      if (is_stop_issued_) {exit_flag = true;}
    }
  }
}

}  // namespace cache
}  // namespace rosbag2_cpp
