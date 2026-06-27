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

#include <iostream>
#include <memory>

#include "rosbag2_cpp/cache/cache_consumer.hpp"
#include "rosbag2_cpp/logging.hpp"

namespace rosbag2_cpp
{
namespace cache
{

CacheConsumer::CacheConsumer(
  std::shared_ptr<MessageCacheInterface> message_cache,
  consume_callback_function_t consume_callback)
: message_cache_(message_cache),
  consume_callback_(consume_callback)
{
  consumer_thread_ = std::thread(&CacheConsumer::exec_consuming, this);
}

CacheConsumer::~CacheConsumer()
{
  try {
    stop();
  } catch (...) {
    // Destructors must not let exceptions escape. Callers that need the original failure should
    // invoke stop() or throw_if_failed() explicitly before destruction.
  }
}

void CacheConsumer::stop()
{
  message_cache_->begin_flushing();
  is_stop_issued_ = true;

  ROSBAG2_CPP_LOG_INFO_STREAM(
    "Writing remaining messages from cache to the bag. It may take a while");

  if (consumer_thread_.joinable()) {
    consumer_thread_.join();
  }
  message_cache_->done_flushing();
  throw_if_failed();
}

void CacheConsumer::start()
{
  is_stop_issued_ = false;
  if (!consumer_thread_.joinable()) {
    consumer_thread_ = std::thread(&CacheConsumer::exec_consuming, this);
  }
}

void CacheConsumer::exec_consuming()
{
  bool exit_flag = false;
  bool flushing = false;
  while (!exit_flag) {
    message_cache_->wait_for_data();
    message_cache_->swap_buffers();
    // Get the current consumer buffer.
    auto consumer_buffer = message_cache_->get_consumer_buffer();
    try {
      consume_callback_(consumer_buffer->data());
    } catch (...) {
      store_exception(std::current_exception());
      consumer_buffer->clear();
      message_cache_->release_consumer_buffer();
      exit_flag = true;
      continue;
    }
    consumer_buffer->clear();
    message_cache_->release_consumer_buffer();

    if (flushing) {exit_flag = true;}  // this was the final run
    if (is_stop_issued_) {flushing = true;}  // run one final time to flush
  }
}

void CacheConsumer::throw_if_failed()
{
  std::exception_ptr consumer_exception;
  {
    std::lock_guard<std::mutex> lock(consumer_exception_mutex_);
    consumer_exception = consumer_exception_;
    consumer_exception_ = nullptr;
  }
  if (consumer_exception) {
    std::rethrow_exception(consumer_exception);
  }
}

void CacheConsumer::store_exception(std::exception_ptr exception)
{
  std::lock_guard<std::mutex> lock(consumer_exception_mutex_);
  if (!consumer_exception_) {
    consumer_exception_ = std::move(exception);
  }
}

}  // namespace cache
}  // namespace rosbag2_cpp
