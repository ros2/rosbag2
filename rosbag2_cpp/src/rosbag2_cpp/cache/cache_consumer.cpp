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
  std::shared_ptr<MessageCacheInterface> message_cache,
  consume_callback_function_t consume_callback)
: message_cache_(message_cache),
  consume_callback_(consume_callback)
{
  consumer_thread_ = std::thread(&CacheConsumer::exec_consuming, this);
}

CacheConsumer::~CacheConsumer()
{
  stop();
}

void CacheConsumer::stop()
{
  issue_stop().get();
}

std::future<void> CacheConsumer::stop_async()
{
  auto stop_future = issue_stop();
  return std::async(std::launch::async, [stop_future]() mutable {
             stop_future.get();
  });
}

void CacheConsumer::start()
{
  std::lock_guard<std::mutex> lock(start_stop_mutex_);
  if (shared_stop_future_.valid()) {
    shared_stop_future_.get();
    shared_stop_future_ = std::shared_future<void>();
  }

  is_stop_issued_ = false;
  if (!consumer_thread_.joinable()) {
    consumer_thread_ = std::thread(&CacheConsumer::exec_consuming, this);
  }
}

void CacheConsumer::flush_remaining_messages() const
{
  // Swap buffers one last time to make sure that all messages are flushed. This is necessary in
  // case stop is called while consumer_thread_ is processing the consumer buffer, which means that
  // the producer buffer may have some messages which has not yet dumped to the storage.
  message_cache_->swap_buffers();
  // Get the current consumer buffer.
  auto consumer_buffer = message_cache_->get_consumer_buffer();
  consume_callback_(consumer_buffer->data());
  consumer_buffer->clear();
  message_cache_->release_consumer_buffer();
}

std::shared_future<void> CacheConsumer::issue_stop()
{
  std::lock_guard<std::mutex> lock(start_stop_mutex_);
  if (shared_stop_future_.valid()) {
    return shared_stop_future_;
  }

  message_cache_->begin_flushing();
  is_stop_issued_ = true;

  ROSBAG2_CPP_LOG_INFO_STREAM(
    "Writing remaining messages from cache to the bag. It may take a while");

  shared_stop_future_ = std::async(std::launch::async, [this]() {
        if (consumer_thread_.joinable()) {
          consumer_thread_.join();
        }
        // Flush remaining messages. This is necessary in case stop is called while consumer_thread_
        // is processing the consumer buffer, which means that the producer buffer may have some
        // messages which has not yet dumped to the storage.
        flush_remaining_messages();
        message_cache_->done_flushing();
        ROSBAG2_CPP_LOG_INFO_STREAM("Finished writing remaining messages from cache to the bag.");
    }).share();
  return shared_stop_future_;
}

void CacheConsumer::exec_consuming() const
{
  while (!is_stop_issued_) {
    message_cache_->wait_for_data();
    // Note: We need to do swap and dump data even if stop is issued, to properly handle snapshot
    // mode case, which is implemented via stop call.
    message_cache_->swap_buffers();
    // Get the current consumer buffer.
    auto consumer_buffer = message_cache_->get_consumer_buffer();
    consume_callback_(consumer_buffer->data());
    consumer_buffer->clear();
    message_cache_->release_consumer_buffer();
  }
}

}  // namespace cache
}  // namespace rosbag2_cpp
