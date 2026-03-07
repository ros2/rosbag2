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
  message_cache_->begin_flushing();
  is_stop_issued_ = true;

  ROSBAG2_CPP_LOG_INFO_STREAM(
    "Writing remaining messages from cache to the bag. It may take a while");

  if (consumer_thread_.joinable()) {
    consumer_thread_.join();
  }
  message_cache_->done_flushing();
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

std::future<void> CacheConsumer::stop_async()
{
  // TODO(morlov): Handle the case when stop_async is called multiple times before the consumer
  //  thread is stopped. In this case, we may end up with multiple threads trying to flush the cache
  //  and write to storage at the same time, which can cause issues. We can handle this by keeping
  //  track of whether a stop has already been issued and in progress and block until the
  //  previous stop will finish.
  //  However, need to take in to account that non-async stop can be called while async stop is
  //  still flushing the cache, or wise versa, so need to make sure that these two functions are
  //  properly synchronized. Perhaps the best way to handle this is to have a single stop function
  //  which will be called by both async and non-async versions, and this function will handle the
  //  synchronization and flushing of the cache.
  message_cache_->begin_flushing();
  is_stop_issued_ = true;

  ROSBAG2_CPP_LOG_INFO_STREAM(
    "Writing remaining messages from cache to the bag. It may take a while");

  std::future<void> stop_future =
    std::async(std::launch::async, [this]()
      {
        if (consumer_thread_.joinable()) {
          consumer_thread_.join();
        }
        message_cache_->done_flushing();
        // Swap buffers one last time to make sure that all messages are flushed. This is necessary
        // in case stop is called while consumer_thread_ is processing the consumer buffer, which
        // means that the producer buffer may have some messages which has not yet dumped to the
        // storage.
        message_cache_->swap_buffers();
        // Get the current consumer buffer.
        auto consumer_buffer = message_cache_->get_consumer_buffer();
        consume_callback_(consumer_buffer->data());
        consumer_buffer->clear();
        message_cache_->release_consumer_buffer();
      }
    );
  return stop_future;
}

void CacheConsumer::start()
{
  is_stop_issued_ = false;
  if (!consumer_thread_.joinable()) {
    consumer_thread_ = std::thread(&CacheConsumer::exec_consuming, this);
  }
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
