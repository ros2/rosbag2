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

#ifndef ROSBAG2_CPP__CACHE__CACHE_CONSUMER_HPP_
#define ROSBAG2_CPP__CACHE__CACHE_CONSUMER_HPP_

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "rosbag2_cpp/cache/message_cache.hpp"
#include "rosbag2_cpp/cache/message_cache_interface.hpp"

// This is necessary because of using stl types here. It is completely safe, because
// a) the member is not accessible from the outside
// b) there are no inline functions.
#ifdef _WIN32
# pragma warning(push)
# pragma warning(disable:4251)
#endif

namespace rosbag2_cpp
{
namespace cache
{
/**
* This class is responsible for consuming the cache using provided fuction.
* It can work with any callback conforming to the consume_callback_function_t
* signature, e.g. a storage write function. Consuming and thus the callback are
* called in a separate thread.
*
* Since the consuming callback likely involves disk operations, the main motivation
* for design is to make sure that consumer is busy anytime there is any work.
* This is realized through conditional variable and greedy buffer switching.
*
* The consumer uses MessageCache and waits for consumer buffer to be ready. This
* will happen as soon as that there are any messages put into producer buffer -
* a switch of buffer pointers will result in these messages being available for
* consumption. The consumer then proceeds to process the entire buffer in one go.
* While this is ongoing, the producer buffer is being filled with new messages.
*
* For SQLite implementation of storage, consumer callback will write consumer buffer
* in each loop iteration as a separate transaction. This results in a balancing
* mechanism for high-performance cases, where transaction size can be increased
* dynamically as previous, smaller transactions introduce delays in loop iteration.
*/
class ROSBAG2_CPP_PUBLIC CacheConsumer
{
public:
  using consume_callback_function_t = std::function<void (const
      std::vector<buffer_element_t> &)>;

  CacheConsumer(
    std::shared_ptr<MessageCacheInterface> message_cache,
    consume_callback_function_t consume_callback);

  ~CacheConsumer();

  /// shut down the consumer thread
  void close();

  /// Set new consume callback, restart thread if necessary
  void change_consume_callback(consume_callback_function_t callback);

private:
  std::shared_ptr<MessageCacheInterface> message_cache_;
  consume_callback_function_t consume_callback_;

  /// Write buffer data to a storage
  void exec_consuming();

  /// Consumer thread shutdown sync
  std::atomic_bool is_stop_issued_ {false};
  std::mutex consumer_mutex_;

  std::thread consumer_thread_;
};

}  // namespace cache
}  // namespace rosbag2_cpp

#ifdef _WIN32
# pragma warning(pop)
#endif

#endif  // ROSBAG2_CPP__CACHE__CACHE_CONSUMER_HPP_
