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

#include <algorithm>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

#include "rosbag2_cpp/cache/message_cache.hpp"
#include "rosbag2_cpp/cache/message_cache_interface.hpp"
#include "rosbag2_cpp/logging.hpp"

namespace rosbag2_cpp
{
namespace cache
{

namespace
{
struct DropCounterOverflow
{
  std::unordered_map<std::string, uint32_t> high_words;
};

// The deleters share overflow words across swaps without changing either public class.
// The inner shared pointer retains the ordinary MessageCacheBuffer allocation and type.
struct TrackedBufferDeleter
{
  std::shared_ptr<MessageCacheBuffer> buffer;
  std::shared_ptr<DropCounterOverflow> overflow;

  void operator()(MessageCacheBuffer * pointer) noexcept
  {
    static_cast<void>(pointer);
    buffer.reset();
    overflow.reset();
  }
};

std::shared_ptr<MessageCacheBuffer> make_tracked_buffer(
  size_t max_size, uint32_t max_duration, const std::shared_ptr<DropCounterOverflow> & overflow)
{
  auto buffer = std::make_shared<MessageCacheBuffer>(max_size, max_duration);
  return std::shared_ptr<MessageCacheBuffer>(buffer.get(), TrackedBufferDeleter{buffer, overflow});
}

CacheDropStatistics make_statistics(
  const std::map<std::string, uint32_t> & sorted_lows,
  const DropCounterOverflow & overflow)
{
  CacheDropStatistics result;
  result.status = CacheDropStatisticsStatus::available;
  for (const auto & [topic, low] : sorted_lows) {
    auto high = overflow.high_words.find(topic);
    const uint64_t high_word = high == overflow.high_words.end() ? 0u : high->second;
    const uint64_t count = (high_word << 32) | low;
    result.dropped_per_topic.emplace(topic, count);
    if (count == std::numeric_limits<uint64_t>::max()) {
      result.status = CacheDropStatisticsStatus::counter_saturated;
    }
  }
  return result;
}
}  // namespace

MessageCache::MessageCache(size_t max_buffer_size, uint32_t max_buffer_duration)
{
  auto overflow = std::make_shared<DropCounterOverflow>();
  producer_buffer_ = make_tracked_buffer(
    max_buffer_size, max_buffer_duration, overflow);
  consumer_buffer_ = make_tracked_buffer(
    max_buffer_size, max_buffer_duration, overflow);
}

MessageCache::~MessageCache()
{
  // Initiate flushing on destruction to unblock wait_for_data. This is defensive programming
  // and in regular flow upper object owner should take care about unblocking wait_for_data in
  // exceptional situations.
  flushing_ = true;
  cache_condition_var_.notify_one();
  MessageCache::log_dropped();
}

bool MessageCache::push(std::shared_ptr<const rosbag2_storage::SerializedBagMessage> msg)
{
  if (!msg) {
    ROSBAG2_CPP_LOG_ERROR("Attempted to push null message into cache. Dropping message!");
    return false;
  }
  // While pushing, we keep track of inserted and dropped messages as well
  bool pushed = false;
  {
    std::lock_guard<std::mutex> lock(producer_buffer_mutex_);
    pushed = producer_buffer_->push(msg);
    data_ready_ = true;  // Don't use notify_data_ready() here for the sake of performance.
    if (!pushed) {
      auto & low = messages_dropped_per_topic_[msg->topic_name];
      if (low == std::numeric_limits<uint32_t>::max()) {
        auto & high = std::get_deleter<TrackedBufferDeleter>(producer_buffer_)->
          overflow->high_words[msg->topic_name];
        if (high < std::numeric_limits<uint32_t>::max()) {
          ++high;
          low = 0;
        }
      } else {
        ++low;
      }
    }
  }

  if (pushed) {
    // Notify the consumer that data is ready
    cache_condition_var_.notify_one();
  }
  return pushed;
}

std::shared_ptr<CacheBufferInterface> MessageCache::get_consumer_buffer()
{
  consumer_buffer_mutex_.lock();
  return consumer_buffer_;
}

void MessageCache::release_consumer_buffer()
{
  consumer_buffer_mutex_.unlock();
}

void MessageCache::notify_data_ready()
{
  {
    std::lock_guard<std::mutex> lock(producer_buffer_mutex_);
    data_ready_ = true;
  }
  cache_condition_var_.notify_one();
}

void MessageCache::wait_for_data()
{
  std::unique_lock<std::mutex> producer_lock(producer_buffer_mutex_);
  if (!flushing_) {
    // Required condition check to protect against spurious wakeups
    cache_condition_var_.wait(
      producer_lock, [this] {
        return data_ready_ || flushing_;
      });
    data_ready_ = false;
  }
}

size_t MessageCache::get_current_size()
{
  std::lock_guard<std::mutex> lock(producer_buffer_mutex_);
  return producer_buffer_->get_bytes_size();
}

void MessageCache::swap_buffers()
{
  std::lock_guard<std::mutex> producer_lock(producer_buffer_mutex_);
  std::lock_guard<std::mutex> consumer_lock(consumer_buffer_mutex_);
  std::swap(producer_buffer_, consumer_buffer_);
}

void MessageCache::begin_flushing()
{
  {
    std::lock_guard<std::mutex> lock(producer_buffer_mutex_);
    flushing_ = true;
  }
  cache_condition_var_.notify_one();
}

void MessageCache::done_flushing()
{
  flushing_ = false;
}

CacheDropStatistics MessageCache::get_dropped_statistics() const
{
  std::lock_guard<std::mutex> lock(producer_buffer_mutex_);
  const std::map<std::string, uint32_t> sorted_lows(
    messages_dropped_per_topic_.begin(), messages_dropped_per_topic_.end());
  return make_statistics(sorted_lows, *std::get_deleter<TrackedBufferDeleter>(producer_buffer_)->
           overflow);
}

void MessageCache::log_dropped()
{
  uint64_t total_lost = 0;
  bool total_saturated = false;
  std::string log_text("Cache buffers lost messages per topic: ");

  auto dropped = [this]() {
      std::lock_guard<std::mutex> lock(producer_buffer_mutex_);
      const std::map<std::string, uint32_t> sorted_lows(
        messages_dropped_per_topic_.begin(), messages_dropped_per_topic_.end());
      return make_statistics(sorted_lows, *std::get_deleter<TrackedBufferDeleter>(
          producer_buffer_)->overflow).dropped_per_topic;
    }();
  std::map<std::string, uint64_t> messages_dropped_per_topic_sorted(
    dropped.begin(), dropped.end());

  std::for_each(
    messages_dropped_per_topic_sorted.begin(),
    messages_dropped_per_topic_sorted.end(),
    [&total_lost, &total_saturated, &log_text](const auto & e) {
      uint64_t lost = e.second;
      if (lost > 0) {
        const bool saturated = lost == std::numeric_limits<uint64_t>::max();
        const auto count_text = lost <= std::numeric_limits<uint32_t>::max() ?
        std::to_string(static_cast<uint32_t>(lost)) : std::to_string(lost);
        log_text += "\n\t" + e.first + (saturated ? ": >= " : ": ") + count_text;
        if (saturated || lost > std::numeric_limits<uint64_t>::max() - total_lost) {
          total_lost = std::numeric_limits<uint64_t>::max();
          total_saturated = true;
        } else {
          total_lost += lost;
        }
      }
    });

  if (total_lost > 0) {
    log_text += std::string("\nTotal lost: ") + (total_saturated ? ">= " : "") +
      std::to_string(total_lost);
    ROSBAG2_CPP_LOG_WARN_STREAM(log_text);
  }

  size_t remaining = producer_buffer_->size() + consumer_buffer_->size();
  if (remaining > 0) {
    ROSBAG2_CPP_LOG_WARN_STREAM(
      "Cache buffers were unflushed with " << remaining << " remaining messages"
    );
  }
}

}  // namespace cache
}  // namespace rosbag2_cpp
