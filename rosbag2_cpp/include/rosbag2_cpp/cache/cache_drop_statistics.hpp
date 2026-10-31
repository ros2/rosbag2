// Copyright 2026 The rosbag2 authors.
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

#ifndef ROSBAG2_CPP__CACHE__CACHE_DROP_STATISTICS_HPP_
#define ROSBAG2_CPP__CACHE__CACHE_DROP_STATISTICS_HPP_

#include <cstdint>
#include <string>
#include <unordered_map>

#include "rosbag2_cpp/visibility_control.hpp"

namespace rosbag2_cpp
{
namespace cache
{

enum class CacheDropStatisticsStatus
{
  /// Counts are available; absent topics have no counted failures.
  available,
  /// No message cache is configured.
  cache_disabled,
  /// This writer or cache does not implement the statistic.
  unsupported,
  /// The writer is not open and has no retained final snapshot.
  writer_not_open,
  /// Another Writer operation or a reentrant call prevents a safe snapshot; retry later.
  busy,
  /// At least one topic reached UINT64_MAX; that value is a lower bound.
  counter_saturated,
};

struct CacheDropStatistics
{
  CacheDropStatisticsStatus status{CacheDropStatisticsStatus::unsupported};
  /// Cumulative insertion failures by topic, not transport or storage losses.
  std::unordered_map<std::string, uint64_t> dropped_per_topic;
  /// True when these counts were captured while closing the recording.
  bool is_final{false};
};

/// Optional capability for custom writer or cache implementations.
/// Implementations must synchronize access to their own statistics and must not acquire
/// the enclosing Writer lock while holding locks needed by this query.
class ROSBAG2_CPP_PUBLIC CacheDropStatisticsProvider
{
public:
  virtual ~CacheDropStatisticsProvider() = default;
  virtual CacheDropStatistics get_cache_drop_statistics() const = 0;
};

}  // namespace cache
}  // namespace rosbag2_cpp

#endif  // ROSBAG2_CPP__CACHE__CACHE_DROP_STATISTICS_HPP_
