// Copyright 2021 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

// NOTE: This file is copied from Rolling rmw, as part of backporting bugfix #684 to Foxy

#include "rmw_time.h"  // NOLINT

#include "rcutils/time.h"

bool
rmw_time_equal(const rmw_time_t left, const rmw_time_t right)
{
  return rmw_time_total_nsec(left) == rmw_time_total_nsec(right);
}

rmw_duration_t
rmw_time_total_nsec(const rmw_time_t time)
{
  static const uint64_t max_sec = INT64_MAX / RCUTILS_S_TO_NS(1);
  if (time.sec > max_sec) {
    // Seconds not representable in nanoseconds
    return INT64_MAX;
  }

  const int64_t sec_as_nsec = RCUTILS_S_TO_NS(time.sec);
  if (time.nsec > (uint64_t)(INT64_MAX - sec_as_nsec)) {
    // overflow
    return INT64_MAX;
  }
  return sec_as_nsec + time.nsec;
}

rmw_time_t
rmw_time_from_nsec(const rmw_duration_t nanoseconds)
{
  if (nanoseconds < 0) {
    return (rmw_time_t)RMW_DURATION_INFINITE;
  }

  // Avoid typing the 1 billion constant
  rmw_time_t time;
  time.sec = RCUTILS_NS_TO_S(nanoseconds);
  time.nsec = nanoseconds % RCUTILS_S_TO_NS(1);
  return time;
}

rmw_time_t
rmw_time_normalize(const rmw_time_t time)
{
  return rmw_time_from_nsec(rmw_time_total_nsec(time));
}
