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

#include <memory>

#include "rosbag2_cpp/clocks/sim_time_clock.hpp"

namespace rosbag2_cpp
{

class SimTimeClockImpl
{
public:
};

SimTimeClock::SimTimeClock()
: impl_(std::make_unique<SimTimeClockImpl>())
{}

SimTimeClock::~SimTimeClock()
{}

rcutils_time_point_value_t SimTimeClock::now() const
{
  // TODO(ek)
  return 0;
}

bool SimTimeClock::sleep_until(rcutils_time_point_value_t /* until */)
{
  // TODO(ek)
  return false;
}

void SimTimeClock::set_rate(double /* rate */)  // NOLINT (https://github.com/cpplint/cpplint/issues/131)
{
  // no-op
}

double SimTimeClock::get_rate() const
{
  // TODO(ek): best-guess?
  return 1.0;
}

void SimTimeClock::pause()
{
  // no-op
}

void SimTimeClock::resume()
{
  // no-op
}

bool SimTimeClock::is_paused() const
{
  // TODO(ek): best-guess?
  return false;
}

}  // namespace rosbag2_cpp
