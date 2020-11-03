/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include "rosbag2_transport/time_translator.hpp"

namespace rosbag2_transport {

TimeTranslator::TimeTranslator()
    : scale_(1.0),
    real_start_(std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>::min()),
    translated_start_(std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>::min())
{
}

void TimeTranslator::setTimeScale(double const& scale)
{
    scale_ = scale;
}

void TimeTranslator::setRealStartTime(
  std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> const& t)
{
    real_start_ = t;
}

void TimeTranslator::setTranslatedStartTime(
  std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> const& t)
{
    translated_start_ = t;
}

void TimeTranslator::startPause()
{
    pause_start_ = std::chrono::system_clock::now();
}

void TimeTranslator::endPause()
{
    translated_start_ += std::chrono::system_clock::now() - pause_start_;
}

void TimeTranslator::shift(std::chrono::duration<int, std::nano> const& d)
{
    translated_start_ += d;
}

std::chrono::time_point<std::chrono::system_clock, std::chrono::duration<double, std::nano> > TimeTranslator::translate(std::chrono::nanoseconds const& t)
{
    auto shifted_time = translated_start_ + (t - real_start_.time_since_epoch()) / scale_;
    return shifted_time;
}

} // namespace rosbag
