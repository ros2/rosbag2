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

#ifndef ROSBAG2_TRANSPORT__TIME_TRANSLATOR_HPP_
#define ROSBAG2_TRANSPORT__TIME_TRANSLATOR_HPP_

#include <chrono>

namespace rosbag2_transport
{

//! Helper class for translating between two times
/*!
 * The time translator can be configured with a Real start time, a
 * Translated start time, and a time scale.
 * 
 * It will convert a time from a series starting at realStartTime to a
 * comparable time series instead starting at translatedStartTime.
 */
class TimeTranslator
{
public:
    TimeTranslator();

    void setTimeScale(double const& scale);
    void setRealStartTime(
      std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> const& t);
    void setTranslatedStartTime(
      std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> const& t);  //!< Increments the translated start time by shift.  Useful for pausing.
    void shift(std::chrono::duration<int, std::nano> const& d);               //!< Increments the translated start time by shift.  Useful for pausing.
    void startPause();
    void endPause();
    std::chrono::time_point<std::chrono::system_clock, std::chrono::duration<double, std::nano> > translate(std::chrono::nanoseconds const& t);

private:
    double scale_;
    std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> pause_start_;
    std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> real_start_;
    std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> translated_start_;
};

} // namespace rosbag2_transport

#endif // ROSBAG2_TRANSPORT__TIME_TRANSLATOR_HPP_
