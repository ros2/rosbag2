/*
 *  Copyright (c) 2018,  Bosch Software Innovations GmbH.
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */

#ifndef ROS2_ROSBAG_EVALUATION_MESSAGE_H
#define ROS2_ROSBAG_EVALUATION_MESSAGE_H

#include <iostream>
#include <memory>
#include <utility>
#include <vector>
#include <chrono>

namespace ros2bag
{

using BlobPtr = std::shared_ptr<std::vector<unsigned char> const>;

class Message
{
public:
  using Timestamp = std::chrono::system_clock::time_point;

  Message(Timestamp const & timestamp, std::string const & topic, BlobPtr blob)
    : timestamp_(timestamp), topic_(topic), blob_(blob)
  {}

  ~Message() = default;

  Timestamp timestamp() const
  {
    return timestamp_;
  }

  std::string const topic() const
  {
    return topic_;
  }

  BlobPtr blob() const
  {
    return blob_;
  }


  friend std::ostream & operator<<(std::ostream & out_stream, Message const & message)
  {
    return out_stream << "{timestamp_:" << message.timestamp().time_since_epoch().count()
                      << ",topic:" << message.topic()
                      << ",bytes:" << message.blob()->size()
                      << "}";
  }

private:
  Timestamp const timestamp_;
  std::string const topic_;
  BlobPtr const blob_;
};

using MessagePtr = std::shared_ptr<Message const>;

}

#endif //ROS2_ROSBAG_EVALUATION_MESSAGE_H
