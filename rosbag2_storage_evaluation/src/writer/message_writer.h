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

#ifndef ROS2_ROSBAG_EVALUATION_MESSAGE_WRITER_H
#define ROS2_ROSBAG_EVALUATION_MESSAGE_WRITER_H

#include <vector>

#include "generators/message.h"

namespace ros2bag
{

class MessageWriter
{
public:
  MessageWriter() = default;

  virtual ~MessageWriter() = default;

  virtual void open() = 0;

  virtual void close() = 0;

  virtual void write(MessagePtr message) = 0;

  virtual void create_index() = 0;

  virtual void reset() = 0;
};

}

#endif //ROS2_ROSBAG_EVALUATION_MESSAGE_WRITER_H
