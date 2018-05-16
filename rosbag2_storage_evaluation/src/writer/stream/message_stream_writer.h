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

#ifndef ROS2_ROSBAG_EVALUATION_MESSAGE_STREAM_WRITER_H
#define ROS2_ROSBAG_EVALUATION_MESSAGE_STREAM_WRITER_H

#include <ostream>

#include "writer/message_writer.h"

namespace ros2bag
{

class MessageStreamWriter : public MessageWriter
{
public:
  explicit MessageStreamWriter(std::ostream & output_stream) : output_stream_(output_stream)
  {}

  ~MessageStreamWriter() override = default;

  void open() override
  {}

  void close() override
  {}

  void write(MessagePtr message) override;

  void create_index() override
  {}

  void reset() override
  {}

private:
  std::ostream & output_stream_;
};

}

#endif //ROS2_ROSBAG_EVALUATION_MESSAGE_STREAM_WRITER_H
