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

#ifndef ROS2_ROSBAG_EVALUATION_MESSAGE_GENERATOR_H
#define ROS2_ROSBAG_EVALUATION_MESSAGE_GENERATOR_H

#include <vector>

#include "generators/message.h"

namespace ros2bag
{

class MessageGenerator
{
public:

  using Specification = std::vector<std::tuple<std::string, unsigned int>>;

  MessageGenerator(unsigned int loop_count, Specification const & msgs);

  bool has_next() const;

  MessagePtr next();

  void reset();

  unsigned long total_msg_count()
  {
    return total_msg_count_;
  }

private:
  unsigned int const loop_count_;
  unsigned int current_loop_;
  unsigned long current_index_;
  unsigned long const max_index_;
  unsigned long const total_msg_count_;
  std::vector<std::string> topics_;
  std::vector<BlobPtr> blobs_;

  BlobPtr random_blob(unsigned int blob_size) const;
};

}

#endif //ROS2_ROSBAG_EVALUATION_MESSAGE_GENERATOR_H
