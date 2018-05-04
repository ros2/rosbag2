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

#include "message_generator.h"

#include <algorithm>
#include <vector>

using namespace ros2bag;

MessageGenerator::MessageGenerator(unsigned int loop_count, Specification const & msgs) :
  loop_count_(loop_count)
  , max_index_(msgs.size())
  , current_loop_(0)
  , current_index_(0)
  , total_msg_count_(loop_count * msgs.size())
{
  topics_ = std::vector<std::string>(msgs.size());
  blobs_ = std::vector<BlobPtr>(msgs.size());

  std::string topic;
  unsigned int blob_size;
  for (int i = 0; i < msgs.size(); ++i) {
    std::tie(topic, blob_size) = msgs[i];

    topics_[i] = topic;
    blobs_[i] = random_blob(blob_size);
  }
}

bool MessageGenerator::has_next() const
{
  return current_loop_ < loop_count_ && current_index_ < max_index_;
}

MessagePtr MessageGenerator::next()
{
  auto topic = topics_[current_index_];
  auto blob = blobs_[current_index_];

  auto timestamp = std::chrono::system_clock::now();
  auto msg = std::make_shared<Message const>(timestamp, topic, blob);

  ++current_index_;
  if (current_index_ >= max_index_) {
    current_index_ = 0;
    ++current_loop_;
  }

  return msg;
}

void MessageGenerator::reset()
{
  current_index_ = 0;
  current_loop_ = 0;
}

BlobPtr MessageGenerator::random_blob(unsigned int blob_size) const
{
  auto blobPtr = std::make_shared<std::vector<unsigned char>>(blob_size);
  std::generate(blobPtr->begin(), blobPtr->end(), std::rand);

  return blobPtr;
}
