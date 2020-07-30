// Copyright 2020, Robotec.ai sp. z o.o.
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

#ifndef ROSBAG2_PERFORMAMCE_WRITER_BENCHMARKING__MESSAGE_QUEUE_HPP_
#define ROSBAG2_PERFORMAMCE_WRITER_BENCHMARKING__MESSAGE_QUEUE_HPP_

#include <string>
#include <queue>
#include <utility>
#include <mutex>
#include <atomic>
#include <iostream>
#include <memory>

#include "std_msgs/msg/byte_multi_array.hpp"

template<typename T>
class MessageQueue
{
public:
  MessageQueue(int maxSize, std::string topicName)
  : mMaxSize(maxSize), mTopicName(topicName), mUnsuccessfulInsertCount(0) {}

  void push(T elem)
  {
    std::lock_guard<std::mutex> lock(mMutex);
    if (mQueue.size() > mMaxSize) {  // We skip the element and consider it "lost"
      ++mUnsuccessfulInsertCount;
      std::cerr << "X" << std::flush;
      return;
    }
    mQueue.push(elem);
  }

  bool isComplete() const
  {
    return mComplete;
  }

  void setComplete()
  {
    mComplete = true;
  }

  bool isEmpty()
  {
    std::lock_guard<std::mutex> lock(mMutex);
    return mQueue.empty();
  }

  T pop_and_return()
  {
    std::lock_guard<std::mutex> lock(mMutex);
    if (mQueue.empty()) {
      throw std::out_of_range("Queue is empty, cannot pop. Check if empty first");
    }
    T elem = mQueue.front();
    mQueue.pop();
    return elem;
  }

  unsigned int getMissedElementsCount() const
  {
    return mUnsuccessfulInsertCount;
  }

  std::string topicName() const
  {
    return mTopicName;
  }

private:
  bool mComplete = false;
  unsigned int mMaxSize;
  std::string mTopicName;
  std::atomic<unsigned int> mUnsuccessfulInsertCount;
  std::queue<T> mQueue;
  mutable std::mutex mMutex;
};

typedef MessageQueue<std::shared_ptr<std_msgs::msg::ByteMultiArray>> ByteMessageQueue;

#endif  // ROSBAG2_PERFORMAMCE_WRITER_BENCHMARKING__MESSAGE_QUEUE_HPP_
