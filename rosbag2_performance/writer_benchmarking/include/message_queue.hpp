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

#ifndef MESSAGE_QUEUE_HPP_
#define MESSAGE_QUEUE_HPP_

#include <string>
#include <queue>
#include <utility>
#include <mutex>
#include <iostream>

#include "std_msgs/msg/byte_multi_array.hpp"

template<typename T>
class MessageQueue
{
public:
  MessageQueue(int maxSize, std::string topicName)
    : mMaxSize(maxSize), mTopicName(topicName) {}

  void push(T elem)
  {
    std::lock_guard<std::mutex> lock(mMutex);
    if (mQueue.size() > mMaxSize)
    {   // We skip the element and consider it "lost"
      mUnsuccessfulInsertCount++;
      std::cerr << "X";
      return;
    }
    // std::cerr << "+";
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
    if (mQueue.empty())
    {
      throw std::out_of_range("Queue is empty, cannot pop. Check if empty first");
    }
    T elem = std::move(mQueue.front());
    // std::cerr << "-";
    mQueue.pop();   // safe with move
    return elem;
  }

  unsigned int getMissedElementsCount() const
  {
    std::lock_guard<std::mutex> lock(mMutex);
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
  unsigned int mUnsuccessfulInsertCount = 0;
  std::queue<T> mQueue;
  mutable std::mutex mMutex;
};

typedef MessageQueue<std_msgs::msg::ByteMultiArray> ByteMessageQueue;

#endif  // MESSAGE_QUEUE_HPP_
