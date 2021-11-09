// Copyright 2021 Open Source Robotics Foundation, Inc.
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

#ifndef ROSBAG2_CPP__BAG_EVENTS_HPP_
#define ROSBAG2_CPP__BAG_EVENTS_HPP_

#include <functional>
#include <memory>
#include <string>

#include "rclcpp/function_traits.hpp"

#include "rosbag2_cpp/visibility_control.hpp"

namespace rosbag2_cpp
{

namespace bag_events
{

enum class BagEvent
{
  WRITE_SPLIT,
  READ_SPLIT,
};

struct WriteSplitInfo
{
  std::string closed_file;
  std::string opened_file;
};

struct ReadSplitInfo
{
  std::string closed_file;
  std::string opened_file;
};

using WriteSplitCallbackType = std::function<void (WriteSplitInfo &)>;
using ReadSplitCallbackType = std::function<void (ReadSplitInfo &)>;

struct WriterEventCallbacks
{
  WriteSplitCallbackType write_split_callback;
};

struct ReaderEventCallbacks
{
  ReadSplitCallbackType read_split_callback;
};

class BagEventCallbackBase
{
public:
  using SharedPtr = std::shared_ptr<BagEventCallbackBase>;
  using InfoPtr = std::shared_ptr<void>;

  virtual void execute(InfoPtr & info) = 0;

  virtual bool is_type(BagEvent event) const = 0;
};

template<typename EventCallbackT>
class BagEventCallback : public BagEventCallbackBase
{
public:
  BagEventCallback(const EventCallbackT & callback, BagEvent event)
  : callback_(callback),
    event_(event)
  {}

  void execute(InfoPtr & info) override {
    callback_(*std::static_pointer_cast<EventCallbackInfoT>(info));
  }

  bool is_type(BagEvent event) const override {
    return event == event_;
  }

private:
  using EventCallbackInfoT = typename std::remove_reference<typename
      rclcpp::function_traits::function_traits<EventCallbackT>::template argument_type<0>>::type;

  EventCallbackT callback_;
  BagEvent event_;
};

class EventCallbackManager
{
public:
  template<typename EventCallbackT>
  void add_event_callback(
    const EventCallbackT & callback,
    const BagEvent event)
  {
    auto cb = std::make_shared<BagEventCallback<EventCallbackT>>(callback, event);
    callbacks_.push_back(cb);
  }

  void execute_callbacks(const BagEvent event, BagEventCallbackBase::InfoPtr info) {
    std::for_each(
      callbacks_.begin(),
      callbacks_.end(),
      [&info, &event](BagEventCallbackBase::SharedPtr & cb) {
        if (cb->is_type(event)) {
          cb->execute(info);
        }
      });
  }

private:
  std::vector<BagEventCallbackBase::SharedPtr> callbacks_;
};

}  // namespace bag_events

}  // namespace rosbag2_cpp

#endif  // ROSBAG2_CPP__BAG_EVENTS_HPP_
