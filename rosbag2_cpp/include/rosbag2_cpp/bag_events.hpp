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
#include <vector>

#include "rclcpp/function_traits.hpp"
#include "rosbag2_cpp/visibility_control.hpp"

namespace rosbag2_cpp
{

namespace bag_events
{

/**
 * \brief The types of bag events available for registering callbacks.
 */
enum class BagEvent
{
  /// The output bag file has been split, starting a new file.
  WRITE_SPLIT,
  /// Reading of the input bag file has gone over a split, opening the next file.
  READ_SPLIT,
};

/**
 * \brief The information structure passed to callbacks for the WRITE_SPLIT and READ_SPLIT events.
 */
struct BagSplitInfo
{
  /// The URI of the file that was closed.
  std::string closed_file;
  /// The URI of the file that was opened.
  std::string opened_file;
};

using BagSplitCallbackType = std::function<void (BagSplitInfo &)>;

/**
 * \brief Use this structure to register callbacks with Writers.
 */
struct WriterEventCallbacks
{
  /// The callback to call for the WRITE_SPLIT event.
  BagSplitCallbackType write_split_callback;
};

/**
 * \brief Use this structure to register callbacks with Readers.
 */
struct ReaderEventCallbacks
{
  /// The callback to call for the READ_SPLIT event.
  BagSplitCallbackType read_split_callback;
};

/**
 * \brief Base class for event callbacks.
 *
 * This class should not be used directly.
 */
class BagEventCallbackBase
{
public:
  using SharedPtr = std::shared_ptr<BagEventCallbackBase>;
  using InfoPtr = std::shared_ptr<void>;

  virtual ~BagEventCallbackBase()
  {}

  virtual void execute(InfoPtr & info) = 0;

  virtual bool is_type(BagEvent event) const = 0;
};

/**
 * \brief Templated class for storing an event callback.
 *
 * This class should not be used directly by users.
 */
template<typename EventCallbackT>
class BagEventCallback : public BagEventCallbackBase
{
public:
  BagEventCallback(const EventCallbackT & callback, BagEvent event)
  : callback_(callback),
    event_(event)
  {}

  virtual ~BagEventCallback()
  {}

  void execute(InfoPtr & info) override
  {
    callback_(*std::static_pointer_cast<EventCallbackInfoT>(info));
  }

  bool is_type(BagEvent event) const override
  {
    return event == event_;
  }

private:
  using EventCallbackInfoT = typename std::remove_reference<typename
      rclcpp::function_traits::function_traits<EventCallbackT>::template argument_type<0>>::type;

  EventCallbackT callback_;
  BagEvent event_;
};

/**
 * \brief The class used to manage event callbacks registered with a Writer or Reader.
 *
 * Each implementation of the Writer and Reader interfaces should store one instance of this type.
 * When new callbacks are registered, they should be passed to that instance using \ref
 * add_event_callback. When an event occurs, the callbacks registered for it can be called by
 * calling the \ref execute_callbacks method, passing in the event information.
 */
class EventCallbackManager
{
public:
  /**
   * \brief Add an event callback.
   *
   * \param callback The callback that should be called for the event.
   * \param event The event, one of the values of the \ref BagEvent enumeration.
   */
  template<typename EventCallbackT>
  void add_event_callback(const EventCallbackT & callback, const BagEvent event)
  {
    auto cb = std::make_shared<BagEventCallback<EventCallbackT>>(callback, event);
    callbacks_.push_back(cb);
  }

  /**
   * \brief Check if a callback is registered for the given event.
   *
   * \param event The event, one of the values of the \ref BagEvent enumeration.
   * \return True if a callback is registered for the event, false otherwise.
   */
  bool has_callback_for_event(const BagEvent event) const
  {
    for (auto & cb : callbacks_) {
      if (cb->is_type(event)) {
        return true;
      }
    }
    return false;
  }

  /**
   * \brief Execute all callbacks registered for the given event.
   *
   * The provided information value is passed to each callback by copy.
   *
   * \param event The event, one of the values of the \ref BagEvent enumeration.
   * \param info The information relevant to the event that has occurred. The type varies by event.
   */
  void execute_callbacks(const BagEvent event, BagEventCallbackBase::InfoPtr info)
  {
    for (auto & cb : callbacks_) {
      if (cb->is_type(event)) {
        cb->execute(info);
      }
    }
  }

private:
  std::vector<BagEventCallbackBase::SharedPtr> callbacks_;
};

}  // namespace bag_events

}  // namespace rosbag2_cpp

#endif  // ROSBAG2_CPP__BAG_EVENTS_HPP_
