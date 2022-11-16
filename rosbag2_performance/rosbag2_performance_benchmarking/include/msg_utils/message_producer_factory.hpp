// Copyright 2022 Apex.AI, Inc.
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

#ifndef MSG_UTILS__MESSAGE_PRODUCER_FACTORY_HPP_
#define MSG_UTILS__MESSAGE_PRODUCER_FACTORY_HPP_

#include <memory>
#include <string>

#include "rosbag2_performance_benchmarking_msgs/msg/byte_array.hpp"

#include "message_producer.hpp"

#define ADD_MSG(space, type) \
  if (key == #space "::msg::"#type) { \
    using space::msg::type; \
    return std::make_shared<MessageProducer<type>>(args ...); \
  } else  // NOLINT  not closed `if` in macro is on purpose

#define ADD_BENCHMARKING_MSG(type) ADD_MSG(rosbag2_performance_benchmarking_msgs, type)

namespace msg_utils
{
template<typename ... Args>
std::shared_ptr<ProducerBase> create(std::string key, Args & ... args)
{
  ADD_BENCHMARKING_MSG(ByteArray)

  throw std::runtime_error("Unknown message type: " + key);
}

}  // namespace msg_utils

#endif  // MSG_UTILS__MESSAGE_PRODUCER_FACTORY_HPP_
