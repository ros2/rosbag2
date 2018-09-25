// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#include <gmock/gmock.h>
#include <memory>

#include "rosbag2_storage/ros_helper.hpp"

using namespace ::testing;  // NOLINT

TEST(ros_helper, make_serialized_message_contains_correct_data) {
  double data_value = 3.14;
  auto data = new double;
  *data = data_value;
  auto size = sizeof(double);

  auto serialized_message = rosbag2_storage::make_serialized_message(data, size);
  delete data;

  ASSERT_THAT(serialized_message->buffer_length, Eq(size));
  ASSERT_THAT(serialized_message->buffer_capacity, Eq(size));
  ASSERT_THAT(reinterpret_cast<double *>(serialized_message->buffer), Pointee(data_value));
}
