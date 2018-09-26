// Copyright 2018, Bosch Software Innovations GmbH.
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

#include <string>
#include <thread>

#include "end_to_end_test_fixture.hpp"

using namespace ::testing;  // NOLINT
using namespace std::chrono_literals;  // NOLINT


TEST_F(EndToEndTestFixture, info_end_to_end_test) {
  internal::CaptureStdout();
  execute("ros2 bag info test.bag");
  std::string output = internal::GetCapturedStdout();

  // TODO(botteroa-si): update once correct pretty printing of baginfo is available.
  EXPECT_THAT(output, HasSubstr("printing bag info of 'test.bag'..."));
}
