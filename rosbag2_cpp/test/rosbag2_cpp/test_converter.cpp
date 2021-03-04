// Copyright 2021 Amazon.com Inc. or its affiliates. All rights reserved.
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
#include <string>

#include "rosbag2_cpp/converter.hpp"

using namespace ::testing;  // NOLINT

class ConverterTest : public Test
{
public:
};

TEST_F(ConverterTest, gets_converter_impls)
{
  rosbag2_cpp::Converter conv{"cdr", "cdr"};
}
