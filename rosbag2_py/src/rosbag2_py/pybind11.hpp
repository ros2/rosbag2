// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#ifndef ROSBAG2_PY__PYBIND11_HPP_
#define ROSBAG2_PY__PYBIND11_HPP_

// Ignore -Wunused-value for clang.
// Based on https://github.com/pybind/pybind11/issues/2225
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunused-value"
#endif
#include <pybind11/pybind11.h>
#if defined(__clang__)
#pragma clang diagnostic pop
#endif
#include <pybind11/stl.h>

#endif  // ROSBAG2_PY__PYBIND11_HPP_
