// Copyright 2024 Intrinsic Innovation LLC. All rights reserved.
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

#include "rosbag2_cpp/message_definitions/local_message_definition_source.hpp"

#include "./pybind11.hpp"

PYBIND11_MODULE(_message_definitions, m) {
  m.doc() = "Python wrapper of the rosbag2_cpp message definitions API";

  pybind11::class_<rosbag2_cpp::LocalMessageDefinitionSource>(
    m, "LocalMessageDefinitionSource")
  .def(pybind11::init<>())
  .def(
    "get_full_text", &rosbag2_cpp::LocalMessageDefinitionSource::get_full_text);
}
