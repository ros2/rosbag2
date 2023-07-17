// Copyright 2023 Open Source Robotics Foundation, Inc.
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

#include <memory>
#include <string>

#include "rosbag2_compression/compression_options.hpp"

#include "./pybind11.hpp"

using CompressionMode = rosbag2_compression::CompressionMode;
using CompressionOptions = rosbag2_compression::CompressionOptions;

PYBIND11_MODULE(_compression_options, m) {
  m.doc() = "Python wrapper of the rosbag2_compression API";

  pybind11::enum_<CompressionMode>(m, "CompressionMode")
  .value("NONE", CompressionMode::NONE)
  .value("FILE", CompressionMode::FILE)
  .value("MESSAGE", CompressionMode::MESSAGE)
  .export_values();

  pybind11::class_<CompressionOptions>(m, "CompressionOptions")
  .def(
    pybind11::init<std::string &, CompressionMode, uint64_t &, uint64_t &>(),
    pybind11::arg("compression_format") = CompressionOptions{}.compression_format,
    pybind11::arg("compression_mode") = CompressionOptions{}.compression_mode,
    pybind11::arg("compression_queue_size") = CompressionOptions{}.compression_queue_size,
    pybind11::arg("compression_threads") = CompressionOptions{}.compression_threads)
  .def_readwrite("compression_format", &CompressionOptions::compression_format)
  .def_readwrite("compression_mode", &CompressionOptions::compression_mode)
  .def_readwrite("compression_queue_size", &CompressionOptions::compression_queue_size)
  .def_readwrite("compression_threads", &CompressionOptions::compression_threads);

  m.def(
    "compression_mode_from_string",
    &rosbag2_compression::compression_mode_from_string,
    "Converts a string into a rosbag2_compression::CompressionMode enum.");

  m.def(
    "compression_mode_to_string",
    &rosbag2_compression::compression_mode_to_string,
    "Converts a rosbag2_compression::CompressionMode enum into a string");
}
