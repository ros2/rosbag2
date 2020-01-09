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

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

#include <chrono>
#include <string>

#include "rosbag2/readers/sequential_reader.hpp"
// #include "rmw/rmw.h"

namespace rosbag2_py {

class SequentialReader
{
public:
  SequentialReader() : reader_()
  {}

  void open(const std::string uri)
  {
    // TODO(jacobperron): pybind StorageOptions nad ConverterOptions and pass to open
    rosbag2::StorageOptions storage_options{};
    storage_options.uri = uri;
    storage_options.storage_id = "sqlite3";
    rosbag2::ConverterOptions converter_options{};
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";
    reader_.open(storage_options, converter_options);
  }

  /// Return a pair containing the topic name and the serialized ROS message.
  std::pair<std::string, py::bytes> read_next() {
    const auto next = reader_.read_next();
    rcutils_uint8_array_t rcutils_data = *next->serialized_data.get();
    std::string serialized_data(rcutils_data.buffer, rcutils_data.buffer + rcutils_data.buffer_length);
    // TODO(jacobperron): Also consider returning the timestamp, next->time_stamp
    return std::make_pair(next->topic_name, py::bytes(serialized_data));
  }

  bool has_next() {
    return reader_.has_next();
  }

  /// Return a mapping from topic name to topic type.
  std::map<std::string, std::string> get_all_topics_and_types() {
    // TODO(jacobperron): pybind TopicMetadata and return the vector of it directly
     auto topics = reader_.get_all_topics_and_types();
     // We're making an assumption that there is only one type per topic
     std::map<std::string, std::string> output;
     for (const rosbag2::TopicMetadata & topic_metadata : topics) {
       output[topic_metadata.name] = topic_metadata.type;
     }
     return output;
  }
private:
  rosbag2::readers::SequentialReader reader_;
};

}
PYBIND11_MODULE(_rosbag2_py, m) {
  m.doc() = "Python wrapper of rosbag2 implementation";

  py::class_<rosbag2_py::SequentialReader>(m, "SequentialReader")
    .def(py::init())
    .def("open", &rosbag2_py::SequentialReader::open)
    .def("read_next", &rosbag2_py::SequentialReader::read_next)
    .def("has_next", &rosbag2_py::SequentialReader::has_next)
    .def("get_all_topics_and_types", &rosbag2_py::SequentialReader::get_all_topics_and_types);
}
