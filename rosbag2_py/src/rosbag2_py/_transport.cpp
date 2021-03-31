// Copyright 2021 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "rosbag2_compression/compression_options.hpp"
#include "rosbag2_compression/sequential_compression_reader.hpp"
#include "rosbag2_compression/sequential_compression_writer.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_cpp/writers/sequential_writer.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "rosbag2_transport/play_options.hpp"
#include "rosbag2_transport/record_options.hpp"
#include "rosbag2_transport/rosbag2_transport.hpp"

#include "./pybind11.hpp"

using PlayOptions = rosbag2_transport::PlayOptions;
using RecordOptions = rosbag2_transport::RecordOptions;
using Rosbag2Transport = rosbag2_transport::Rosbag2Transport;

namespace rosbag2_py
{

class Player
{
public:
  Player() = default;
  virtual ~Player() = default;

  void play(
    const rosbag2_storage::StorageOptions & storage_options,
    const rosbag2_transport::PlayOptions & play_options)
  {
    auto writer = std::make_shared<rosbag2_cpp::Writer>(
      std::make_unique<rosbag2_cpp::writers::SequentialWriter>());
    std::shared_ptr<rosbag2_cpp::Reader> reader = nullptr;
    // Determine whether to build compression or regular reader
    {
      rosbag2_storage::MetadataIo metadata_io{};
      rosbag2_storage::BagMetadata metadata{};
      if (metadata_io.metadata_file_exists(storage_options.uri)) {
        metadata = metadata_io.read_metadata(storage_options.uri);
        if (!metadata.compression_format.empty()) {
          reader = std::make_shared<rosbag2_cpp::Reader>(
            std::make_unique<rosbag2_compression::SequentialCompressionReader>());
        }
      }
      if (reader == nullptr) {
        reader = std::make_shared<rosbag2_cpp::Reader>(
          std::make_unique<rosbag2_cpp::readers::SequentialReader>());
      }
    }

    Rosbag2Transport impl(reader, writer);
    impl.init();
    impl.play(storage_options, play_options);
    impl.shutdown();
  }
};

class Recorder
{
public:
  Recorder() = default;
  virtual ~Recorder() = default;

  void record(
    const rosbag2_storage::StorageOptions & storage_options,
    rosbag2_transport::RecordOptions & record_options)
  {
    rosbag2_compression::CompressionOptions compression_options {
      record_options.compression_format,
      rosbag2_compression::compression_mode_from_string(record_options.compression_mode),
      record_options.compression_queue_size,
      record_options.compression_threads
    };
    if (compression_options.compression_threads < 1) {
      compression_options.compression_threads = std::thread::hardware_concurrency();
    }

    if (record_options.rmw_serialization_format.empty()) {
      record_options.rmw_serialization_format = std::string(rmw_get_serialization_format());
    }


    auto reader = std::make_shared<rosbag2_cpp::Reader>(
      std::make_unique<rosbag2_cpp::readers::SequentialReader>());
    std::shared_ptr<rosbag2_cpp::Writer> writer;
    // Change writer based on recording options
    if (!record_options.compression_format.empty()) {
      writer = std::make_shared<rosbag2_cpp::Writer>(
        std::make_unique<rosbag2_compression::SequentialCompressionWriter>(compression_options));
    } else {
      writer = std::make_shared<rosbag2_cpp::Writer>(
        std::make_unique<rosbag2_cpp::writers::SequentialWriter>());
    }

    rosbag2_transport::Rosbag2Transport impl(reader, writer);
    impl.init();
    impl.record(storage_options, record_options);
    impl.shutdown();
  }
};

}  // namespace rosbag2_py

PYBIND11_MODULE(_transport, m) {
  m.doc() = "Python wrapper of the rosbag2_transport API";

  pybind11::class_<PlayOptions>(m, "PlayOptions")
  .def(
    pybind11::init<
      size_t,
      std::string,
      float,
      std::vector<std::string>,
      std::unordered_map<std::string, rclcpp::QoS>,
      bool,
      std::vector<std::string>
    >(),
    pybind11::arg("read_ahead_queue_size"),
    pybind11::arg("node_prefix") = "",
    pybind11::arg("rate") = 1.0,
    pybind11::arg("topics_to_filter") = std::vector<std::string>{},
    pybind11::arg("topic_qos_profile_overrides") = std::unordered_map<std::string, rclcpp::QoS>{},
    pybind11::arg("loop") = false,
    pybind11::arg("topic_remapping_options") = std::vector<std::string>{}
  )
  .def_readwrite("read_ahead_queue_size", &PlayOptions::read_ahead_queue_size)
  .def_readwrite("node_prefix", &PlayOptions::node_prefix)
  .def_readwrite("rate", &PlayOptions::rate)
  .def_readwrite("topics_to_filter", &PlayOptions::topics_to_filter)
  .def_readwrite("topic_qos_profile_overrides", &PlayOptions::topic_qos_profile_overrides)
  .def_readwrite("loop", &PlayOptions::loop)
  .def_readwrite("topic_remapping_options", &PlayOptions::topic_remapping_options)
  ;

  pybind11::class_<RecordOptions>(m, "RecordOptions")
  .def(
    pybind11::init<
      bool,
      bool,
      std::vector<std::string>,
      std::string,
      std::chrono::milliseconds,
      std::string,
      std::string,
      std::string,
      std::string,
      std::string,
      uint64_t,
      uint64_t,
      std::unordered_map<std::string, rclcpp::QoS>,
      bool
    >(),
    pybind11::arg("all") = false,
    pybind11::arg("is_discovery_disabled") = true,
    pybind11::arg("topics") = std::vector<std::string>{},
    pybind11::arg("rmw_serialization_format") = "",
    pybind11::arg("topic_polling_interval") = std::chrono::milliseconds(100),
    pybind11::arg("regex") = "",
    pybind11::arg("exclude") = "",
    pybind11::arg("node_prefix") = "",
    pybind11::arg("compression_mode") = "",
    pybind11::arg("compression_format") = "",
    pybind11::arg("compression_queue_size") = 1,
    pybind11::arg("compression_threads") = 0,
    pybind11::arg("topic_qos_profile_overrides") = std::unordered_map<std::string, rclcpp::QoS>{},
    pybind11::arg("include_hidden_topics") = false
  )
  .def_readwrite("all", &RecordOptions::all)
  .def_readwrite("is_discovery_disabled", &RecordOptions::is_discovery_disabled)
  .def_readwrite("topics", &RecordOptions::topics)
  .def_readwrite("rmw_serialization_format", &RecordOptions::rmw_serialization_format)
  .def_readwrite("topic_polling_interval", &RecordOptions::topic_polling_interval)
  .def_readwrite("regex", &RecordOptions::regex)
  .def_readwrite("exclude", &RecordOptions::exclude)
  .def_readwrite("node_prefix", &RecordOptions::node_prefix)
  .def_readwrite("compression_mode", &RecordOptions::compression_mode)
  .def_readwrite("compression_format", &RecordOptions::compression_format)
  .def_readwrite("compression_queue_size", &RecordOptions::compression_queue_size)
  .def_readwrite("compression_threads", &RecordOptions::compression_threads)
  .def_readwrite("topic_qos_profile_overrides", &RecordOptions::topic_qos_profile_overrides)
  .def_readwrite("include_hidden_topics", &RecordOptions::include_hidden_topics)
  ;

  pybind11::class_<rosbag2_py::Player>(m, "Player")
  .def(pybind11::init())
  .def("play", &rosbag2_py::Player::play)
  ;

  pybind11::class_<rosbag2_py::Recorder>(m, "Recorder")
  .def(pybind11::init())
  .def("record", &rosbag2_py::Recorder::record)
  ;
}
