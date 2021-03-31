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

namespace py = pybind11;
typedef std::unordered_map<std::string, rclcpp::QoS> QoSMap;

namespace
{

rclcpp::QoS qos_from_handle(const py::handle source)
{
  auto py_capsule = PyObject_CallMethod(source.ptr(), "get_c_qos_profile", "");
  const auto rmw_qos_profile = reinterpret_cast<rmw_qos_profile_t *>(
    PyCapsule_GetPointer(py_capsule, "rmw_qos_profile_t"));
  const auto qos_init = rclcpp::QoSInitialization::from_rmw(*rmw_qos_profile);
  return rclcpp::QoS{qos_init, *rmw_qos_profile};
}

QoSMap qos_map_from_py_dict(const py::dict & dict)
{
  QoSMap value;
  for (const auto & item : dict) {
    auto key = std::string(py::str(item.first));
    value.insert({key, qos_from_handle(item.second)});
  }
  return value;
}

template<class T>
struct OptionsWrapper : public T
{
public:
  OptionsWrapper(T base) : T(base) {}

  void setTopicQoSProfileOverrides(
    const py::dict & overrides)
  {
    this->topic_qos_profile_overrides = qos_map_from_py_dict(overrides);
  }

  const py::dict & getTopicQoSProfileOverrides()
  {
    return py_dict;
  }

  py::dict py_dict;
};
typedef OptionsWrapper<rosbag2_transport::PlayOptions> PlayOptions;
typedef OptionsWrapper<rosbag2_transport::RecordOptions> RecordOptions;

}  // namespace

namespace rosbag2_py
{

class Player
{
public:
  Player() = default;
  virtual ~Player() = default;

  void play(
    const rosbag2_storage::StorageOptions & storage_options,
    PlayOptions & play_options)
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

    rosbag2_transport::Rosbag2Transport impl(reader, writer);
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
    RecordOptions & record_options)
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

  py::class_<PlayOptions>(m, "PlayOptions")
  .def(
    py::init([](
      size_t read_ahead_queue_size,
      std::string node_prefix,
      float rate,
      std::vector<std::string> topics_to_filter,
      py::dict topic_qos_profile_overrides,
      bool loop,
      std::vector<std::string> topic_remapping_options
    ){
      return new PlayOptions({
        read_ahead_queue_size,
        node_prefix,
        rate,
        topics_to_filter,
        qos_map_from_py_dict(topic_qos_profile_overrides),
        loop,
        topic_remapping_options
      });
    }),
    py::arg("read_ahead_queue_size"),
    py::arg("node_prefix") = "",
    py::arg("rate") = 1.0,
    py::arg("topics_to_filter") = std::vector<std::string>{},
    py::arg("topic_qos_profile_overrides") = py::dict{},
    py::arg("loop") = false,
    py::arg("topic_remapping_options") = std::vector<std::string>{}
  )
  .def_readwrite("read_ahead_queue_size", &PlayOptions::read_ahead_queue_size)
  .def_readwrite("node_prefix", &PlayOptions::node_prefix)
  .def_readwrite("rate", &PlayOptions::rate)
  .def_readwrite("topics_to_filter", &PlayOptions::topics_to_filter)
  .def_property(
    "topic_qos_profile_overrides",
    &PlayOptions::getTopicQoSProfileOverrides,
    &PlayOptions::setTopicQoSProfileOverrides)
  .def_readwrite("loop", &PlayOptions::loop)
  .def_readwrite("topic_remapping_options", &PlayOptions::topic_remapping_options)
  ;

  py::class_<RecordOptions>(m, "RecordOptions")
  .def(
    py::init([](
      bool all,
      bool is_discovery_disabled,
      std::vector<std::string> topics,
      std::string rmw_serialization_format,
      std::chrono::milliseconds topic_polling_interval,
      std::string regex,
      std::string exclude,
      std::string node_prefix,
      std::string compression_mode,
      std::string compression_format,
      uint64_t compression_queue_size,
      uint64_t compression_threads,
      py::dict topic_qos_profile_overrides,
      bool include_hidden_topics
    ) {
      return new RecordOptions({
        all,
        is_discovery_disabled,
        topics,
        rmw_serialization_format,
        topic_polling_interval,
        regex,
        exclude,
        node_prefix,
        compression_mode,
        compression_format,
        compression_queue_size,
        compression_threads,
        qos_map_from_py_dict(topic_qos_profile_overrides),
        include_hidden_topics
      });
    }),
    py::arg("all") = false,
    py::arg("is_discovery_disabled") = true,
    py::arg("topics") = std::vector<std::string>{},
    py::arg("rmw_serialization_format") = "",
    py::arg("topic_polling_interval") = std::chrono::milliseconds(100),
    py::arg("regex") = "",
    py::arg("exclude") = "",
    py::arg("node_prefix") = "",
    py::arg("compression_mode") = "",
    py::arg("compression_format") = "",
    py::arg("compression_queue_size") = 1,
    py::arg("compression_threads") = 0,
    py::arg("topic_qos_profile_overrides") = py::dict{},
    py::arg("include_hidden_topics") = false
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
  .def_property(
    "topic_qos_profile_overrides",
    &RecordOptions::getTopicQoSProfileOverrides,
    &RecordOptions::setTopicQoSProfileOverrides)
  .def_readwrite("include_hidden_topics", &RecordOptions::include_hidden_topics)
  ;

  py::class_<rosbag2_py::Player>(m, "Player")
  .def(py::init())
  .def("play", &rosbag2_py::Player::play)
  ;

  py::class_<rosbag2_py::Recorder>(m, "Recorder")
  .def(py::init())
  .def("record", &rosbag2_py::Recorder::record)
  ;
}
